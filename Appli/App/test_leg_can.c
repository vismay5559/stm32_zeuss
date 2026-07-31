#include "test_leg_can.h"
#include "critical.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef   htim2;
extern TIM_HandleTypeDef   htim6;

/* ===================================================================== */
/*  CONFIGURATION - everything you are likely to change lives up here     */
/* ===================================================================== */

/*
 * ODrive node IDs, in joint order. Set these to match what you configured in
 * odrivetool (axis0.config.can.node_id). They must be unique on the bus.
 */
#define JOINT_COUNT      4

static const uint8_t     s_node_id[JOINT_COUNT] = { 1, 2, 3, 4 };
static const char *const s_joint_name[JOINT_COUNT] = {
    "hip_roll", "hip_pitch", "knee     ", "ankle    "
};

/*
 * SAFETY: with this at 0 the test never commands an axis into closed loop, so
 * the motors stay unpowered and cannot move no matter what positions are sent.
 * That makes it safe to run on a bench with the leg attached.
 *
 * Only set it to 1 once you have: the leg in a fixture and off the ground, a
 * physical e-stop within reach, and low current/velocity limits configured in
 * odrivetool. Bring up ONE node at a time before running all four.
 */
#define LEGTEST_ENABLE_CLOSED_LOOP   0

/* Motion profile. Only has any effect when closed loop is enabled above. */
#define LEGTEST_AMPLITUDE_TURNS      0.05f   /* +/- turns, after gearbox      */
#define LEGTEST_FREQ_HZ              0.25f   /* slow enough to watch          */

/*
 * ODrive S1 may or may not support CAN FD depending on firmware version - it
 * is worth confirming before trusting FD framing. Set this to 0 to fall back
 * to classic CAN 2.0 (still 1 Mbit, since classic uses only the nominal bit
 * timing) if the nodes do not respond in FD.
 */
#define LEGTEST_USE_CAN_FD           1

/* Consider a node dead if nothing has been heard from it for this long. */
#define NODE_SILENT_TICKS            500u    /* ms */

/* ===================================================================== */

/* ODrive CANSimple: arbitration id = (node_id << 5) | cmd_id */
#define ODRV_CMD_HEARTBEAT      0x001u
#define ODRV_CMD_SET_AXIS_STATE 0x007u
#define ODRV_CMD_GET_ENCODER    0x009u
#define ODRV_CMD_SET_INPUT_POS  0x00Cu
#define ODRV_CMD_GET_TORQUES    0x01Cu

#define ODRV_AXIS_STATE_IDLE            1u
#define ODRV_AXIS_STATE_CLOSED_LOOP     8u

typedef struct
{
    /* Most recent values received */
    float    pos;
    float    vel;
    float    torque;
    uint32_t axis_error;
    uint8_t  axis_state;

    /* Message counters - these are what actually prove the link works */
    uint32_t n_heartbeat;
    uint32_t n_encoder;
    uint32_t n_torque;

    uint32_t last_rx_tick;   /* tick of the most recent frame from this node */
    float    cmd;            /* last position we commanded                   */
} joint_t;

static volatile joint_t s_joint[JOINT_COUNT];
static volatile uint32_t s_rx_total;
static volatile uint32_t s_rx_unknown;   /* frames from unexpected node ids  */

static volatile uint32_t s_tick_pending;
static uint32_t s_tick;
static uint32_t s_tx_fail;

/* ------------------------------------------------------------------ */

static int joint_from_node(uint32_t node)
{
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (s_node_id[j] == node)
        {
            return j;
        }
    }
    return -1;
}

static float le_f32(const uint8_t *p)
{
    uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float f;
    memcpy(&f, &v, sizeof(f));
    return f;
}

static uint32_t le_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* ------------------------------------------------------------------ */
/*  Transmit                                                           */
/* ------------------------------------------------------------------ */

static uint8_t can_send(uint32_t node, uint32_t cmd, const uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef hdr;

    hdr.Identifier          = (node << 5) | cmd;
    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = (len == 8u) ? FDCAN_DLC_BYTES_8 : FDCAN_DLC_BYTES_4;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
#if LEGTEST_USE_CAN_FD
    hdr.BitRateSwitch       = FDCAN_BRS_ON;
    hdr.FDFormat            = FDCAN_FD_CAN;
#else
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
#endif
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, (uint8_t *)data) != HAL_OK)
    {
        s_tx_fail++;
        return 0;
    }
    return 1;
}

static void send_input_pos(int j, float pos)
{
    uint8_t  data[8];
    uint32_t bits;

    memcpy(&bits, &pos, sizeof(bits));
    data[0] = (uint8_t)(bits & 0xFFu);
    data[1] = (uint8_t)((bits >> 8) & 0xFFu);
    data[2] = (uint8_t)((bits >> 16) & 0xFFu);
    data[3] = (uint8_t)((bits >> 24) & 0xFFu);
    data[4] = 0;   /* vel_ff    - left at zero for a bring-up test */
    data[5] = 0;
    data[6] = 0;   /* torque_ff */
    data[7] = 0;

    s_joint[j].cmd = pos;
    (void)can_send(s_node_id[j], ODRV_CMD_SET_INPUT_POS, data, 8u);
}

#if LEGTEST_ENABLE_CLOSED_LOOP
static void send_axis_state(int j, uint32_t state)
{
    uint8_t data[4];

    data[0] = (uint8_t)(state & 0xFFu);
    data[1] = (uint8_t)((state >> 8) & 0xFFu);
    data[2] = (uint8_t)((state >> 16) & 0xFFu);
    data[3] = (uint8_t)((state >> 24) & 0xFFu);

    (void)can_send(s_node_id[j], ODRV_CMD_SET_AXIS_STATE, data, 4u);
}
#endif

/* ------------------------------------------------------------------ */
/*  Receive - called from the FDCAN1 interrupt                         */
/* ------------------------------------------------------------------ */

void legtest_on_rx(void)
{
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[64];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK)
        {
            return;
        }

        s_rx_total++;

        uint32_t node = (hdr.Identifier >> 5) & 0x3Fu;
        uint32_t cmd  = hdr.Identifier & 0x1Fu;

        int j = joint_from_node(node);
        if (j < 0)
        {
            s_rx_unknown++;   /* a node id we were not expecting - worth seeing */
            continue;
        }

        s_joint[j].last_rx_tick = s_tick;

        switch (cmd)
        {
        case ODRV_CMD_GET_ENCODER:
            s_joint[j].pos = le_f32(&data[0]);
            s_joint[j].vel = le_f32(&data[4]);
            s_joint[j].n_encoder++;
            break;

        case ODRV_CMD_GET_TORQUES:
            /* bytes 0-3 are the target, 4-7 the estimate */
            s_joint[j].torque = le_f32(&data[4]);
            s_joint[j].n_torque++;
            break;

        case ODRV_CMD_HEARTBEAT:
            s_joint[j].axis_error = le_u32(&data[0]);
            s_joint[j].axis_state = data[4];
            s_joint[j].n_heartbeat++;
            break;

        default:
            break;
        }
    }
}

void legtest_on_tick(void)
{
    s_tick_pending++;
}

/* ------------------------------------------------------------------ */

static void bus_setup(void)
{
    FDCAN_FilterTypeDef f;

    /* Accept the whole CANSimple id space for nodes 1..15 so that a node with
       an unexpected id still shows up as "unknown" rather than vanishing. */
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x020u;   /* node 1,  cmd 0  */
    f.FilterID2    = 0x1FFu;   /* node 15, cmd 31 */

    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void legtest_init(void)
{
    memset((void *)s_joint, 0, sizeof(s_joint));
    s_rx_total     = 0;
    s_rx_unknown   = 0;
    s_tick_pending = 0;
    s_tick         = 0;
    s_tx_fail      = 0;

    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Init(LED_RED);
    BSP_LED_Off(LED_YELLOW);

    bus_setup();

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);

    printf("\r\n=========== CAN-FD SINGLE LEG TEST ===========\r\n");
    printf("bus      : FDCAN1  (PB8 rx / PD1 tx)\r\n");
#if LEGTEST_USE_CAN_FD
    printf("framing  : CAN FD, BRS on (1 Mbit arb / 5 Mbit data)\r\n");
#else
    printf("framing  : classic CAN 2.0 @ 1 Mbit\r\n");
#endif
    printf("nodes    : ");
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        printf("%u=%s ", (unsigned)s_node_id[j], s_joint_name[j]);
    }
    printf("\r\n");
#if LEGTEST_ENABLE_CLOSED_LOOP
    printf("closed loop: ENABLED - MOTORS WILL MOVE. amplitude %.3f turns @ %.2f Hz\r\n",
           (double)LEGTEST_AMPLITUDE_TURNS, (double)LEGTEST_FREQ_HZ);
#else
    printf("closed loop: disabled (safe) - axes stay IDLE, motors cannot move.\r\n");
    printf("             positions are still transmitted so TX can be verified.\r\n");
#endif
    printf("\r\nFor telemetry, ODrive must be publishing cyclically:\r\n");
    printf("  axis0.config.can.encoder_msg_rate_ms = 10\r\n");
    printf("  axis0.config.can.torque_msg_rate_ms  = 10\r\n");
    printf("  axis0.config.can.heartbeat_msg_rate_ms = 100\r\n");
    printf("==============================================\r\n\r\n");
}

/* ------------------------------------------------------------------ */

static void report(void)
{
    uint32_t now = s_tick;
    int      silent = 0;

    printf("--- t=%lus  rx=%lu (unknown=%lu)  txfail=%lu ---\r\n",
           (unsigned long)(now / 1000u), (unsigned long)s_rx_total,
           (unsigned long)s_rx_unknown, (unsigned long)s_tx_fail);

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        joint_t v;

        /* The ISR writes these, so take a consistent snapshot. */
        uint32_t pm = critical_enter();
        v = *(joint_t *)&s_joint[j];
        critical_exit(pm);

        uint8_t alive = ((now - v.last_rx_tick) < NODE_SILENT_TICKS) &&
                        ((v.n_heartbeat + v.n_encoder + v.n_torque) > 0u);

        if (!alive)
        {
            silent++;
            printf("  node %u %s : ---- SILENT ----  (hb=%lu enc=%lu trq=%lu)\r\n",
                   (unsigned)s_node_id[j], s_joint_name[j],
                   (unsigned long)v.n_heartbeat, (unsigned long)v.n_encoder,
                   (unsigned long)v.n_torque);
        }
        else
        {
            printf("  node %u %s : pos=%+8.4f vel=%+8.3f trq=%+7.3f "
                   "state=%u err=0x%08lX  hb=%lu enc=%lu trq=%lu  cmd=%+.4f\r\n",
                   (unsigned)s_node_id[j], s_joint_name[j],
                   (double)v.pos, (double)v.vel, (double)v.torque,
                   (unsigned)v.axis_state, (unsigned long)v.axis_error,
                   (unsigned long)v.n_heartbeat, (unsigned long)v.n_encoder,
                   (unsigned long)v.n_torque, (double)v.cmd);
        }
    }

    /* Red LED blinks once per silent node; dark when all four answer. */
    if (silent == 0)
    {
        BSP_LED_Off(LED_RED);
    }

    printf("\r\n");
}

void legtest_run(void)
{
    uint32_t report_tick = 0;
    uint32_t beat        = 0;
    int      tx_slot     = 0;

    for (;;)
    {
        if (s_tick_pending == 0u)
        {
            continue;
        }

        uint32_t pm = critical_enter();
        s_tick_pending = 0;
        critical_exit(pm);

        s_tick++;

        /*
         * One node per tick, round-robin. Two reasons: the hardware TX FIFO
         * only holds three frames, and spreading the four commands across four
         * ticks keeps the bus evenly loaded instead of bursting. Each joint
         * still gets commanded at 250 Hz, far more than enough to watch a leg
         * move.
         */
        float phase = 2.0f * 3.14159265f * LEGTEST_FREQ_HZ *
                      ((float)s_tick * 0.001f);
        float target = LEGTEST_AMPLITUDE_TURNS * sinf(phase);

        send_input_pos(tx_slot, target);
        tx_slot = (tx_slot + 1) % JOINT_COUNT;

#if LEGTEST_ENABLE_CLOSED_LOOP
        /* Re-assert closed loop every 2 s: an ODrive that trips into IDLE on a
           fault would otherwise sit there silently ignoring position commands. */
        if ((s_tick % 2000u) == 500u)
        {
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                if (s_joint[j].axis_state != ODRV_AXIS_STATE_CLOSED_LOOP)
                {
                    send_axis_state(j, ODRV_AXIS_STATE_CLOSED_LOOP);
                }
            }
        }
#endif

        /* Green heartbeat at 1 Hz - proves the loop itself is alive. */
        if (++beat >= 500u)
        {
            beat = 0;
            BSP_LED_Toggle(LED_GREEN);
        }

        if (++report_tick >= 1000u)
        {
            report_tick = 0;
            report();

            /* The report blocks for several ms on the UART; drop the tick
               backlog it created rather than letting it skew the next cycle. */
            pm = critical_enter();
            s_tick_pending = 0;
            critical_exit(pm);
        }
    }
}
