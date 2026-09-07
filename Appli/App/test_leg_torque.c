#include "test_leg_torque.h"
#include "gait_ref.h"
#include "critical.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef   htim2;
extern TIM_HandleTypeDef   htim6;

/* =====================================================================
 *  TORQUE ARCHITECTURE
 *
 *  The ODrive is a cascaded position -> velocity -> current controller.
 *  In POSITION_CONTROL the whole cascade runs and the drive owns the
 *  stiffness. In TORQUE_CONTROL only the current loop runs: the drive
 *  becomes a torque source and the outer loop is ours.
 *
 *  This file closes position and velocity on the STM32:
 *
 *      tau_out = Kp * (theta_ref - theta) + Kd * (omega_ref - omega) + tau_ff
 *      tau_motor = tau_out / gear
 *
 *  and sends tau_motor with Set_Input_Torque (0x00E) at 1 kHz.
 *
 *  What changes versus test_leg_can.c, and why it matters:
 *
 *  - The drive no longer holds position on its own. If our commands stop,
 *    the joint does NOT hold - it goes limp under whatever torque was last
 *    latched. Every failure path here has to command zero and disarm.
 *  - There is no gravity compensation unless tau_ff provides it. A PD law
 *    alone will sag under load with a steady-state error of tau_gravity/Kp.
 *  - ODrive's own pos_gain / vel_gain are IGNORED in torque mode. The tuning
 *    recorded in docs/ODRIVE_COMMANDS.md does not carry over. Start soft.
 * ===================================================================== */

/* ===================================================================== */
/*  CONFIGURATION                                                        */
/* ===================================================================== */

#define JOINT_COUNT      3

static const uint8_t s_node_id[JOINT_COUNT] = { 1, 3, 4 };

static const char *const s_joint_name[JOINT_COUNT] =
    { "hip_pitch", "knee", "ankle" };

static const uint8_t s_gait_col[JOINT_COUNT] =
    { GAIT_COL_HIP_PITCH, GAIT_COL_KNEE, GAIT_COL_ANKLE };

/*
 * Two different ratios. They are NOT interchangeable and the tables stay
 * separate even when one of them is all ones.
 *
 * s_enc_per_out  - drive turns per turn of the OUTPUT shaft. Follows where the
 *                  encoder physically sits. Hip and knee read the LOAD side, so
 *                  pos_estimate and vel_estimate already arrive in output-shaft
 *                  units: 1.0. Hip and ankle read the MOTOR side of their
 *                  47:1 and 9:1, so their readings are that many times the
 *                  output and are divided down before the control law.
 *
 * s_gear         - the GEARBOX reduction. A property of the mechanism, not of
 *                  the encoder, so it does NOT become 1.0 just because the
 *                  encoder moved. ODrive commands and reports torque at the
 *                  MOTOR shaft only - it has no concept of load-side torque -
 *                  so every torque we send is divided by this, and every
 *                  torque it reports is multiplied by it to read as joint Nm.
 *
 * Using the encoder table for torque would command 47x too much on the hip and
 * knee. Using the gear table for position would command 47x too far. They look
 * similar and mean opposite things, which is why they are two tables.
 */
static const float s_enc_per_out[JOINT_COUNT] = { 47.0f,  1.0f, 9.0f };
static const float s_gear[JOINT_COUNT]        = { 47.0f, 47.0f, 9.0f };

/*
 * Gains, at the OUTPUT shaft.
 *   Kp [Nm per rad of joint error]
 *   Kd [Nm per rad/s of joint velocity error]
 *
 * These are deliberately soft. They are NOT derived from the ODrive gains that
 * were tuned for position mode - that tuning does not transfer, because those
 * gains fed a cascade whose inner loops are now switched off.
 *
 * To tune: raise Kp until the joint tracks without buzzing, then raise Kd until
 * overshoot stops. If it oscillates, Kd is too low or Kp too high. If it sags,
 * that is gravity and needs tau_ff, not more Kp.
 */
static const float s_kp[JOINT_COUNT] = { 50.0f, 50.0f, 20.0f };
static const float s_kd[JOINT_COUNT] = {  1.5f,  1.5f,  0.6f };

/*
 * Hard clamp on what we will ask any motor to produce, in Nm AT THE MOTOR.
 * The position-mode runs peaked at ~1.1 Nm, so this starts at roughly half.
 * This is the last line of defence against a gain or sign error - keep it low
 * until the leg has moved under torque control and you believe the numbers.
 */
static const float s_tau_max[JOINT_COUNT] = { 0.5f, 0.5f, 0.3f };

/* Joint travel, degrees either side of zero. Exceeding it disarms. */
static const float s_limit_deg[JOINT_COUNT] = { 25.0f, 35.0f, 35.0f };

/* Runaway detection, output shaft. Exceeding it disarms. */
#define LEGTQ_VEL_MAX_TURNS_S        3.0f

#define LEGTQ_SCAN_MS                2000u
#define LEGTQ_ARM_DELAY_MS           3000u
#define LEGTQ_ENABLE_TORQUE          1      /* 0 = compute and print only  */
#define LEGTQ_HOLD_MS                3000u  /* hold arming pose, then gait */
#define LEGTQ_GAIT                   1      /* 0 = hold pose forever       */
#define LEGTQ_GAIT_SPEED             0.25f
#define LEGTQ_GAIT_CYCLES            5u
#define LEGTQ_RAMP_MS                1000u  /* fade torque in over this    */
#define LEGTQ_STOP_BUTTON            1
#define LEGTQ_USE_CAN_FD             1
#define LEGTQ_TDC_OFFSET             33u
#define NODE_SILENT_TICKS            100u   /* telemetry watchdog, ms      */

#define TWO_PI                       6.28318530718f

/* ===================================================================== */

#define ODRV_CMD_HEARTBEAT       0x001u
#define ODRV_CMD_SET_AXIS_STATE  0x007u
#define ODRV_CMD_GET_ENCODER     0x009u
#define ODRV_CMD_SET_CTRL_MODE   0x00Bu
#define ODRV_CMD_SET_INPUT_TORQUE 0x00Eu
#define ODRV_CMD_GET_TORQUES     0x01Cu

#define ODRV_CTRL_MODE_TORQUE    1u
#define ODRV_INPUT_MODE_PASSTHR  1u
#define ODRV_AXIS_STATE_IDLE         1u
#define ODRV_AXIS_STATE_CLOSED_LOOP  8u

typedef struct
{
    float    pos;            /* drive turns, as the encoder reports    */
    float    vel;            /* drive turns/s                          */
    float    torque;         /* Nm at the motor, as the drive reports  */
    uint32_t axis_error;
    uint8_t  axis_state;
    uint32_t n_heartbeat;
    uint32_t n_encoder;
    uint32_t last_rx_tick;
    float    tau_cmd;        /* what we last sent, Nm at the motor     */
    float    err_rad;        /* last position error, output radians    */
} joint_t;

static volatile joint_t s_joint[JOINT_COUNT];
static uint8_t  s_joint_live[JOINT_COUNT];
static float    s_hold_out[JOINT_COUNT];   /* pose at arming, output turns */
static float    s_offset_out[JOINT_COUNT]; /* gait re-centring, output turns */

static volatile uint16_t s_node_seen[64];
static volatile uint8_t  s_node_state[64];
static volatile uint32_t s_node_err[64];

static volatile uint32_t s_tick_pending;
static uint32_t s_tick;
static volatile uint32_t s_rx_total;
static uint32_t s_tx_ok, s_tx_fail, s_txq_drop;
static uint8_t  s_scan_ok;
static uint8_t  s_stopped;
static uint8_t  s_armed;
static uint8_t  s_gait_done;
static float    s_gait_phase;
static const char *s_stop_reason = "";

static int joint_from_node(uint32_t node)
{
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (s_node_id[j] == node) { return j; }
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

static void put_f32(uint8_t *d, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    d[0] = (uint8_t)(u & 0xFFu);
    d[1] = (uint8_t)((u >> 8) & 0xFFu);
    d[2] = (uint8_t)((u >> 16) & 0xFFu);
    d[3] = (uint8_t)((u >> 24) & 0xFFu);
}

/* ===================================================================== */
/*  CAN transport                                                        */
/* ===================================================================== */

#define TXQ_LEN   32u
#define TXQ_MASK  (TXQ_LEN - 1u)

typedef struct
{
    uint32_t identifier;
    uint8_t  data[8];
    uint8_t  len;
} txq_entry_t;

static txq_entry_t s_txq[TXQ_LEN];
static uint8_t     s_txq_head, s_txq_tail;

static void tx_enqueue(uint32_t node, uint32_t cmd, const uint8_t *data,
                       uint32_t len)
{
    uint8_t next = (uint8_t)((s_txq_head + 1u) & TXQ_MASK);

    if (next == s_txq_tail)
    {
        s_txq_tail = (uint8_t)((s_txq_tail + 1u) & TXQ_MASK);
        s_txq_drop++;
    }
    s_txq[s_txq_head].identifier = (node << 5) | cmd;
    s_txq[s_txq_head].len        = (uint8_t)len;
    memcpy(s_txq[s_txq_head].data, data, len);
    s_txq_head = next;
}

static uint8_t can_send(uint32_t node, uint32_t cmd, const uint8_t *data,
                        uint32_t len)
{
    FDCAN_TxHeaderTypeDef hdr;

    hdr.Identifier          = (node << 5) | cmd;
    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = (len == 0u) ? FDCAN_DLC_BYTES_0 :
                              (len == 8u) ? FDCAN_DLC_BYTES_8 :
                                            FDCAN_DLC_BYTES_4;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
#if LEGTQ_USE_CAN_FD
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
    s_tx_ok++;
    return 1;
}

static void can_recover_poll(void)
{
    FDCAN_ProtocolStatusTypeDef ps;

    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);

    if (ps.ErrorPassive && (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u))
    {
        s_txq_tail = s_txq_head;          /* stop spinning on a wedged FIFO */
    }
    if (ps.BusOff == 0u) { return; }

    if (READ_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT) == 0u) { return; }

    HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 |
                                       FDCAN_TX_BUFFER2);
    CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
}

static void tx_pump(void)
{
    can_recover_poll();

    while (s_txq_tail != s_txq_head)
    {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u) { break; }

        const txq_entry_t *e = &s_txq[s_txq_tail];

        if (!can_send(e->identifier >> 5, e->identifier & 0x1Fu,
                      e->data, e->len))
        {
            break;
        }
        s_txq_tail = (uint8_t)((s_txq_tail + 1u) & TXQ_MASK);
    }
}

/* ===================================================================== */
/*  Commands                                                             */
/* ===================================================================== */

static void send_torque(int j, float tau_motor)
{
    uint8_t data[4];

    put_f32(data, tau_motor);
    s_joint[j].tau_cmd = tau_motor;
    tx_enqueue(s_node_id[j], ODRV_CMD_SET_INPUT_TORQUE, data, 4u);
}

static void send_ctrl_mode(int j)
{
    uint8_t data[8];

    put_f32(&data[0], 0.0f);
    put_f32(&data[4], 0.0f);
    data[0] = (uint8_t)ODRV_CTRL_MODE_TORQUE;
    data[4] = (uint8_t)ODRV_INPUT_MODE_PASSTHR;
    tx_enqueue(s_node_id[j], ODRV_CMD_SET_CTRL_MODE, data, 8u);
}

static void send_axis_state(int j, uint32_t state)
{
    uint8_t data[4];

    data[0] = (uint8_t)(state & 0xFFu);
    data[1] = 0; data[2] = 0; data[3] = 0;
    tx_enqueue(s_node_id[j], ODRV_CMD_SET_AXIS_STATE, data, 4u);
}

/*
 * Zero torque FIRST, then idle. In torque mode the last value latches, so
 * disarming without zeroing can leave a joint pushing for as long as it takes
 * the state change to arrive.
 */
static void stop_all(const char *why)
{
    if (s_stopped) { return; }

    s_stopped    = 1;
    s_stop_reason = why;
    s_txq_tail   = s_txq_head;

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (s_joint_live[j]) { send_torque(j, 0.0f); }
    }
    for (uint32_t spin = 0u; (s_txq_tail != s_txq_head) && (spin < 100000u);
         spin++)
    {
        tx_pump();
    }
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (s_joint_live[j]) { send_axis_state(j, ODRV_AXIS_STATE_IDLE); }
    }
    for (uint32_t spin = 0u; (s_txq_tail != s_txq_head) && (spin < 100000u);
         spin++)
    {
        tx_pump();
    }
    printf("\r\n*** TORQUE STOPPED: %s - axes zeroed and idled ***\r\n\r\n",
           why);
}

/* ===================================================================== */
/*  Interrupts                                                           */
/* ===================================================================== */

void legtorque_on_rx(void)
{
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[64];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &hdr, data)
            != HAL_OK)
        {
            return;
        }
        s_rx_total++;

        uint32_t node = (hdr.Identifier >> 5) & 0x3Fu;
        uint32_t cmd  = hdr.Identifier & 0x1Fu;

        if (node < 64u)
        {
            if (s_node_seen[node] < 0xFFFFu) { s_node_seen[node]++; }
            if (cmd == ODRV_CMD_HEARTBEAT)
            {
                s_node_state[node] = data[4];
                s_node_err[node]   = le_u32(&data[0]);
            }
        }

        int j = joint_from_node(node);
        if (j < 0) { continue; }

        s_joint[j].last_rx_tick = s_tick;

        switch (cmd)
        {
        case ODRV_CMD_GET_ENCODER:
            s_joint[j].pos = le_f32(&data[0]);
            s_joint[j].vel = le_f32(&data[4]);
            s_joint[j].n_encoder++;
            break;

        case ODRV_CMD_GET_TORQUES:
            s_joint[j].torque = le_f32(&data[4]);
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

void legtorque_on_tick(void) { s_tick_pending++; }

/* ===================================================================== */
/*  Bring-up                                                             */
/* ===================================================================== */

static void bus_setup(void)
{
    FDCAN_FilterTypeDef f;

    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x000u;
    f.FilterID2    = 0x7FFu;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK)
    {
        printf("!! HAL_FDCAN_ConfigFilter FAILED\r\n");
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        printf("!! HAL_FDCAN_ConfigGlobalFilter FAILED\r\n");
    }
#if LEGTQ_USE_CAN_FD
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, LEGTQ_TDC_OFFSET, 0u)
        != HAL_OK)
    {
        printf("!! HAL_FDCAN_ConfigTxDelayCompensation FAILED\r\n");
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
    {
        printf("!! HAL_FDCAN_EnableTxDelayCompensation FAILED\r\n");
    }
#endif
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        printf("!! HAL_FDCAN_Start FAILED\r\n");
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
        != HAL_OK)
    {
        printf("!! HAL_FDCAN_ActivateNotification FAILED\r\n");
    }
}

static void bus_scan(void)
{
    printf("\r\nscanning the bus for %u ms - not transmitting...\r\n",
           (unsigned)LEGTQ_SCAN_MS);

    for (uint32_t i = 0; i < LEGTQ_SCAN_MS; i += 10u)
    {
        HAL_Delay(10);
        legtorque_on_rx();
    }

    for (int n = 0; n < 64; n++)
    {
        if (s_node_seen[n] == 0u) { continue; }

        printf("  node %-2d  %5u frames  axis_state %u  axis_error 0x%08lX%s\r\n",
               n, (unsigned)s_node_seen[n], (unsigned)s_node_state[n],
               (unsigned long)s_node_err[n],
               (joint_from_node((uint32_t)n) >= 0) ? "   <-- configured" : "");
    }

    int live = 0;
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        s_joint_live[j] = (s_node_seen[s_node_id[j]] != 0u) ? 1u : 0u;
        if (s_joint_live[j]) { live++; }
        else
        {
            printf("  !! node %u (%s) never answered - SKIPPED\r\n",
                   (unsigned)s_node_id[j], s_joint_name[j]);
        }
    }
    s_scan_ok = (live > 0) ? 1u : 0u;
    if (!s_scan_ok) { printf("  !! nothing on the bus - torque DISABLED\r\n"); }
    printf("\r\n");
}

void legtorque_init(void)
{
    memset((void *)s_joint, 0, sizeof(s_joint));

    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Init(LED_RED);
    BSP_LED_Off(LED_YELLOW);

    bus_setup();

#if LEGTQ_STOP_BUTTON
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
#endif

    bus_scan();

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);

    printf("\r\n====== CAN-FD SINGLE LEG - STM32-SIDE TORQUE CONTROL ======\r\n");
    printf("the drives run TORQUE_CONTROL; the position and velocity loops\r\n"
           "are closed here at 1 kHz. ODrive pos_gain/vel_gain are unused.\r\n\r\n");

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        printf("  %-9s Kp %6.1f Nm/rad  Kd %5.2f Nm.s/rad  "
               "gear %4.0f:1  clamp %.2f Nm motor (%.1f Nm out)\r\n",
               s_joint_name[j], (double)s_kp[j], (double)s_kd[j],
               (double)s_gear[j], (double)s_tau_max[j],
               (double)(s_tau_max[j] * s_gear[j]));
    }
    printf("\r\n");
}

/* ===================================================================== */
/*  Control                                                              */
/* ===================================================================== */

static void report(void)
{
    static uint32_t prev_tx, prev_rx;
    uint32_t d_tx = s_tx_ok - prev_tx;
    uint32_t d_rx = s_rx_total - prev_rx;
    prev_tx = s_tx_ok; prev_rx = s_rx_total;

    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef  ec;
    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);

    printf("--- t=%lus  tx=%lu rx=%lu  txfail=%lu qdrop=%lu  "
           "TEC=%lu REC=%lu%s%s ---\r\n",
           (unsigned long)(s_tick / 1000u), (unsigned long)d_tx,
           (unsigned long)d_rx, (unsigned long)s_tx_fail,
           (unsigned long)s_txq_drop,
           (unsigned long)ec.TxErrorCnt, (unsigned long)ec.RxErrorCnt,
           ps.BusOff ? "  [BUS-OFF]" : "",
           ps.ErrorPassive ? "  [ERROR-PASSIVE]" : "");

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (!s_joint_live[j])
        {
            printf("  node %u %-9s : ---- not on the bus ----\r\n",
                   (unsigned)s_node_id[j], s_joint_name[j]);
            continue;
        }

        joint_t v;
        uint32_t pm = critical_enter();
        v = *(joint_t *)&s_joint[j];
        critical_exit(pm);

        /* tau values are MOTOR-side, which is the only side ODrive speaks.
           The bracketed figures are the same torque at the joint. */
        printf("  node %u %-9s : out=%+8.4f tn (%+7.2f deg)  vel=%+7.3f  "
               "err=%+6.2f deg\r\n"
               "                       tau_cmd=%+6.3f Nm motor (%+7.2f Nm joint)"
               "   tau_meas=%+6.3f motor (%+7.2f joint)\r\n"
               "                       state=%u err=0x%08lX\r\n",
               (unsigned)s_node_id[j], s_joint_name[j],
               (double)(v.pos / s_enc_per_out[j]),
               (double)(v.pos / s_enc_per_out[j] * 360.0f),
               (double)(v.vel / s_enc_per_out[j]),
               (double)(v.err_rad * 57.2957795f),
               (double)v.tau_cmd, (double)(v.tau_cmd * s_gear[j]),
               (double)v.torque, (double)(v.torque * s_gear[j]),
               (unsigned)v.axis_state, (unsigned long)v.axis_error);
    }
    printf("\r\n");
}

void legtorque_run(void)
{
    uint32_t beat = 0;
    float    ref_out[JOINT_COUNT]  = { 0.0f };
    float    ref_vel[JOINT_COUNT]  = { 0.0f };

    for (;;)
    {
        tx_pump();

        if (s_tick_pending == 0u) { continue; }

        uint32_t pm = critical_enter();
        s_tick_pending = 0;
        critical_exit(pm);

        s_tick++;

#if LEGTQ_STOP_BUTTON
        if (!s_stopped && (BSP_PB_GetState(BUTTON_USER) == 0))
        {
            stop_all("user button");
        }
#endif

        /* ---- arm once, into TORQUE_CONTROL ---- */
        if (!s_armed && !s_stopped && s_scan_ok &&
            (s_tick == LEGTQ_ARM_DELAY_MS))
        {
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                if (!s_joint_live[j]) { continue; }

                s_hold_out[j]   = s_joint[j].pos / s_enc_per_out[j];
                s_offset_out[j] = 0.0f;

                /*
                 * Output travel is well under half a turn on every joint, so a
                 * load-side reading of several turns means the drive is still
                 * reporting the motor side and s_enc_per_out is wrong. Say so
                 * rather than computing torque from a 47x position error.
                 */
                if (fabsf(s_hold_out[j]) > 0.5f)
                {
                    printf("!! %s reads %+.3f output turns at arming."
                           " No joint here has that much travel.\r\n"
                           "   s_enc_per_out[%d] is probably wrong for where"
                           " that encoder sits.\r\n",
                           s_joint_name[j], (double)s_hold_out[j], j);
                    stop_all("implausible encoder reading at arming");
                }
            }

#if LEGTQ_GAIT
            {
                float g0[GAIT_JOINTS];
                gait_sample(0.0f, g0);
                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    /* play the gait about wherever the leg is standing */
                    s_offset_out[j] = s_hold_out[j] - g0[s_gait_col[j]];
                }
            }
#endif
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                if (!s_joint_live[j]) { continue; }
                send_ctrl_mode(j);
                send_axis_state(j, ODRV_AXIS_STATE_CLOSED_LOOP);
                printf("  %-9s holding %+.4f output turns\r\n",
                       s_joint_name[j], (double)s_hold_out[j]);
            }
            s_armed = 1;
            BSP_LED_On(LED_YELLOW);
            printf("*** ARMED in TORQUE_CONTROL ***\r\n\r\n");
        }

        if ((s_tick < LEGTQ_ARM_DELAY_MS) && ((s_tick % 1000u) == 0u))
        {
            printf("*** ARMING in %lu s - the drives become TORQUE SOURCES "
                   "***\r\n",
                   (unsigned long)((LEGTQ_ARM_DELAY_MS - s_tick) / 1000u));
        }

        /* ---- reference ---- */
        if (s_armed && !s_stopped)
        {
            uint32_t since = s_tick - LEGTQ_ARM_DELAY_MS;

            for (int j = 0; j < JOINT_COUNT; j++)
            {
                ref_out[j] = s_hold_out[j];
                ref_vel[j] = 0.0f;
            }

#if LEGTQ_GAIT
            if ((since > LEGTQ_HOLD_MS) && !s_gait_done)
            {
                float t   = (float)(since - LEGTQ_HOLD_MS) * 0.001f;
                float all[GAIT_JOINTS], vel[GAIT_JOINTS];

                s_gait_phase = (t * LEGTQ_GAIT_SPEED) / GAIT_CYCLE_S;

                if (s_gait_phase >= (float)LEGTQ_GAIT_CYCLES)
                {
                    s_gait_phase = (float)LEGTQ_GAIT_CYCLES;
                    s_gait_done  = 1;
                    printf("\r\ngait complete - holding the last pose\r\n\r\n");
                }

                gait_sample_vel(fmodf(s_gait_phase, 1.0f), all, vel);

                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    ref_out[j] = all[s_gait_col[j]] + s_offset_out[j];
                    ref_vel[j] = vel[s_gait_col[j]] * LEGTQ_GAIT_SPEED;
                }
                /* freeze the reference where the gait ended */
                if (s_gait_done)
                {
                    for (int j = 0; j < JOINT_COUNT; j++)
                    {
                        s_hold_out[j] = ref_out[j];
                    }
                }
            }
#endif
            /* fade torque in so arming is not a step */
            float ramp = (since < LEGTQ_RAMP_MS)
                       ? ((float)since / (float)LEGTQ_RAMP_MS) : 1.0f;

            for (int j = 0; j < JOINT_COUNT; j++)
            {
                if (!s_joint_live[j]) { continue; }

                /* ---- watchdogs, before any torque is computed ---- */
                if ((s_tick - s_joint[j].last_rx_tick) > NODE_SILENT_TICKS)
                {
                    stop_all("telemetry timeout - a drive went silent");
                    break;
                }
                if (s_joint[j].axis_error != 0u)
                {
                    stop_all("a drive reported an axis error");
                    break;
                }

                float pos_out = s_joint[j].pos / s_enc_per_out[j];
                float vel_out = s_joint[j].vel / s_enc_per_out[j];

                if (fabsf(pos_out) * 360.0f > s_limit_deg[j])
                {
                    stop_all("a joint left its travel limit");
                    break;
                }
                if (fabsf(vel_out) > LEGTQ_VEL_MAX_TURNS_S)
                {
                    stop_all("a joint exceeded its velocity limit");
                    break;
                }

                /* ---- the control law ---- */
                float e_rad  = (ref_out[j] - pos_out) * TWO_PI;
                float ed_rad = (ref_vel[j] - vel_out) * TWO_PI;

                s_joint[j].err_rad = e_rad;

                float tau_out   = (s_kp[j] * e_rad) + (s_kd[j] * ed_rad);
                float tau_motor = (tau_out / s_gear[j]) * ramp;

                if (tau_motor >  s_tau_max[j]) { tau_motor =  s_tau_max[j]; }
                if (tau_motor < -s_tau_max[j]) { tau_motor = -s_tau_max[j]; }

#if LEGTQ_ENABLE_TORQUE
                send_torque(j, tau_motor);
#else
                s_joint[j].tau_cmd = tau_motor;   /* compute, do not send */
#endif
            }
        }

        if (++beat >= 1000u)
        {
            beat = 0;
            report();
        }
    }
}
