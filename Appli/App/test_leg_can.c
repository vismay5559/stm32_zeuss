#include "test_leg_can.h"
#include "gait_ref.h"
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
/*
 * WHICH ACTUATORS ARE ON THE BUS
 *
 * Start with one drive and add entries as you connect more. Each entry needs
 * the CAN node id you set in odrivetool (axis0.config.can.node_id) and which
 * column of the reference gait feeds it.
 *
 * If you do not know the node id, leave this as it is and run once: the bus
 * scan below listens before transmitting and prints every node that answers.
 * ODrive ships with node_id 0.
 */
#define JOINT_COUNT      1

static float         s_gait_phase;

static const uint8_t s_node_id[JOINT_COUNT]  = { 3 };
static const uint8_t s_gait_col[JOINT_COUNT] = { GAIT_COL_KNEE };

static const char *const s_joint_name[JOINT_COUNT] = { "knee" };

/*
 * Listen this long before transmitting anything, in ms.
 *
 * ODrive sends a heartbeat unprompted, roughly every 100 ms, without being
 * asked. So a silent window at startup answers the question that everything
 * else depends on: is the drive powered, on the right bitrate, and wired the
 * right way round? Nothing we transmit can make that answer ambiguous.
 *
 * Heartbeats arriving means the bus works. Silence means it does not, and no
 * amount of sending Set_Input_Pos will change that.
 */
#define LEGTEST_SCAN_MS              2000u

/*
 * Print every frame received during the scan, decoded.
 *
 * This is the closest thing to a CAN analyser you have without buying one, and
 * for bring-up it is usually enough - the question is almost never "what are
 * the exact bytes" but "is anything arriving, from whom, and does it look
 * like ODrive".
 *
 * Frames are captured into a ring buffer by the RX handler and printed from
 * the main loop. printf() from an interrupt at 115200 baud blocks for
 * milliseconds and would drop the very frames it is trying to show.
 */
#define LEGTEST_TRACE_RX             1
#define TRACE_LEN                    96u

/*
 * ======================= MOTORS LIVE WHEN THIS IS 1 =======================
 *
 * Commands each axis into CLOSED_LOOP_CONTROL over CAN (Set_Axis_State), which
 * energises the motors. Set_Input_Pos alone does NOT do this - an IDLE axis
 * accepts positions and stays unpowered, which is why this is separate.
 *
 * Before running with this at 1:
 *   - leg in a fixture, off the ground, nothing in its swing range
 *   - physical e-stop that cuts actuator power within reach
 *   - low current and velocity limits set in odrivetool
 *   - ODrive watchdog enabled (axis0.config.enable_watchdog = True) so the
 *     actuators shut down by themselves if this firmware ever stops
 *   - bring up ONE node at a time before running all four
 *
 * Set back to 0 to transmit positions without ever energising anything, which
 * is still a complete test of the TX path.
 * ==========================================================================
 */
#define LEGTEST_ENABLE_CLOSED_LOOP   0

/*
 * Delay before arming, in ticks (ms). A reset must never energise motors
 * instantly - this gives you time to see the countdown and pull power.
 */
#define LEGTEST_ARM_DELAY_MS         3000u

/*
 * Motion profile. Only has any effect when closed loop is enabled above.
 *
 *   1 = play the reference gait from gait_ref.c (generated from the
 *       spreadsheet by tools/gen_gait.py)
 *   0 = the old single-joint sine, useful for a first spin of one axis
 */
#define LEGTEST_MOTION_GAIT          1

/* Sine fallback, used only when LEGTEST_MOTION_GAIT is 0. */
#define LEGTEST_AMPLITUDE_TURNS      0.05f   /* +/- turns, after gearbox      */
#define LEGTEST_FREQ_HZ              0.25f   /* slow enough to watch          */

/*
 * Playback speed, 1.0 = the 0.8 s cycle the trajectory was optimised for.
 * START BELOW 1. At quarter speed every joint moves a quarter as fast, so a
 * wrong sign or a bad node mapping shows up as a slow drift you can watch and
 * kill rather than a snap you can only hear.
 */
#define LEGTEST_GAIT_SPEED           0.25f

/*
 * Time to travel from wherever the leg is when the axes arm to the first pose
 * of the trajectory, in ticks (ms).
 *
 * Without this the first command after arming would be gait phase 0, and the
 * leg would jump there from its resting position as fast as the drives allow.
 * The ramp starts from the MEASURED position, so it is only as good as the CAN
 * telemetry - if no encoder estimates have arrived the entry is skipped and
 * nothing moves.
 */
#define LEGTEST_GAIT_ENTRY_MS        2000u

/*
 * ODrive S1 may or may not support CAN FD depending on firmware version - it
 * is worth confirming before trusting FD framing. Set this to 0 to fall back
 * to classic CAN 2.0 (still 1 Mbit, since classic uses only the nominal bit
 * timing) if the nodes do not respond in FD.
 */
#define LEGTEST_USE_CAN_FD           1

/*
 * How often each joint gets a Set_Input_Pos, as a divider on the 1 kHz tick.
 *   1 = 1 kHz per joint  (4 TX frames every tick)
 *   4 = 250 Hz per joint (one joint per tick, round-robin)
 *
 * At 1 kHz with encoder+torque telemetry also at 1 kHz, this bus carries
 * 3 frames per node per ms = 12 frames/ms for four nodes. At the current
 * 1 Mbit nominal / 5 Mbit data that is ~50 us per frame, so ~60% bus load -
 * workable for a four-node leg, but see the note in README about raising the
 * NOMINAL bitrate before all five nodes per bus go live.
 */
#define LEGTEST_TX_DIV               1u

/* Consider a node dead if nothing has been heard from it for this long. */
#define NODE_SILENT_TICKS            500u    /* ms */

/*
 * Approximate time one 8-byte frame occupies the bus, used only to report an
 * estimated load figure. Small 8-byte frames are dominated by the arbitration
 * phase, which runs at the NOMINAL rate - which is why raising the nominal
 * bitrate helps far more than raising the data bitrate.
 */
#if LEGTEST_USE_CAN_FD
#define FRAME_US                     50u     /* 1 Mbit arb + 5 Mbit data */
#else
#define FRAME_US                     121u    /* classic 1 Mbit           */
#endif

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
/* Recent frames, captured cheaply in the RX handler and printed elsewhere. */
typedef struct
{
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
} trace_t;

static volatile trace_t  s_trace[TRACE_LEN];
static volatile uint8_t  s_trace_head;
static volatile uint8_t  s_trace_tail;

/* Every node id heard from, whether or not we were expecting it. */
static volatile uint16_t s_node_seen[64];
static volatile uint8_t  s_node_state[64];
static volatile uint32_t s_node_err[64];

static uint8_t s_scan_ok;          /* the configured nodes all answered */

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

/*
 * Software transmit queue.
 *
 * The hardware TX FIFO on this part is fixed at three entries, but a 1 kHz
 * tick wants to hand over four Set_Input_Pos frames at once. Pushing all four
 * straight at the hardware silently loses one - so frames go into this ring
 * and tx_pump() feeds the FIFO as space frees up.
 *
 * Producer (the tick) and consumer (tx_pump) both run in main-loop context,
 * never in an ISR, so no locking is needed.
 */
#define TXQ_LEN   32u                  /* power of two */
#define TXQ_MASK  (TXQ_LEN - 1u)

typedef struct
{
    uint32_t identifier;
    uint8_t  data[8];
    uint8_t  len;
} txq_entry_t;

static txq_entry_t s_txq[TXQ_LEN];
static uint8_t     s_txq_head;
static uint8_t     s_txq_tail;
static uint32_t    s_tx_ok;
static uint32_t    s_txq_drop;

static void tx_enqueue(uint32_t node, uint32_t cmd, const uint8_t *data, uint32_t len)
{
    uint8_t next = (uint8_t)((s_txq_head + 1u) & TXQ_MASK);

    if (next == s_txq_tail)
    {
        /*
         * Queue full - the bus is not draining. Discard the OLDEST entry, not
         * this one: these are position setpoints, and a stale setpoint is
         * worthless while the newest is exactly what the actuator should get.
         * Dropping the newest instead would leave the queue full of commands
         * from seconds ago and effectively freeze the leg once the bus
         * recovered.
         */
        s_txq_tail = (uint8_t)((s_txq_tail + 1u) & TXQ_MASK);
        s_txq_drop++;
    }

    s_txq[s_txq_head].identifier = (node << 5) | cmd;
    s_txq[s_txq_head].len        = (uint8_t)len;
    memcpy(s_txq[s_txq_head].data, data, len);
    s_txq_head = next;
}

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
    s_tx_ok++;
    return 1;
}

/* Move queued frames into the hardware FIFO. Call often from the main loop. */
static void tx_pump(void)
{
    while (s_txq_tail != s_txq_head)
    {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u)
        {
            break;                      /* hardware full, retry next pass */
        }

        const txq_entry_t *e = &s_txq[s_txq_tail];

        if (!can_send(e->identifier >> 5, e->identifier & 0x1Fu, e->data, e->len))
        {
            break;                      /* leave it queued rather than lose it */
        }

        s_txq_tail = (uint8_t)((s_txq_tail + 1u) & TXQ_MASK);
    }
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
    tx_enqueue(s_node_id[j], ODRV_CMD_SET_INPUT_POS, data, 8u);
}

static void send_axis_state(int j, uint32_t state)
{
    uint8_t data[4];

    data[0] = (uint8_t)(state & 0xFFu);
    data[1] = (uint8_t)((state >> 8) & 0xFFu);
    data[2] = (uint8_t)((state >> 16) & 0xFFu);
    data[3] = (uint8_t)((state >> 24) & 0xFFu);

    tx_enqueue(s_node_id[j], ODRV_CMD_SET_AXIS_STATE, data, 4u);
}

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

#if LEGTEST_TRACE_RX
        {
            uint8_t next = (uint8_t)((s_trace_head + 1u) % TRACE_LEN);

            /* Drop the newest when full rather than overwrite unread history:
               the first frames after power-up are the interesting ones. */
            if (next != s_trace_tail)
            {
                s_trace[s_trace_head].id  = hdr.Identifier;
                s_trace[s_trace_head].len = 8u;
                for (int b = 0; b < 8; b++)
                {
                    s_trace[s_trace_head].data[b] = data[b];
                }
                s_trace_head = next;
            }
        }
#endif

        /* Log every node before filtering, so the scan can report drives we
           were not configured for - the usual case on a first bring-up. */
        if (node < 64u)
        {
            if (s_node_seen[node] < 0xFFFFu)
            {
                s_node_seen[node]++;
            }
            if (cmd == ODRV_CMD_HEARTBEAT)
            {
                s_node_state[node] = data[4];
                s_node_err[node]   = le_u32(&data[0]);
            }
        }

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

    /*
     * Accept the ENTIRE standard id space. CANSimple packs id = node<<5 | cmd,
     * so 0x000..0x7FF covers every node from 0 to 63.
     *
     * The obvious filter starts at node 1 and would never see node 0 - which is
     * exactly what an ODrive ships with. A drive nobody had configured yet would
     * simply be invisible, and look identical to a dead bus.
     */
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x000u;   /* node 0,  cmd 0  */
    f.FilterID2    = 0x7FFu;   /* node 63, cmd 31 */

    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

#if LEGTEST_TRACE_RX
/* ODrive CANSimple command names, for the ones we actually expect. */
static const char *cmd_name(uint32_t cmd)
{
    switch (cmd)
    {
    case 0x001u: return "Heartbeat";
    case 0x002u: return "Estop";
    case 0x003u: return "GetError";
    case 0x007u: return "SetAxisState";
    case 0x009u: return "EncoderEstimates";
    case 0x00Cu: return "SetInputPos";
    case 0x014u: return "GetBusVoltage";
    case 0x01Cu: return "GetTorques";
    default:     return "";
    }
}

/* Print whatever the RX handler captured. Main-loop context only. */
static void trace_drain(int max_lines)
{
    while ((s_trace_tail != s_trace_head) && (max_lines-- > 0))
    {
        volatile trace_t *t = &s_trace[s_trace_tail];

        uint32_t node = (t->id >> 5) & 0x3Fu;
        uint32_t cmd  = t->id & 0x1Fu;

        printf("  id 0x%03lX  node %-2lu cmd 0x%02lX %-17s",
               (unsigned long)t->id, (unsigned long)node,
               (unsigned long)cmd, cmd_name(cmd));

        for (int b = 0; b < 8; b++)
        {
            printf("%02X ", (unsigned)t->data[b]);
        }

        /* Heartbeat is the one worth decoding inline - it is what tells you
           the drive is alive and whether it is sitting in a fault. */
        if (cmd == 0x001u)
        {
            uint32_t err = le_u32((const uint8_t *)t->data);
            printf(" err=0x%08lX state=%u", (unsigned long)err,
                   (unsigned)t->data[4]);
        }

        printf("\r\n");
        s_trace_tail = (uint8_t)((s_trace_tail + 1u) % TRACE_LEN);
    }
}
#endif

/*
 * Listen for ODrive heartbeats before transmitting anything.
 *
 * Every other failure mode looks the same from the outside: a drive on the
 * wrong bitrate, CANH/CANL swapped, no termination, no power, or simply a
 * different node id than we were told. Sending first makes all of those
 * indistinguishable. Listening first separates "the bus works" from
 * "the bus does not", which is the only question worth answering first.
 */
static void bus_scan(void)
{
    printf("\r\nscanning the bus for %u ms - not transmitting...\r\n",
           (unsigned)LEGTEST_SCAN_MS);

    for (uint32_t i = 0; i < LEGTEST_SCAN_MS; i += 10u)
    {
        HAL_Delay(10);
        legtest_on_rx();          /* drain, in case the ISR is not wired yet */
#if LEGTEST_TRACE_RX
        trace_drain(2);           /* a couple per pass so printf never piles up */
#endif
    }

    int found = 0;

    for (int n = 0; n < 64; n++)
    {
        if (s_node_seen[n] == 0u)
        {
            continue;
        }

        found++;
        printf("  node %-2d  %5u frames  axis_state %u  axis_error 0x%08lX%s\r\n",
               n, (unsigned)s_node_seen[n], (unsigned)s_node_state[n],
               (unsigned long)s_node_err[n],
               (joint_from_node((uint32_t)n) >= 0) ? "   <-- configured" : "");
    }

    if (found == 0)
    {
        printf("  NOTHING ON THE BUS.\r\n");
        printf("  The drive sends heartbeats on its own, so silence here is\r\n");
        printf("  physical, not protocol. In rough order of likelihood:\r\n");
        printf("    - transceiver not powered, or CANH/CANL swapped\r\n");
        printf("    - no 120 ohm termination (you need it at BOTH ends)\r\n");
        printf("    - ODrive on a different CAN bitrate than 1 Mbit nominal\r\n");
        printf("    - ODrive not powered, or its CAN not enabled\r\n");
    }

    /* Every configured node must have answered. Commanding a node that is not
       there is harmless; assuming one IS there and arming is not. */
    s_scan_ok = 1;
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (s_node_seen[s_node_id[j]] == 0u)
        {
            s_scan_ok = 0;
            printf("  !! node %u (%s) never answered - closed loop DISABLED\r\n",
                   (unsigned)s_node_id[j], s_joint_name[j]);
        }
    }

    printf("\r\n");
}

void legtest_init(void)
{
    memset((void *)s_joint, 0, sizeof(s_joint));
    s_rx_total     = 0;
    s_rx_unknown   = 0;
    s_tick_pending = 0;
    s_tick         = 0;
    s_tx_fail      = 0;
    s_tx_ok        = 0;
    s_txq_drop     = 0;
    s_txq_head     = 0;
    s_txq_tail     = 0;

    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Init(LED_RED);
    BSP_LED_Off(LED_YELLOW);

    bus_setup();

    bus_scan();

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
#if LEGTEST_MOTION_GAIT
    printf("motion   : REFERENCE GAIT, %d samples, %.3f s cycle at %.2fx"
           " speed\r\n", GAIT_SAMPLES, (double)GAIT_CYCLE_S,
           (double)LEGTEST_GAIT_SPEED);
    printf("closed loop: ENABLED - MOTORS WILL MOVE. %u ms ramp from the"
           " measured pose\r\n             into the trajectory before the phase clock starts.\r\n",
           (unsigned)LEGTEST_GAIT_ENTRY_MS);
#else
    printf("closed loop: ENABLED - MOTORS WILL MOVE. sine %.3f turns @ %.2f Hz\r\n",
           (double)LEGTEST_AMPLITUDE_TURNS, (double)LEGTEST_FREQ_HZ);
#endif
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

    /*
     * Estimated bus load over the last second. Frames on the wire are what
     * matter, so count what we successfully sent plus everything received.
     * This is the number to watch when raising rates: past roughly 50% the
     * latency of lower-priority frames starts to degrade badly, and past 70%
     * it becomes effectively unbounded.
     */
    static uint32_t prev_tx_ok, prev_rx;

    uint32_t d_tx = s_tx_ok - prev_tx_ok;
    uint32_t d_rx = s_rx_total - prev_rx;
    prev_tx_ok = s_tx_ok;
    prev_rx    = s_rx_total;

    uint32_t load_pct = ((d_tx + d_rx) * FRAME_US) / 10000u;   /* per second */

    printf("--- t=%lus  tx=%lu rx=%lu (unknown=%lu)  txfail=%lu qdrop=%lu  "
           "bus~%lu%% ---\r\n",
           (unsigned long)(now / 1000u), (unsigned long)d_tx,
           (unsigned long)d_rx, (unsigned long)s_rx_unknown,
           (unsigned long)s_tx_fail, (unsigned long)s_txq_drop,
           (unsigned long)load_pct);

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
        /* Keep feeding the 3-deep hardware FIFO from the software queue. This
           runs far more often than once per tick, so a tick's worth of frames
           reaches the wire well inside that tick. */
        tx_pump();

        if (s_tick_pending == 0u)
        {
            continue;
        }

        uint32_t pm = critical_enter();
        s_tick_pending = 0;
        critical_exit(pm);

        s_tick++;

#if LEGTEST_TRACE_RX
        /* A few frames per tick after the scan. Enough to watch traffic without
           the console becoming the thing that breaks the timing. */
        if ((s_tick % 200u) == 0u)
        {
            trace_drain(3);
        }
#endif

        /*
         * One node per tick, round-robin. Two reasons: the hardware TX FIFO
         * only holds three frames, and spreading the four commands across four
         * ticks keeps the bus evenly loaded instead of bursting. Each joint
         * still gets commanded at 250 Hz, far more than enough to watch a leg
         * move.
         */
        float target[JOINT_COUNT] = { 0.0f };

#if LEGTEST_ENABLE_CLOSED_LOOP
        /* Stay at zero through the arming delay so the leg does not lurch the
           instant the axes come live. */
        if (s_tick > LEGTEST_ARM_DELAY_MS)
        {
            uint32_t since_arm = s_tick - LEGTEST_ARM_DELAY_MS;

#if LEGTEST_MOTION_GAIT
            static float   s_entry_from[JOINT_COUNT];
            static uint8_t s_entry_ok;

            float all[GAIT_JOINTS], first[JOINT_COUNT];

            gait_sample(0.0f, all);
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                first[j] = all[s_gait_col[j]];
            }

            if (since_arm == 1u)
            {
                /*
                 * Capture where the leg actually is, once, at the moment of
                 * arming. Every joint must have reported an encoder estimate;
                 * ramping from an assumed zero towards the trajectory would
                 * command a jump exactly as large as the assumption is wrong.
                 */
                s_entry_ok = 1;
                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    if (s_joint[j].n_encoder == 0u)
                    {
                        s_entry_ok = 0;
                    }
                    s_entry_from[j] = s_joint[j].pos;
                }

                if (!s_entry_ok)
                {
                    printf("\r\n!! no encoder estimate from every"
                           " joint - gait NOT started, holding position\r\n");
                }
            }

            if (!s_entry_ok)
            {
                /* Hold station at the last measured position. */
                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    target[j] = s_joint[j].pos;
                }
            }
            else if (since_arm < LEGTEST_GAIT_ENTRY_MS)
            {
                /* Straight-line move into the start of the trajectory. */
                float a = (float)since_arm / (float)LEGTEST_GAIT_ENTRY_MS;

                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    target[j] = s_entry_from[j] +
                                (first[j] - s_entry_from[j]) * a;
                }
            }
            else
            {
                /* Free-running phase clock. gait_sample wraps it, so this can
                   count up forever without special-casing the seam. */
                float t = (float)(since_arm - LEGTEST_GAIT_ENTRY_MS) * 0.001f;

                s_gait_phase = (t * LEGTEST_GAIT_SPEED) / GAIT_CYCLE_S;

                gait_sample(s_gait_phase, all);
                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    target[j] = all[s_gait_col[j]];
                }
            }
#else
            float t     = (float)since_arm * 0.001f;
            float phase = 2.0f * 3.14159265f * LEGTEST_FREQ_HZ * t;
            float v     = LEGTEST_AMPLITUDE_TURNS * sinf(phase);

            for (int j = 0; j < JOINT_COUNT; j++)
            {
                target[j] = v;
            }
#endif
        }
#endif

        if ((s_tick % LEGTEST_TX_DIV) == 0u)
        {
#if (LEGTEST_TX_DIV == 1u)
            /* Full rate: command every joint on every tick. */
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                send_input_pos(j, target[j]);
            }
#else
            /* Reduced rate: one joint per tick, round-robin. */
            send_input_pos(tx_slot, target[tx_slot]);
            tx_slot = (tx_slot + 1) % JOINT_COUNT;
#endif
        }

#if LEGTEST_ENABLE_CLOSED_LOOP
        /* Countdown, so a reset never energises motors without warning. */
        if ((s_tick <= LEGTEST_ARM_DELAY_MS) && ((s_tick % 1000u) == 0u))
        {
            printf("*** ARMING in %lu s - motors will become live ***\r\n",
                   (unsigned long)((LEGTEST_ARM_DELAY_MS - s_tick) / 1000u));
        }

        /*
         * Re-assert closed loop every 2 s. An ODrive that trips into IDLE on a
         * fault would otherwise sit there silently ignoring position commands,
         * and the leg would look "dead" for no visible reason.
         *
         * Only re-arm an axis reporting no error: repeatedly forcing a faulted
         * axis back into closed loop fights whatever protection tripped it,
         * which is exactly the wrong response to a real fault.
         */
        /* s_scan_ok gates this. If a configured node never answered during the
           scan, the bus is not what we think it is, and arming a drive we
           cannot hear back from is the one thing not worth risking. */
        if (s_scan_ok && (s_tick > LEGTEST_ARM_DELAY_MS) &&
            ((s_tick % 2000u) == 500u))
        {
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                if ((s_joint[j].axis_state != ODRV_AXIS_STATE_CLOSED_LOOP) &&
                    (s_joint[j].axis_error == 0u) &&
                    (s_joint[j].n_heartbeat > 0u))
                {
                    printf("arming node %u (%s)\r\n",
                           (unsigned)s_node_id[j], s_joint_name[j]);
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
