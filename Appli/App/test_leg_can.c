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

#define JOINT_COUNT      3

static float         s_gait_phase;
static uint8_t       s_gait_done;
static uint8_t       s_gait_running;
static uint8_t       s_stopped;      

static const uint8_t s_node_id[JOINT_COUNT]  = { 1, 3, 4 };
static const uint8_t s_gait_col[JOINT_COUNT] =
    { GAIT_COL_HIP_PITCH, GAIT_COL_KNEE, GAIT_COL_ANKLE };

static const float s_zero_offset[JOINT_COUNT] = { 0.0f, 0.0f, 0.0f };

static const char *const s_joint_name[JOINT_COUNT] =
    { "hip_pitch", "knee", "ankle" };

/*
 * Which joints actually answered the boot scan. A joint that never spoke is
 * not armed, not commanded and not captured - but it does NOT stop the rest of
 * the leg from running. Filled in by bus_scan() before the timers start, so it
 * is only ever read after that.
 */
static uint8_t s_joint_live[JOINT_COUNT];

/*
 * Nodes we listen to but never command. They are not armed, not gaited, not
 * captured and they cannot block arming - they just appear in the periodic
 * report so their telemetry is visible. Set MONITOR_COUNT to 0 to drop them.
 */
#define MONITOR_COUNT    0
#if (MONITOR_COUNT > 0)
static const uint8_t s_mon_node[MONITOR_COUNT] = { 0 };
static const char *const s_mon_name[MONITOR_COUNT] = { "" };
#endif

/*
 * Output-shaft turns -> what each drive is sent.
 *
 * The drive's position units follow ITS ENCODER, not its gearbox. The KNEE
 * reads its load-side encoder, so one drive turn is one output turn and a
 * gait value goes to it unscaled. HIP and ANKLE read the motor side of their
 * 47:1 and 9:1, so everything sent to them - position and velocity
 * feedforward alike - is multiplied by that ratio.
 *
 * This table is about POSITION units only. It is not a gearbox table:
 * test_leg_torque.c keeps a separate s_gear[] = { 47, 47, 9 } because torque
 * always has to cross the reduction, whatever the encoder is doing.
 */
static const float s_cmd_scale[JOINT_COUNT] = { 47.0f, 1.0f, 9.0f };

/*
 * Controller gains, pushed to every live drive from legtest_init() so all
 * three S1s are configured from THIS TABLE instead of one at a time in
 * odrivetool. Change a number here, rebuild, reflash - all three follow.
 *
 * GAIN_KEEP leaves whatever is saved in that drive untouched. These are
 * runtime writes to controller.config.*; nothing here calls
 * save_configuration(), so a power cycle reverts them. That is deliberate -
 * it makes a bad number recoverable by cycling power rather than by digging
 * through odrivetool.
 *
 * 20 / 1.0 / 5.0 is where the tuning campaign in the README finished: 0.167
 * -> 0.5 -> 1.0 on vel_gain, halving the RMS error each round, 12.16 deg ->
 * 2.14 -> 0.75. pos_gain is dimensionless (turn/s per turn) so it carries
 * across unchanged.
 *
 * EXPECT TO RAISE vel_gain. That campaign ran before every axis was moved to
 * its load-side encoder, and vel_gain is Nm of MOTOR torque per encoder
 * turn/s. Reading the load side divides the measured velocity by the gear
 * ratio, so the same number now produces far less torque for the same physical
 * motion. The evidence is already in the logs: the knee, which was always
 * load-side, barely moved on these numbers - 3.3 deg commanded, 0.25 deg
 * achieved - while the hip on its motor-side encoder tracked well on them.
 *
 * So this is a deliberately soft, known-safe starting point, not a finished
 * answer. Double vel_gain (and vi with it) between runs and watch the RMS
 * error in the capture, exactly as the README describes. Stop when it stops
 * improving or the capture starts to show oscillation. Starting soft and
 * climbing is the safe direction; starting stiff can oscillate against the
 * mechanism.
 */
#define GAIN_KEEP  (-1.0f)

static const float s_pos_gain[JOINT_COUNT]     = { 20.0f, 20.0f, 20.0f };
static const float s_vel_gain[JOINT_COUNT]     = {  1.0f,  1.0f,  1.0f };
static const float s_vel_int_gain[JOINT_COUNT] = {  5.0f,  5.0f,  5.0f };

#define LEGTEST_GAIT_RELATIVE        1
#define LEGTEST_MAX_SWING_DEG        20.0f

static float s_auto_offset[JOINT_COUNT];

static float joint_cmd(int j, float out_turns)
{
    return (out_turns + s_zero_offset[j] + s_auto_offset[j]) * s_cmd_scale[j];
}

static const float s_limit_deg[JOINT_COUNT] = { 25.0f, 35.0f, 35.0f };
#define LEGTEST_LIMIT_MARGIN_DEG     3.0f

static uint8_t s_arm_blocked;

static uint8_t limits_ok(const float *entry_from)
{
    uint8_t ok = 1;
    float   g0[GAIT_JOINTS];

    gait_sample(0.0f, g0);

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (!s_joint_live[j]) { continue; }

        float lim = s_limit_deg[j];
        float lo = 1e9f, hi = -1e9f;

        for (int s = 0; s < GAIT_SAMPLES; s++)
        {
            float v = g_gait_turns[s][s_gait_col[j]];
            if (v < lo) { lo = v; }
            if (v > hi) { hi = v; }
        }

#if LEGTEST_GAIT_RELATIVE
        float start = g0[s_gait_col[j]];
        float up    = (hi - start) * 360.0f;
        float down  = (start - lo) * 360.0f;
        float swing = (up > down) ? up : down;

        printf("  %-9s swings +%.1f / -%.1f deg from where it is now"
               " (travel is +/-%.0f)\r\n",
               s_joint_name[j], (double)up, (double)down, (double)lim);

        if (swing > LEGTEST_MAX_SWING_DEG)
        {
            printf("!! %s: %.1f deg of swing exceeds the %.1f deg cap.\r\n",
                   s_joint_name[j], (double)swing,
                   (double)LEGTEST_MAX_SWING_DEG);
            ok = 0;
        }

        (void)entry_from;
#else
        float at_deg = (entry_from[j] / s_cmd_scale[j]) * 360.0f;

        if ((at_deg > lim) || (at_deg < -lim))
        {
            printf("!! %s: measured pose %+.1f deg is outside its +/-%.0f deg"
                   " travel.\r\n", s_joint_name[j], (double)at_deg,
                   (double)lim);
            printf("   The joint cannot physically be there, so this drive's"
                   " zero is not the robot's\r\n"
                   "   zero. Set it per docs/ZEROING.md before running.\r\n");
            ok = 0;
            continue;
        }

        float lo_deg = (lo + s_zero_offset[j]) * 360.0f;
        float hi_deg = (hi + s_zero_offset[j]) * 360.0f;
        float worst  = (hi_deg > -lo_deg) ? hi_deg : -lo_deg;
        float clear  = lim - worst;

        if (clear < 0.0f)
        {
            printf("!! %s: gait reaches %+.1f..%+.1f deg, past its"
                   " +/-%.0f deg travel.\r\n", s_joint_name[j],
                   (double)lo_deg, (double)hi_deg, (double)lim);
            ok = 0;
        }
        else if (clear < LEGTEST_LIMIT_MARGIN_DEG)
        {
            printf("!! %s: gait clears its stop by only %.1f deg,"
                   " want %.1f.\r\n", s_joint_name[j], (double)clear,
                   (double)LEGTEST_LIMIT_MARGIN_DEG);
            printf("   Reduce the amplitude or re-centre with"
                   " s_zero_offset[%d].\r\n", j);
            ok = 0;
        }
#endif
    }

    return ok;
}

#define LEGTEST_SCAN_MS              2000u
#define LEGTEST_TRACE_RX             0
#define LEGTEST_LISTEN_ONLY          0
#define TRACE_LEN                    96u
#define LEGTEST_ENABLE_CLOSED_LOOP   1
#define LEGTEST_ARM_DELAY_MS         3000u
#define LEGTEST_STOP_BUTTON          1
#define LEGTEST_MOTION_GAIT          1
#define LEGTEST_AMPLITUDE_TURNS      0.05f 
#define LEGTEST_FREQ_HZ              0.25f 
#define LEGTEST_GAIT_SPEED           1.0f
#define LEGTEST_GAIT_ENTRY_MS        2000u
#define LEGTEST_GAIT_CYCLES          10u
#define LEGTEST_GAIT_VEL_FF          1
#define LEGTEST_GAIT_TORQUE_FF       0
#define LEGTEST_GAIT_IDLE_AFTER      1
#define LEGTEST_CAPTURE              1
#define CAPTURE_HZ                   100u
#define CAPTURE_MAX                  2048u

#if LEGTEST_CAPTURE
typedef struct
{
    float cmd[JOINT_COUNT];
    float pos[JOINT_COUNT];
    float trq[JOINT_COUNT];
} cap_row_t;

static cap_row_t s_cap[CAPTURE_MAX];
static uint16_t  s_cap_n;
static uint8_t   s_cap_dumped;
#endif

#define LEGTEST_VEL_POKE             0
#define LEGTEST_VEL_POKE_TURNS_S     0.5f
#define LEGTEST_USE_CAN_FD           1
/*
 * Secondary Sample Point, in tq (1 tq = 12.5 ns at the 80 MHz kernel clock).
 *
 * The SSP is where the transmitter re-reads its own bit to check the bus
 * followed it, and it should sit at the SAME place in the bit as the receive
 * sample point - not merely past the transceiver's loop delay. 33 tq =
 * 412.5 ns = 82.5% of a 500 ns data bit, matching DataTimeSeg1/2 below.
 *
 * It was 20 tq (250 ns = 50%), which self-checked halfway through the bit
 * while receivers sampled at 75%. At 5 Mbit that offset was worse still: a
 * 200 ns bit with the SSP at 250 ns checked PAST THE END of the bit entirely,
 * which is the likelier cause of the transmit-only errors (TEC climbing to
 * bus-off with REC at 0) than the isolator distortion first suspected.
 */
#define LEGTEST_TDC_OFFSET           33u
#define LEGTEST_TX_DIV               1u
#define NODE_SILENT_TICKS            500u

#if LEGTEST_USE_CAN_FD
#define FRAME_US                     75u 
#else
#define FRAME_US                     121u
#endif

/* ===================================================================== */

#define ODRV_CMD_HEARTBEAT      0x001u
#define ODRV_CMD_SET_AXIS_STATE 0x007u
#define ODRV_CMD_GET_ENCODER    0x009u
#define ODRV_CMD_SET_CTRL_MODE  0x00Bu
#define ODRV_CMD_SET_INPUT_POS  0x00Cu
#define ODRV_CMD_SET_INPUT_VEL  0x00Du
#define ODRV_CTRL_MODE_VELOCITY 2u
#define ODRV_CTRL_MODE_POSITION 3u
#define ODRV_INPUT_MODE_PASSTHR 1u
#define ODRV_CMD_SET_POS_GAIN   0x01Au
#define ODRV_CMD_SET_VEL_GAINS  0x01Bu
#define ODRV_CMD_GET_TORQUES    0x01Cu
#define ODRV_AXIS_STATE_IDLE            1u
#define ODRV_AXIS_STATE_CLOSED_LOOP     8u

/*
 * Calibration states. Set_Axis_State (0x007) takes these exactly like IDLE or
 * CLOSED_LOOP - the drive runs the routine and returns to IDLE when it is done.
 *
 *   3  FULL_CALIBRATION_SEQUENCE    motor R/L, then the encoder offset
 *   4  MOTOR_CALIBRATION            resistance and inductance only, no motion
 *   7  ENCODER_OFFSET_CALIBRATION   finds the electrical angle; MOVES the motor
 *
 * These numbers are from the ODrive axis-state enum. They have not been
 * verified against the 0.6.12 documentation from here - the error and enum
 * pages return HTTP 403 - so confirm 3 and 7 in odrivetool before trusting a
 * joint to them.
 */
#define ODRV_AXIS_STATE_FULL_CALIB      3u
#define ODRV_AXIS_STATE_MOTOR_CALIB     4u
#define ODRV_AXIS_STATE_ENC_OFFSET_CALIB 7u

/*
 * Arbitrary parameter access (ODrive "CANSimple" SDO).
 *
 *   RxSdo  0x004  us -> drive   [ opcode | ep_lo | ep_hi | 0 | value(4) ]
 *   TxSdo  0x005  drive -> us   [   0    | ep_lo | ep_hi | 0 | value(4) ]
 *
 * Endpoint numbers are NOT stable: they move with every firmware and hardware
 * revision. The ones below come from flat_endpoints.json for fw 0.6.12 /
 * hw 5.2.0. Writing the wrong endpoint silently corrupts an unrelated setting,
 * so odrv_check_version() interrogates each drive with Get_Version first and
 * refuses to write to anything that does not match exactly.
 */
#define ODRV_CMD_GET_VERSION    0x000u
#define ODRV_CMD_RX_SDO         0x004u
#define ODRV_CMD_TX_SDO         0x005u
#define SDO_OP_READ             0x00u
#define SDO_OP_WRITE            0x01u

#define EP_JSON_HW_LINE   5u
#define EP_JSON_HW_VER    2u
#define EP_JSON_HW_VAR    0u
#define EP_JSON_FW_MAJOR  0u
#define EP_JSON_FW_MINOR  6u
#define EP_JSON_FW_REV    12u

#define EP_AXIS0_LOAD_ENCODER       294u   /* uint8, rw */
#define EP_AXIS0_COMMUT_ENCODER     295u   /* uint8, rw */
#define EP_SPI_ENC0_MAX_ERROR_RATE  673u   /* float, rw */
#define EP_SAVE_CONFIGURATION       718u   /* function  */

/*
 * spi_encoder0.config.max_error_rate - the fraction of SPI transactions the
 * drive will tolerate coming back bad before it declares the encoder estimate
 * missing and disarms. The knee has been dropping out mid-gait; loosening this
 * buys headroom while the harness is investigated. It does NOT fix bad wiring,
 * it only stops a handful of corrupt reads from ending the run.
 */
#define LEGTEST_SET_SPI_ERR_RATE    1
#define LEGTEST_SPI_MAX_ERROR_RATE  0.1f

/*
 * ---------------------------------------------------------------------------
 * CALIBRATION OVER CAN - READ THIS BEFORE SETTING IT TO 1
 * ---------------------------------------------------------------------------
 *
 * Which joints to run FULL_CALIBRATION_SEQUENCE on at boot, by index into the
 * joint tables. -1 ends the list. { -1 } means calibrate nothing.
 *
 * THE JOINT WILL MOVE. Encoder offset calibration spins the motor to find the
 * electrical angle, and it expects the shaft to turn freely. Through a 47:1
 * gearbox into a joint with 70 degrees of total travel, "freely" is not what it
 * gets: the routine can drive the joint into a hard stop and either fail with a
 * calibration error or load the mechanism against it.
 *
 * So run this with the joint DECOUPLED or with the leg supported and the travel
 * clear, watch it, and keep the stop button in reach. It is not something to
 * leave enabled - calibration belongs in a bring-up session, not in every boot
 * of a control test.
 *
 * Results are NOT saved. LEGTEST_SDO_SAVE persists them if you want them to
 * survive a power cycle, and that needs a reboot to take effect.
 */
#define LEGTEST_CALIBRATE_JOINTS    { 1 }
#define LEGTEST_CALIB_TIMEOUT_MS    5000u

/*
 * ---------------------------------------------------------------------------
 * WHICH ENCODER EACH AXIS USES
 * ---------------------------------------------------------------------------
 *
 * axis0.config.load_encoder and axis0.config.commutation_encoder hold an
 * EncoderId enum. The number behind ONBOARD_ENCODER0 is not published anywhere
 * reachable: it is absent from flat_endpoints.json, the encoders and
 * hardware-config pages name it only symbolically, and the API reference
 * returns HTTP 403. A wrong value in commutation_encoder tells the drive to
 * commutate off the wrong sensor, so it is left unset here rather than guessed.
 *
 * Read it off a drive once:
 *
 *     odrv0.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
 *     int(odrv0.axis0.config.commutation_encoder)      # <- this number
 *
 * Put that number in ODRV_ENC_ID_ONBOARD0 and the write below happens every
 * boot. Until then the firmware only READS the two values and prints them,
 * which is safe and is the only place a drive quietly running off a different
 * encoder than s_cmd_scale[] assumes would ever become visible.
 */
#define ODRV_ENC_ID_UNKNOWN     (-1)
#define ODRV_ENC_ID_ONBOARD0    ODRV_ENC_ID_UNKNOWN

/*
 * Per joint: the EncoderId to write, or ODRV_ENC_ID_UNKNOWN to leave that
 * drive's own configuration alone. Load and commutation are set to the same
 * source, which is what "run this axis off its onboard encoder" means.
 *
 * This table and s_cmd_scale[] have to agree. An axis on its onboard encoder
 * reads the motor side and needs its gear ratio; an axis on a load-side
 * encoder needs 1.0.
 */
#define LEGTEST_SET_ENCODER_SRC  1
static const int s_enc_src[JOINT_COUNT] = {
    ODRV_ENC_ID_ONBOARD0,      /* hip_pitch - motor side, x47 */
    ODRV_ENC_ID_UNKNOWN,       /* knee      - leave on its load-side encoder */
    ODRV_ENC_ID_ONBOARD0,      /* ankle     - motor side, x9  */
};

/* 1 = also persist it to the drive's flash (needs a power cycle to re-init). */
#define LEGTEST_SDO_SAVE            0

typedef struct
{
    float    pos;
    float    vel;
    float    torque;
    uint32_t axis_error;
    uint8_t  axis_state;
    uint32_t n_heartbeat;
    uint32_t n_encoder;
    uint32_t n_torque;
    uint32_t last_rx_tick;  
    float    cmd;          
} joint_t;

static volatile joint_t s_joint[JOINT_COUNT];

#if (MONITOR_COUNT > 0)
static volatile joint_t s_mon[MONITOR_COUNT];
#endif

/* filled by the RX handler while an SDO / version exchange is outstanding */
static volatile uint8_t  s_sdo_node;
static volatile uint16_t s_sdo_ep;
static volatile uint8_t  s_sdo_got;
static volatile uint8_t  s_sdo_val[4];
static volatile uint8_t  s_ver_node;
static volatile uint8_t  s_ver_got;
static volatile uint8_t  s_ver[8];

typedef struct
{
    uint32_t id;
    uint8_t  len;
    uint8_t  fd;       
    uint8_t  brs;       
    uint8_t  data[8];
} trace_t;

static volatile trace_t  s_trace[TRACE_LEN];
static volatile uint8_t  s_trace_head;
static volatile uint8_t  s_trace_tail;

static volatile uint16_t s_node_seen[64];
static volatile uint8_t  s_node_state[64];
static volatile uint32_t s_node_err[64];

static uint8_t s_scan_ok;         
static volatile uint32_t s_rx_total;
static volatile uint32_t s_rx_unknown; 

static volatile uint32_t s_tick_pending;
static uint32_t s_tick;
static uint32_t s_tx_fail;

static int joint_from_node(uint32_t node)
{
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (s_node_id[j] == node) { return j; }
    }
    return -1;
}

static int monitor_from_node(uint32_t node)
{
#if (MONITOR_COUNT > 0)
    for (int m = 0; m < MONITOR_COUNT; m++)
    {
        if (s_mon_node[m] == node) { return m; }
    }
#else
    (void)node;
#endif
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

#define TXQ_LEN   32u                  
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
static uint32_t    s_busoff_count;

static void tx_enqueue(uint32_t node, uint32_t cmd, const uint8_t *data, uint32_t len)
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

static uint8_t can_send(uint32_t node, uint32_t cmd, const uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef hdr;

#if LEGTEST_LISTEN_ONLY
    (void)node; (void)cmd; (void)data; (void)len;
    return 1;              
#else
    hdr.Identifier          = (node << 5) | cmd;
    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = (len == 0u) ? FDCAN_DLC_BYTES_0 :
                              (len == 8u) ? FDCAN_DLC_BYTES_8 :
                                            FDCAN_DLC_BYTES_4;
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
#endif
}

static void can_busoff_poll(void)
{
    FDCAN_ProtocolStatusTypeDef ps;

    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);

    /*
     * Error-passive is not bus-off, so the recovery below never fired for it.
     * But once the drives disarm and stop ACKing, TX fills, txfifo_free hits 0
     * and every queued frame is dropped forever - qdrop climbing by 2000/s
     * with tx=0 and rx=0, which is what a wedged bus looks like from here.
     * Drop the backlog so the queue is not spending the whole tick failing.
     */
    if (ps.ErrorPassive && (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u))
    {
        s_txq_tail = s_txq_head;
    }

    if (ps.BusOff == 0u)
    {
        return;
    }
    if (READ_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT) == 0u)
    {
        return;                 
    }

    s_busoff_count++;

    HAL_FDCAN_AbortTxRequest(&hfdcan1,
                             FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 |
                             FDCAN_TX_BUFFER2);

    CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
}

static void tx_pump(void)
{
    can_busoff_poll();

    while (s_txq_tail != s_txq_head)
    {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u)
        {
            break;                      
        }

        const txq_entry_t *e = &s_txq[s_txq_tail];

        if (!can_send(e->identifier >> 5, e->identifier & 0x1Fu, e->data, e->len))
        {
            break;                      
        }

        s_txq_tail = (uint8_t)((s_txq_tail + 1u) & TXQ_MASK);
    }
}

static int16_t ff_thousandths(float v)
{
    float scaled = v * 1000.0f;

    if (scaled >  32767.0f) { return  32767; }
    if (scaled < -32768.0f) { return -32768; }

    return (int16_t)scaled;
}

static void send_input_pos(int j, float pos, float vel_ff, float trq_ff)
{
    uint8_t  data[8];
    uint32_t bits;

    memcpy(&bits, &pos, sizeof(bits));
    data[0] = (uint8_t)(bits & 0xFFu);
    data[1] = (uint8_t)((bits >> 8) & 0xFFu);
    data[2] = (uint8_t)((bits >> 16) & 0xFFu);
    data[3] = (uint8_t)((bits >> 24) & 0xFFu);

    uint16_t v = (uint16_t)ff_thousandths(vel_ff);
    uint16_t q = (uint16_t)ff_thousandths(trq_ff);

    data[4] = (uint8_t)(v & 0xFFu);
    data[5] = (uint8_t)((v >> 8) & 0xFFu);
    data[6] = (uint8_t)(q & 0xFFu);
    data[7] = (uint8_t)((q >> 8) & 0xFFu);

    s_joint[j].cmd = pos;
    tx_enqueue(s_node_id[j], ODRV_CMD_SET_INPUT_POS, data, 8u);
}

#if LEGTEST_VEL_POKE
static void send_input_vel(int j, float vel)
{
    uint8_t  data[8];
    uint32_t bits;

    memcpy(&bits, &vel, sizeof(bits));
    data[0] = (uint8_t)(bits & 0xFFu);
    data[1] = (uint8_t)((bits >> 8) & 0xFFu);
    data[2] = (uint8_t)((bits >> 16) & 0xFFu);
    data[3] = (uint8_t)((bits >> 24) & 0xFFu);
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;   

    s_joint[j].cmd = vel;
    tx_enqueue(s_node_id[j], ODRV_CMD_SET_INPUT_VEL, data, 8u);
}
#endif

static void send_controller_mode(int j)
{
    uint8_t data[8];

#if LEGTEST_VEL_POKE
    data[0] = (uint8_t)ODRV_CTRL_MODE_VELOCITY;
#else
    data[0] = (uint8_t)ODRV_CTRL_MODE_POSITION;
#endif
    data[1] = 0; data[2] = 0; data[3] = 0;
    data[4] = (uint8_t)ODRV_INPUT_MODE_PASSTHR;
    data[5] = 0; data[6] = 0; data[7] = 0;

    tx_enqueue(s_node_id[j], ODRV_CMD_SET_CTRL_MODE, data, 8u);
}

/* Little-endian float32 into a byte buffer, the ODrive wire convention. */
static void put_f32(uint8_t *d, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    d[0] = (uint8_t)(bits & 0xFFu);
    d[1] = (uint8_t)((bits >> 8) & 0xFFu);
    d[2] = (uint8_t)((bits >> 16) & 0xFFu);
    d[3] = (uint8_t)((bits >> 24) & 0xFFu);
}

/*
 * Set_Pos_Gain (0x01A): float32 pos_gain.
 * Set_Vel_Gains (0x01B): float32 vel_gain, float32 vel_integrator_gain.
 *
 * Sent before Set_Controller_Mode so the loop is already tuned the moment the
 * axis is energised, rather than running one arming cycle on whatever the
 * drive had saved.
 */
static void send_gains(int j)
{
    uint8_t data[8];

    if (s_pos_gain[j] >= 0.0f)
    {
        put_f32(&data[0], s_pos_gain[j]);
        data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
        tx_enqueue(s_node_id[j], ODRV_CMD_SET_POS_GAIN, data, 4u);
    }

    if ((s_vel_gain[j] >= 0.0f) && (s_vel_int_gain[j] >= 0.0f))
    {
        put_f32(&data[0], s_vel_gain[j]);
        put_f32(&data[4], s_vel_int_gain[j]);
        tx_enqueue(s_node_id[j], ODRV_CMD_SET_VEL_GAINS, data, 8u);
    }

    if ((s_pos_gain[j] >= 0.0f) || (s_vel_gain[j] >= 0.0f))
    {
        printf("  gains -> %s: pos %.2f  vel %.3f  vel_int %.2f\r\n",
               s_joint_name[j], (double)s_pos_gain[j],
               (double)s_vel_gain[j], (double)s_vel_int_gain[j]);
    }
}

/*
 * Every live drive, once, from legtest_init(). The 1 kHz tick is not running
 * yet, so the queue is drained by hand here rather than by the control loop.
 */
static void send_all_gains(void)
{
    printf("\r\ncontroller gains, from s_pos_gain[] / s_vel_gain[] /"
           " s_vel_int_gain[]:\r\n");

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (!s_joint_live[j])
        {
            printf("  %-9s not on the bus - skipped\r\n", s_joint_name[j]);
            continue;
        }
        send_gains(j);
    }

    for (uint32_t spin = 0u;
         (s_txq_tail != s_txq_head) && (spin < 100000u);
         spin++)
    {
        tx_pump();
    }
    HAL_Delay(20);
    printf("\r\n");
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

static void disarm_all(void)
{
    s_txq_tail = s_txq_head;

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (s_joint_live[j]) { send_axis_state(j, ODRV_AXIS_STATE_IDLE); }
    }

    for (uint32_t spin = 0u;
         (s_txq_tail != s_txq_head) && (spin < 100000u);
         spin++)
    {
        tx_pump();
    }
}

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

            if (next != s_trace_tail)
            {
                s_trace[s_trace_head].id  = hdr.Identifier;
                s_trace[s_trace_head].len = 8u;
                s_trace[s_trace_head].fd  = (hdr.FDFormat == FDCAN_FD_CAN) ? 1u : 0u;
                s_trace[s_trace_head].brs = (hdr.BitRateSwitch == FDCAN_BRS_ON) ? 1u : 0u;
                for (int b = 0; b < 8; b++)
                {
                    s_trace[s_trace_head].data[b] = data[b];
                }
                s_trace_head = next;
            }
        }
#endif

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

        /* replies to a setup exchange, from any node - handled before the
           joint lookup so a monitor-only node can answer too */
        if (cmd == ODRV_CMD_GET_VERSION)
        {
            if (node == s_ver_node)
            {
                for (int b = 0; b < 8; b++) { s_ver[b] = data[b]; }
                s_ver_got = 1u;
            }
            continue;
        }
        if (cmd == ODRV_CMD_TX_SDO)
        {
            uint16_t ep = (uint16_t)data[1] | ((uint16_t)data[2] << 8);

            if ((node == s_sdo_node) && (ep == s_sdo_ep))
            {
                for (int b = 0; b < 4; b++) { s_sdo_val[b] = data[4 + b]; }
                s_sdo_got = 1u;
            }
            continue;
        }

        volatile joint_t *t = NULL;
        int j = joint_from_node(node);

        if (j >= 0)
        {
            t = &s_joint[j];
        }
#if (MONITOR_COUNT > 0)
        else
        {
            int m = monitor_from_node(node);
            if (m >= 0) { t = &s_mon[m]; }
        }
#endif
        if (t == NULL)
        {
            s_rx_unknown++;  
            continue;
        }

        t->last_rx_tick = s_tick;

        switch (cmd)
        {
        case ODRV_CMD_GET_ENCODER:
            t->pos = le_f32(&data[0]);
            t->vel = le_f32(&data[4]);
            t->n_encoder++;
            break;

        case ODRV_CMD_GET_TORQUES:
            t->torque = le_f32(&data[4]);
            t->n_torque++;
            break;

        case ODRV_CMD_HEARTBEAT:
            t->axis_error = le_u32(&data[0]);
            t->axis_state = data[4];
            t->n_heartbeat++;
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
#if LEGTEST_USE_CAN_FD
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, LEGTEST_TDC_OFFSET, 0u) != HAL_OK)
    {
        printf("!! HAL_FDCAN_ConfigTxDelayCompensation FAILED\r\n");
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
    {
        printf("!! HAL_FDCAN_EnableTxDelayCompensation FAILED\r\n");
    }
    printf("CAN FD: TDC on, SSP offset %u tq (%u ns past the measured"
           " loop delay)\r\n",
           (unsigned)LEGTEST_TDC_OFFSET, (unsigned)(LEGTEST_TDC_OFFSET * 25u / 2u));
#else
    /*
     * Classic frames are 1.6x the airtime of FD+BRS, so say what this run will
     * cost before it starts rather than leaving it to be discovered at 80%.
     */
    {
        uint32_t per_node = 1000u   /* Set_Input_Pos, one per tick   */
                          + 1250u;  /* heartbeat + encoder + torques */
        uint32_t load = (per_node * JOINT_COUNT * FRAME_US) / 10000u;

        printf("CAN: CLASSIC 2.0, no BRS, 1 Mbit - CAN-FD is DISABLED\r\n");
        printf("     %u us per frame. %d node(s) at current rates ~= %lu%% of"
               " the bus.\r\n",
               (unsigned)FRAME_US, JOINT_COUNT, (unsigned long)load);
        if (load > 60u)
        {
            printf("     !! above 60%% - drop encoder_msg_rate_ms on the drives"
                   " (5 = 200 Hz)\r\n");
        }
    }
#endif

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        printf("!! HAL_FDCAN_Start FAILED - the peripheral is not on the bus\r\n");
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        printf("!! HAL_FDCAN_ActivateNotification FAILED\r\n");
    }
}

#if LEGTEST_TRACE_RX
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

static void trace_drain(int max_lines)
{
    while ((s_trace_tail != s_trace_head) && (max_lines-- > 0))
    {
        volatile trace_t *t = &s_trace[s_trace_tail];

        uint32_t node = (t->id >> 5) & 0x3Fu;
        uint32_t cmd  = t->id & 0x1Fu;

        printf("  id 0x%03lX  node %-2lu cmd 0x%02lX %-17s %s%s ",
               (unsigned long)t->id, (unsigned long)node,
               (unsigned long)cmd, cmd_name(cmd),
               t->fd ? "FD " : "CLASSIC",
               t->brs ? "+BRS" : "    ");

        for (int b = 0; b < 8; b++)
        {
            printf("%02X ", (unsigned)t->data[b]);
        }

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

static void can_status(void);

/* ===================================================================== */
/*  Arbitrary parameter access - runs once at boot, before the timers      */
/* ===================================================================== */

/*
 * Blocking. Only ever called from legtest_init(), before the 1 kHz tick is
 * started, so pumping the queue and draining RX inline here is safe.
 */
static uint8_t sdo_wait(volatile uint8_t *flag, uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++)
    {
        tx_pump();
        legtest_on_rx();
        if (*flag) { return 1u; }
        HAL_Delay(1);
    }
    return 0u;
}

/*
 * The endpoint numbers this file hard-codes are only meaningful for one
 * firmware/hardware pair. Ask the drive what it is and refuse to write if it
 * disagrees - a mismatched write lands on whatever parameter happens to sit at
 * that number, which is far worse than not writing at all.
 */
static uint8_t odrv_check_version(uint8_t node)
{
    uint8_t empty[8] = { 0 };

    s_ver_node = node;
    s_ver_got  = 0u;
    tx_enqueue(node, ODRV_CMD_GET_VERSION, empty, 0u);

    if (!sdo_wait(&s_ver_got, 250u))
    {
        printf("  node %u: no reply to Get_Version - skipped\r\n", (unsigned)node);
        return 0u;
    }

    uint8_t hw_line = s_ver[1], hw_ver = s_ver[2], hw_var = s_ver[3];
    uint8_t fw_maj  = s_ver[4], fw_min = s_ver[5], fw_rev = s_ver[6];

    if ((hw_line != EP_JSON_HW_LINE) || (hw_ver != EP_JSON_HW_VER) ||
        (hw_var != EP_JSON_HW_VAR)   || (fw_maj != EP_JSON_FW_MAJOR) ||
        (fw_min != EP_JSON_FW_MINOR) || (fw_rev != EP_JSON_FW_REV))
    {
        printf("  node %u: hw %u.%u.%u fw %u.%u.%u does not match the endpoint\r\n"
               "          table (hw %u.%u.%u fw %u.%u.%u) - NOT writing.\r\n"
               "          Fetch that drive's own flat_endpoints.json.\r\n",
               (unsigned)node, hw_line, hw_ver, hw_var, fw_maj, fw_min, fw_rev,
               EP_JSON_HW_LINE, EP_JSON_HW_VER, EP_JSON_HW_VAR,
               EP_JSON_FW_MAJOR, EP_JSON_FW_MINOR, EP_JSON_FW_REV);
        return 0u;
    }
    return 1u;
}

static uint8_t sdo_read_f32(uint8_t node, uint16_t ep, float *out)
{
    uint8_t d[4];

    d[0] = SDO_OP_READ;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);
    d[3] = 0u;

    s_sdo_node = node;
    s_sdo_ep   = ep;
    s_sdo_got  = 0u;
    tx_enqueue(node, ODRV_CMD_RX_SDO, d, 4u);

    if (!sdo_wait(&s_sdo_got, 250u)) { return 0u; }

    *out = le_f32((const uint8_t *)s_sdo_val);
    return 1u;
}

static void sdo_write_f32(uint8_t node, uint16_t ep, float v)
{
    uint8_t d[8];

    d[0] = SDO_OP_WRITE;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);
    d[3] = 0u;
    put_f32(&d[4], v);

    tx_enqueue(node, ODRV_CMD_RX_SDO, d, 8u);
    for (uint32_t i = 0u; i < 20u; i++) { tx_pump(); HAL_Delay(1); }
}

static void sdo_call(uint8_t node, uint16_t ep)
{
    uint8_t d[4];

    d[0] = SDO_OP_WRITE;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);
    d[3] = 0u;

    tx_enqueue(node, ODRV_CMD_RX_SDO, d, 4u);
    for (uint32_t i = 0u; i < 20u; i++) { tx_pump(); HAL_Delay(1); }
}

static uint8_t sdo_read_u8(uint8_t node, uint16_t ep, uint8_t *out)
{
    uint8_t d[4];

    d[0] = SDO_OP_READ;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);
    d[3] = 0u;

    s_sdo_node = node;
    s_sdo_ep   = ep;
    s_sdo_got  = 0u;
    tx_enqueue(node, ODRV_CMD_RX_SDO, d, 4u);

    if (!sdo_wait(&s_sdo_got, 250u)) { return 0u; }

    *out = s_sdo_val[0];
    return 1u;
}

static void sdo_write_u8(uint8_t node, uint16_t ep, uint8_t v)
{
    uint8_t d[8];

    d[0] = SDO_OP_WRITE;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);
    d[3] = 0u;
    d[4] = v; d[5] = 0u; d[6] = 0u; d[7] = 0u;

    tx_enqueue(node, ODRV_CMD_RX_SDO, d, 8u);
    for (uint32_t i = 0u; i < 20u; i++) { tx_pump(); HAL_Delay(1); }
}

/*
 * Report which encoder each axis is using, and set it when a value was given.
 *
 * The read happens either way, because a drive running off a different encoder
 * than s_cmd_scale[] assumes is a 47x scaling error that looks exactly like a
 * tuning problem from the outside.
 */
static void encoder_source_one(int j)
{
    const uint8_t node = s_node_id[j];
    uint8_t load = 0u, commut = 0u;

    if (!s_joint_live[j])
    {
        printf("  %-9s not on the bus - skipped\r\n", s_joint_name[j]);
        return;
    }
    if (!odrv_check_version(node)) { return; }

    if (s_enc_src[j] != ODRV_ENC_ID_UNKNOWN)
    {
        sdo_write_u8(node, EP_AXIS0_LOAD_ENCODER,   (uint8_t)s_enc_src[j]);
        sdo_write_u8(node, EP_AXIS0_COMMUT_ENCODER, (uint8_t)s_enc_src[j]);
    }

    if (!sdo_read_u8(node, EP_AXIS0_LOAD_ENCODER, &load) ||
        !sdo_read_u8(node, EP_AXIS0_COMMUT_ENCODER, &commut))
    {
        printf("  %-9s no reply reading the encoder source\r\n",
               s_joint_name[j]);
        return;
    }

    printf("  %-9s load=%u  commutation=%u%s   cmd_scale %.0f\r\n",
           s_joint_name[j], (unsigned)load, (unsigned)commut,
           (s_enc_src[j] == ODRV_ENC_ID_UNKNOWN) ? "  [left alone]" : "  [set]",
           (double)s_cmd_scale[j]);

    if (load != commut)
    {
        printf("  %-9s !! load and commutation differ - s_cmd_scale follows"
               " load\r\n", "");
    }
}

static void encoder_source_all(void)
{
#if LEGTEST_SET_ENCODER_SRC
    printf("\r\nencoder source per axis (endpoint %u load / %u commutation)\r\n",
           (unsigned)EP_AXIS0_LOAD_ENCODER,
           (unsigned)EP_AXIS0_COMMUT_ENCODER);

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        encoder_source_one(j);
    }

#if (ODRV_ENC_ID_ONBOARD0 == ODRV_ENC_ID_UNKNOWN)
    printf("  ODRV_ENC_ID_ONBOARD0 is not set, so nothing was written."
           " Read it once in\r\n"
           "  odrivetool - int(odrv0.axis0.config.commutation_encoder) - and"
           " put it in\r\n"
           "  test_leg_can.c.\r\n");
#endif
    printf("\r\n");
#endif
}

static void spi_err_rate_one(uint8_t node, const char *name)
{
    float before = 0.0f, after = 0.0f;

    if (s_node_seen[node] == 0u)
    {
        printf("  node %u %-9s : silent on the scan - skipped\r\n",
               (unsigned)node, name);
        return;
    }
    if (!odrv_check_version(node)) { return; }

    if (!sdo_read_f32(node, EP_SPI_ENC0_MAX_ERROR_RATE, &before))
    {
        printf("  node %u %-9s : no reply reading the endpoint - skipped\r\n",
               (unsigned)node, name);
        return;
    }

    sdo_write_f32(node, EP_SPI_ENC0_MAX_ERROR_RATE,
                  LEGTEST_SPI_MAX_ERROR_RATE);

    if (!sdo_read_f32(node, EP_SPI_ENC0_MAX_ERROR_RATE, &after))
    {
        printf("  node %u %-9s : wrote %.3f but could not read it back\r\n",
               (unsigned)node, name, (double)LEGTEST_SPI_MAX_ERROR_RATE);
        return;
    }

    float want = LEGTEST_SPI_MAX_ERROR_RATE;
    float d    = (after > want) ? (after - want) : (want - after);

    printf("  node %u %-9s : max_error_rate %.4f -> %.4f%s\r\n",
           (unsigned)node, name, (double)before, (double)after,
           (d < 1e-6f) ? "" : "   !! DID NOT TAKE");

#if LEGTEST_SDO_SAVE
    if (d < 1e-6f)
    {
        printf("  node %u %-9s : save_configuration()\r\n", (unsigned)node, name);
        sdo_call(node, EP_SAVE_CONFIGURATION);
        HAL_Delay(500);
    }
#else
    (void)sdo_call;
#endif
}

/*
 * Run FULL_CALIBRATION_SEQUENCE on one joint and wait for it to finish.
 *
 * Blocking, and only called from legtest_init() before the timers start, like
 * everything else in this section. The drive returns itself to IDLE when the
 * routine completes, so "done" is axis_state back at IDLE with no error - and
 * an error is reported rather than swallowed, because a joint that failed
 * calibration will refuse closed loop later with no obvious reason.
 */
static uint8_t calibrate_joint(int j)
{
    const uint8_t node = s_node_id[j];

    if (!s_joint_live[j])
    {
        printf("  %-9s not on the bus - not calibrated\r\n", s_joint_name[j]);
        return 0u;
    }

    printf("  %-9s (node %u) FULL_CALIBRATION_SEQUENCE - THE JOINT WILL MOVE\r\n",
           s_joint_name[j], (unsigned)node);

    s_joint[j].axis_state = 0u;
    s_joint[j].axis_error = 0u;
    send_axis_state(j, ODRV_AXIS_STATE_FULL_CALIB);

    /* Let it leave IDLE first, so "already idle" is not read as "finished". */
    uint8_t started = 0u;

    for (uint32_t ms = 0u; ms < LEGTEST_CALIB_TIMEOUT_MS; ms++)
    {
        tx_pump();
        legtest_on_rx();
        HAL_Delay(1);

        if (s_joint[j].axis_error != 0u)
        {
            printf("  %-9s calibration FAILED, axis_error 0x%08lX\r\n",
                   s_joint_name[j], (unsigned long)s_joint[j].axis_error);
            return 0u;
        }
        if (!started)
        {
            if (s_joint[j].axis_state == ODRV_AXIS_STATE_FULL_CALIB)
            {
                started = 1u;
            }
            continue;
        }
        if (s_joint[j].axis_state == ODRV_AXIS_STATE_IDLE)
        {
            printf("  %-9s calibration complete, back in IDLE\r\n",
                   s_joint_name[j]);
            return 1u;
        }
    }

    printf("  %-9s calibration TIMED OUT after %u ms (state %u)\r\n",
           s_joint_name[j], (unsigned)LEGTEST_CALIB_TIMEOUT_MS,
           (unsigned)s_joint[j].axis_state);
    return 0u;
}

static void run_calibration(void)
{
    static const int want[] = LEGTEST_CALIBRATE_JOINTS;

    if ((sizeof(want) / sizeof(want[0]) == 1u) && (want[0] < 0))
    {
        return;                     /* nothing asked for */
    }

    printf("\r\ncalibration\r\n");

    for (unsigned k = 0; k < (sizeof(want) / sizeof(want[0])); k++)
    {
        if ((want[k] < 0) || (want[k] >= JOINT_COUNT)) { continue; }
        (void)calibrate_joint(want[k]);
    }
    printf("\r\n");
}

static void apply_encoder_config(void)
{
#if LEGTEST_SET_SPI_ERR_RATE
    printf("\r\nspi_encoder0.config.max_error_rate -> %.3f  "
           "(endpoint %u, fw %u.%u.%u / hw %u.%u.%u)\r\n",
           (double)LEGTEST_SPI_MAX_ERROR_RATE,
           (unsigned)EP_SPI_ENC0_MAX_ERROR_RATE,
           EP_JSON_FW_MAJOR, EP_JSON_FW_MINOR, EP_JSON_FW_REV,
           EP_JSON_HW_LINE, EP_JSON_HW_VER, EP_JSON_HW_VAR);

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        spi_err_rate_one(s_node_id[j], s_joint_name[j]);
    }
#if (MONITOR_COUNT > 0)
    for (int m = 0; m < MONITOR_COUNT; m++)
    {
        spi_err_rate_one(s_mon_node[m], s_mon_name[m]);
    }
#endif

#if !LEGTEST_SDO_SAVE
    printf("  not saved to flash - this is a runtime write and is lost on the\r\n"
           "  next power cycle. Set LEGTEST_SDO_SAVE 1 to persist it.\r\n");
#endif
    printf("\r\n");
#endif
}

static void bus_scan(void)
{
    printf("\r\nscanning the bus for %u ms - not transmitting...\r\n",
           (unsigned)LEGTEST_SCAN_MS);

    for (uint32_t i = 0; i < LEGTEST_SCAN_MS; i += 10u)
    {
        HAL_Delay(10);
        legtest_on_rx();          
#if LEGTEST_TRACE_RX
        trace_drain(2);           
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
               (joint_from_node((uint32_t)n) >= 0)   ? "   <-- configured" :
               (monitor_from_node((uint32_t)n) >= 0) ? "   <-- monitored" : "");
    }

    can_status();

    if (found == 0)
    {
        printf("  NOTHING ON THE BUS.\r\n");
    }

    /*
     * One silent drive used to disable closed loop for the whole leg. It no
     * longer does: each joint stands or falls on its own, and the run goes
     * ahead with whatever answered. Only a completely silent bus stops it.
     */
    int live = 0;

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        s_joint_live[j] = (s_node_seen[s_node_id[j]] != 0u) ? 1u : 0u;

        if (s_joint_live[j])
        {
            live++;
        }
        else
        {
            printf("  !! node %u (%s) never answered - that joint is SKIPPED\r\n"
                   "     (not armed, not commanded, not captured). The rest of\r\n"
                   "     the leg still runs.\r\n",
                   (unsigned)s_node_id[j], s_joint_name[j]);
        }
    }

    s_scan_ok = (live > 0) ? 1u : 0u;

    if (!s_scan_ok)
    {
        printf("  !! no joint answered at all - closed loop DISABLED\r\n");
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
    s_busoff_count = 0;
    s_arm_blocked  = 0;

    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Init(LED_RED);
    BSP_LED_Off(LED_YELLOW);

    bus_setup();

#if LEGTEST_STOP_BUTTON
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
#endif

    bus_scan();
    encoder_source_all();
    apply_encoder_config();
    run_calibration();
    send_all_gains();

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);

    printf("\r\n=========== CAN-FD SINGLE LEG TEST ===========\r\n");
}

static const char *lec_name(uint32_t lec)
{
    switch (lec)
    {
    case 0: return "none";
    case 1: return "STUFF - bitrate mismatch or noise";
    case 2: return "FORM - frame format, often FD vs classic";
    case 3: return "ACK - we transmitted and NOBODY answered";
    case 4: return "BIT1 - drove recessive, read dominant";
    case 5: return "BIT0 - drove dominant, read recessive (shorted? no xcvr?)";
    case 6: return "CRC";
    default: return "no change since last read";
    }
}

static void can_status(void)
{
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef  ec;

    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);

    printf("    can: TEC=%lu REC=%lu txfifo_free=%lu%s%s%s\r\n",
           (unsigned long)ec.TxErrorCnt, (unsigned long)ec.RxErrorCnt,
           (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1),
           ps.BusOff       ? "  [BUS-OFF]"       : "",
           ps.ErrorPassive ? "  [ERROR-PASSIVE]" : "",
           ps.Warning      ? "  [WARNING]"       : "");
    printf("    last error: %s\r\n", lec_name(ps.LastErrorCode));

    if (ps.BusOff)
    {
        printf("    -> BUS-OFF (event %lu): rejoining\r\n",
               (unsigned long)s_busoff_count);
    }
    else if (s_busoff_count != 0u)
    {
        printf("    -> recovered from %lu bus-off event(s) so far\r\n",
               (unsigned long)s_busoff_count);
    }
}

static void report(void)
{
    uint32_t now = s_tick;
    int      silent = 0;

    static uint32_t prev_tx_ok, prev_rx;

    uint32_t d_tx = s_tx_ok - prev_tx_ok;
    uint32_t d_rx = s_rx_total - prev_rx;
    prev_tx_ok = s_tx_ok;
    prev_rx    = s_rx_total;

    uint32_t load_pct = ((d_tx + d_rx) * FRAME_US) / 10000u;  

    printf("--- t=%lus  tx=%lu rx=%lu (unknown=%lu)  txfail=%lu qdrop=%lu  "
           "bus~%lu%% ---\r\n",
           (unsigned long)(now / 1000u), (unsigned long)d_tx,
           (unsigned long)d_rx, (unsigned long)s_rx_unknown,
           (unsigned long)s_tx_fail, (unsigned long)s_txq_drop,
           (unsigned long)load_pct);

    can_status();

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        joint_t v;
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
                   "state=%u err=0x%08lX  hb=%lu enc=%lu trq=%lu  cmd=%+.4f err=%+.4f\r\n",
                   (unsigned)s_node_id[j], s_joint_name[j],
                   (double)v.pos, (double)v.vel, (double)v.torque,
                   (unsigned)v.axis_state, (unsigned long)v.axis_error,
                   (unsigned long)v.n_heartbeat, (unsigned long)v.n_encoder,
                   (unsigned long)v.n_torque, (double)v.cmd,
                   (double)(v.cmd - v.pos));
        }
    }

#if (MONITOR_COUNT > 0)
    for (int m = 0; m < MONITOR_COUNT; m++)
    {
        joint_t v;
        uint32_t pm = critical_enter();
        v = *(joint_t *)&s_mon[m];
        critical_exit(pm);

        uint8_t alive = ((now - v.last_rx_tick) < NODE_SILENT_TICKS) &&
                        ((v.n_heartbeat + v.n_encoder + v.n_torque) > 0u);

        /* monitor nodes never set `silent` - they must not gate arming */
        if (!alive)
        {
            printf("  node %u %s : ---- SILENT ----  (hb=%lu enc=%lu trq=%lu)"
                   "   [monitor]\r\n",
                   (unsigned)s_mon_node[m], s_mon_name[m],
                   (unsigned long)v.n_heartbeat, (unsigned long)v.n_encoder,
                   (unsigned long)v.n_torque);
        }
        else
        {
            printf("  node %u %s : pos=%+8.4f vel=%+8.3f trq=%+7.3f "
                   "state=%u err=0x%08lX  hb=%lu enc=%lu trq=%lu   [monitor]\r\n",
                   (unsigned)s_mon_node[m], s_mon_name[m],
                   (double)v.pos, (double)v.vel, (double)v.torque,
                   (unsigned)v.axis_state, (unsigned long)v.axis_error,
                   (unsigned long)v.n_heartbeat, (unsigned long)v.n_encoder,
                   (unsigned long)v.n_torque);
        }
    }
#endif

    if (silent == 0)
    {
        BSP_LED_Off(LED_RED);
    }
    printf("\r\n");
}

#if LEGTEST_CAPTURE
static void cap_dump(void)
{
    if (s_cap_dumped || (s_cap_n == 0u))
    {
        return;
    }
    s_cap_dumped = 1;

    printf("\r\n===== TRAJECTORY, %u samples at %u Hz =====\r\n",
           (unsigned)s_cap_n, (unsigned)CAPTURE_HZ);

    /*
     * Per-joint summary, in BOTH the drive's turns and output degrees.
     *
     * Turns are what went on the wire; degrees are what the leg did. They
     * differ by s_cmd_scale, which is 47 on the hip while it runs on its
     * motor-side encoder - so the same "1.8 turns" is 13.8 deg at the hip and
     * would be 648 deg at the knee. Printing only one of the two is how a
     * gear-ratio mistake hides.
     *
     * "tracking" is achieved travel over commanded travel. It is the single
     * number that says whether the joint followed the trajectory or merely
     * received it: a stalled joint still gets perfect commands.
     */
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (!s_joint_live[j])
        {
            printf("  %-9s not on the bus - not commanded, nothing captured\r\n\r\n",
                   s_joint_name[j]);
            continue;
        }

        float cmd_lo = 1e9f, cmd_hi = -1e9f;
        float pos_lo = 1e9f, pos_hi = -1e9f;
        float err_max = 0.0f, trq_max = 0.0f;
        float jump_max = 0.0f;
        float prev = 0.0f;
        uint16_t bad = 0, good = 0;

        /*
         * A dead encoder does not report itself as dead: it reports NaN, or a
         * number that has jumped somewhere the joint cannot physically have
         * gone in one millisecond. Both then poison min/max and produce a
         * tracking figure that looks like a control problem and is not one.
         *
         * So: drop the NaNs, and measure the largest single-sample step. If
         * one step exceeds the whole commanded travel, the joint did not move
         * that far - the measurement did, and every number below is suspect.
         */
        for (uint16_t n = 0; n < s_cap_n; n++)
        {
            float c = s_cap[n].cmd[j];
            float o = s_cap[n].pos[j];

            if (isnan(c) || isnan(o)) { bad++; continue; }

            float e = (c > o) ? (c - o) : (o - c);
            float t = (s_cap[n].trq[j] > 0.0f) ? s_cap[n].trq[j]
                                               : -s_cap[n].trq[j];

            if (good != 0u)
            {
                float d = (o > prev) ? (o - prev) : (prev - o);
                if (d > jump_max) { jump_max = d; }
            }
            prev = o;
            good++;

            if (c < cmd_lo) { cmd_lo = c; }
            if (c > cmd_hi) { cmd_hi = c; }
            if (o < pos_lo) { pos_lo = o; }
            if (o > pos_hi) { pos_hi = o; }
            if (e > err_max) { err_max = e; }
            if (t > trq_max) { trq_max = t; }
        }

        if (good == 0u)
        {
            printf("  %-9s NO VALID SAMPLES - encoder never reported\r\n\r\n",
                   s_joint_name[j]);
            continue;
        }

        float cmd_pp = cmd_hi - cmd_lo;
        float pos_pp = pos_hi - pos_lo;
        float deg    = 360.0f / s_cmd_scale[j];   /* turns -> output degrees */
        float track  = (cmd_pp > 1e-6f) ? (100.0f * pos_pp / cmd_pp) : 0.0f;

        printf("  %-9s commanded %+.4f .. %+.4f = %.4f turns (%.2f deg out)\r\n",
               s_joint_name[j], (double)cmd_lo, (double)cmd_hi,
               (double)cmd_pp, (double)(cmd_pp * deg));
        printf("  %-9s achieved  %+.4f .. %+.4f = %.4f turns (%.2f deg out)\r\n",
               "", (double)pos_lo, (double)pos_hi,
               (double)pos_pp, (double)(pos_pp * deg));
        printf("  %-9s tracking %.0f%%   worst error %.4f turns (%.2f deg out)"
               "   peak torque %.3f Nm\r\n",
               "", (double)track, (double)err_max,
               (double)(err_max * deg), (double)trq_max);

        if ((bad != 0u) || (jump_max > cmd_pp))
        {
            printf("  %-9s !! ENCODER FAULT - %u NaN sample(s), biggest"
                   " one-tick step %.4f turns (%.2f deg out).\r\n",
                   "", (unsigned)bad, (double)jump_max,
                   (double)(jump_max * deg));
            printf("  %-9s    The joint cannot move that fast. The numbers"
                   " above are the encoder\r\n"
                   "  %-9s    dropping out, not the controller missing."
                   " Fix the encoder first.\r\n",
                   "", "");
        }
        printf("\r\n");
    }

    printf("t_s");
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (!s_joint_live[j]) { continue; }
        printf(",%s_cmd,%s_pos,%s_err",
               s_joint_name[j], s_joint_name[j], s_joint_name[j]);
    }
    printf("\r\n");

    for (uint16_t n = 0; n < s_cap_n; n++)
    {
        printf("%.3f", (double)n / (double)CAPTURE_HZ);
        for (int j = 0; j < JOINT_COUNT; j++)
        {
            if (!s_joint_live[j]) { continue; }
            printf(",%.5f,%.5f,%.5f",
                   (double)s_cap[n].cmd[j], (double)s_cap[n].pos[j],
                   (double)(s_cap[n].cmd[j] - s_cap[n].pos[j]));
        }
        printf("\r\n");
    }
    printf("===== end =====\r\n\r\n");
}
#endif

void legtest_run(void)
{
    uint32_t report_tick = 0;
    uint32_t beat        = 0;
#if (LEGTEST_TX_DIV != 1u)
    int      tx_slot     = 0;
#endif

    for (;;)
    {
        tx_pump();

        if (s_tick_pending == 0u)
        {
            continue;
        }

        uint32_t pm = critical_enter();
        s_tick_pending = 0;
        critical_exit(pm);

        s_tick++;

#if LEGTEST_STOP_BUTTON
        if (!s_stopped && (BSP_PB_GetState(BUTTON_USER) == 0))
        {
            s_stopped = 1;
            disarm_all();
            printf("\r\n*** STOPPED by user button - axes commanded to IDLE. ***\r\n\r\n");
        }
#endif

#if LEGTEST_TRACE_RX
        if ((s_tick % 200u) == 0u)
        {
            trace_drain(3);
        }
#endif

        float target[JOINT_COUNT] = { 0.0f };
        float target_vel[JOINT_COUNT] = { 0.0f };
        float target_trq[JOINT_COUNT] = { 0.0f };

#if LEGTEST_ENABLE_CLOSED_LOOP
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
                first[j] = joint_cmd(j, all[s_gait_col[j]]);
            }

            if (since_arm == 1u)
            {
                /*
                 * A joint that is not on the bus has no encoder count and
                 * never will. Requiring one from every joint means a single
                 * absent drive pins the whole leg in the hold-pose branch
                 * forever - armed, commanded to stand still, looking healthy.
                 * Only the joints we are actually driving get a vote.
                 */
                s_entry_ok = 0;
                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    s_entry_from[j] = s_joint[j].pos;

                    if (!s_joint_live[j]) { continue; }

                    if (s_joint[j].n_encoder == 0u)
                    {
                        printf("!! %s is on the bus but has sent no encoder"
                               " data - holding pose.\r\n", s_joint_name[j]);
                        s_entry_ok = 0;
                        break;
                    }
                    s_entry_ok = 1;
                }

#if LEGTEST_GAIT_RELATIVE
                if (s_entry_ok)
                {
                    float g0[GAIT_JOINTS];
                    gait_sample(0.0f, g0);

                    for (int j = 0; j < JOINT_COUNT; j++)
                    {
                        s_auto_offset[j] = (s_entry_from[j] / s_cmd_scale[j])
                                           - g0[s_gait_col[j]]
                                           - s_zero_offset[j];
                    }
                }
#endif

                if (s_entry_ok && !limits_ok(s_entry_from))
                {
                    s_entry_ok   = 0;
                    s_arm_blocked = 1;
                }
            }

            if (!s_entry_ok)
            {
                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    target[j] = s_joint[j].pos;
                }
            }
            else if (since_arm < LEGTEST_GAIT_ENTRY_MS)
            {
                /*
                 * Straight line from the pose measured at arming to gait
                 * sample 0.
                 *
                 * The setpoint is a function of TIME, deliberately, not of the
                 * live encoder. A setpoint recomputed from the measurement
                 * every tick can never build an error larger than one tick's
                 * step - and error is what a position loop turns into torque,
                 * so the drive could never be asked for enough force to move a
                 * joint that is not already moving. It would follow the leg
                 * around rather than drive it, and would track a droop under
                 * gravity straight down instead of resisting it.
                 *
                 * Being measurement-independent is what lets the error grow
                 * when the joint lags, which is the whole mechanism by which
                 * the drive decides to push harder.
                 *
                 * A straight line has one constant velocity: the whole
                 * distance over the whole ramp. The entry deserves the same
                 * feedforward as the gait - it is the move most likely to be
                 * large, and the one where the drive is coldest.
                 */
                float a      = (float)since_arm / (float)LEGTEST_GAIT_ENTRY_MS;
                float ramp_s = (float)LEGTEST_GAIT_ENTRY_MS * 0.001f;

                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    target[j] = s_entry_from[j] +
                                (first[j] - s_entry_from[j]) * a;

                    target_vel[j] = (first[j] - s_entry_from[j]) / ramp_s;
                }
            }
            else
            {
                float t = (float)(since_arm - LEGTEST_GAIT_ENTRY_MS) * 0.001f;

                s_gait_running = 1;      
                s_gait_phase = (t * LEGTEST_GAIT_SPEED) / GAIT_CYCLE_S;

#if (LEGTEST_GAIT_CYCLES > 0u)
                if (s_gait_phase >= (float)LEGTEST_GAIT_CYCLES)
                {
                    s_gait_phase = (float)LEGTEST_GAIT_CYCLES;

                    if (!s_gait_done)
                    {
                        s_gait_done = 1;

#if LEGTEST_GAIT_IDLE_AFTER
                        s_stopped = 1;
                        disarm_all();
#endif
#if LEGTEST_CAPTURE
                        cap_dump();
#endif
                    }
                }
#endif

                float all_vel[GAIT_JOINTS];
                float safe_phase;

                if (s_gait_done) {
                    safe_phase = 1.0f; // Hold the last frame when finished
                } else {
                    safe_phase = fmodf(s_gait_phase, 1.0f); // Cleanly loop 0.0 -> 1.0
                }

                gait_sample_vel(safe_phase, all, all_vel);
                float vscale = s_gait_done ? 0.0f : LEGTEST_GAIT_SPEED;

                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    /*
                     * The trajectory's POSITION, not an integral of its
                     * velocity seeded from the encoder.
                     *
                     * Commanding measured_position + one tick of velocity has
                     * no absolute reference anywhere in it: the setpoint is a
                     * function of the measurement with unity gain, so every
                     * overshoot and every bit of telemetry noise is folded
                     * straight back into the next command and kept. There is
                     * no term that pulls it back to where the trajectory says
                     * the joint should be, so it walks away and never returns.
                     * The hip, whose delta is multiplied by its 47:1 scaling,
                     * integrated itself to 64 motor turns against a gait that
                     * only spans 1.8.
                     *
                     * joint_cmd() carries the absolute answer: gait position,
                     * re-centred once at arming, scaled to the drive's shaft.
                     */
                    target[j]     = joint_cmd(j, all[s_gait_col[j]]);
                    target_vel[j] = all_vel[s_gait_col[j]] * vscale
                                    * s_cmd_scale[j];
                }
            }
#else
            /* INCREMENTAL MODIFICATION FOR SINE WAVE FALLBACK */
            float t     = (float)since_arm * 0.001f;
            float w     = 2.0f * 3.14159265f * LEGTEST_FREQ_HZ;
            float phase = w * t;
            float dv    = LEGTEST_AMPLITUDE_TURNS * w * cosf(phase);

            for (int j = 0; j < JOINT_COUNT; j++)
            {
                target_vel[j] = dv;
                float delta_pos = dv * 0.001f;

                target[j]     = s_joint[j].pos + delta_pos; // RELATIVE TO CURRENT
            }
#endif
        }
#endif

        if (!s_stopped && ((s_tick % LEGTEST_TX_DIV) == 0u))
        {
#if (LEGTEST_TX_DIV == 1u)
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                if (!s_joint_live[j]) { continue; }
#if LEGTEST_VEL_POKE
                send_input_vel(j, (s_tick > LEGTEST_ARM_DELAY_MS)
                                      ? LEGTEST_VEL_POKE_TURNS_S : 0.0f);
#else
                send_input_pos(j, target[j],
                               LEGTEST_GAIT_VEL_FF    ? target_vel[j] : 0.0f,
                               LEGTEST_GAIT_TORQUE_FF ? target_trq[j] : 0.0f);
#endif
            }
#else
#if LEGTEST_VEL_POKE
            send_input_vel(tx_slot, (s_tick > LEGTEST_ARM_DELAY_MS)
                                        ? LEGTEST_VEL_POKE_TURNS_S : 0.0f);
#else
            send_input_pos(tx_slot, target[tx_slot],
                           LEGTEST_GAIT_VEL_FF    ? target_vel[tx_slot] : 0.0f,
                           LEGTEST_GAIT_TORQUE_FF ? target_trq[tx_slot] : 0.0f);
#endif
            tx_slot = (tx_slot + 1) % JOINT_COUNT;
#endif
        }

#if LEGTEST_CAPTURE
        if ((s_cap_n < CAPTURE_MAX) && s_gait_running &&
            (s_joint[0].axis_state == ODRV_AXIS_STATE_CLOSED_LOOP) &&
            ((s_tick % (1000u / CAPTURE_HZ)) == 0u))
        {
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                s_cap[s_cap_n].cmd[j] = s_joint[j].cmd;
                s_cap[s_cap_n].pos[j] = s_joint[j].pos;
                s_cap[s_cap_n].trq[j] = s_joint[j].torque;
            }
            s_cap_n++;
        }
#endif

#if LEGTEST_ENABLE_CLOSED_LOOP
        if ((s_tick <= LEGTEST_ARM_DELAY_MS) && ((s_tick % 1000u) == 0u))
        {
            printf("*** ARMING in %lu s - motors will become live ***\r\n",
                   (unsigned long)((LEGTEST_ARM_DELAY_MS - s_tick) / 1000u));
        }

        if (!s_stopped && s_scan_ok && !s_arm_blocked &&
            (s_tick > LEGTEST_ARM_DELAY_MS) && ((s_tick % 2000u) == 500u))
        {
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                if (s_joint_live[j] &&
                    (s_joint[j].axis_state != ODRV_AXIS_STATE_CLOSED_LOOP) &&
                    (s_joint[j].axis_error == 0u) &&
                    (s_joint[j].n_heartbeat > 0u))
                {
                    send_gains(j);
                    send_controller_mode(j);
                    send_axis_state(j, ODRV_AXIS_STATE_CLOSED_LOOP);
                }
            }
        }
#endif

        if (++beat >= 500u)
        {
            beat = 0;
            BSP_LED_Toggle(LED_GREEN);
        }

        if (++report_tick >= 1000u)
        {
            report_tick = 0;
            report();

            pm = critical_enter();
            s_tick_pending = 0;
            critical_exit(pm);
        }
    }
}