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
static uint8_t       s_gait_done;
static uint8_t       s_gait_running;
static uint8_t       s_stopped;      /* latched by the user button */



static const uint8_t s_node_id[JOINT_COUNT]  = { 3 };
/*
 * Which gait column drives this node. Independent of what the actuator is
 * physically bolted to - on the bench the drive is a knee, but any column can
 * be played through it to test a different range of motion.
 *
 * Ranges, from tools/gen_gait.py:
 *   HIP_ROLL    -0.0047 .. +0.0191 turns   ( 8.5 deg)
 *   HIP_PITCH   -0.0570 .. -0.0186 turns   (13.8 deg)  <- entirely NEGATIVE
 *   KNEE        +0.0709 .. +0.0840 turns   ( 4.7 deg)
 *   ANKLE       +0.0167 .. +0.0595 turns   (15.4 deg)
 */
static const uint8_t s_gait_col[JOINT_COUNT] = { GAIT_COL_HIP_PITCH };

/*
 * Where the gait's zero sits in the drive's own coordinates, in TURNS.
 *
 * The trajectory is joint angles in the simulator's convention; the drive
 * measures from wherever it was homed. Those two zeros have no reason to agree,
 * and nothing in the data says how far apart they are.
 *
 * command = gait_turns + zero_offset
 *
 * To find it: put the leg in the pose the gait calls sample 0 - knee at
 * +26.2 degrees - and read `pos` off the console. Whatever it says goes here.
 * The arming log prints exactly this, so one run tells you the number.
 *
 * Leaving it at 0 is not wrong, it just means the gait plays around the drive's
 * homing position rather than around the intended joint angle. The MOTION is
 * identical either way; only where it happens changes.
 */
static const float s_zero_offset[JOINT_COUNT] = { 0.0f };

static const char *const s_joint_name[JOINT_COUNT] = { "hip_pitch" };

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
#define LEGTEST_TRACE_RX             0

/*
 * Never transmit. Receive only.
 *
 * CAN is acknowledged, so a transmitter whose frames do not reach the bus
 * drives its own error counter to 255 and takes itself BUS-OFF - which kills
 * RECEPTION too. That is why a working receive path can look completely dead
 * a second after boot.
 *
 * With this set, the controller only ever listens, never reaches bus-off, and
 * reception can be verified for as long as you like. It answers "is our
 * receive path fine and only transmit broken?" - which the counters cannot,
 * because bus-off destroys the evidence.
 */
#define LEGTEST_LISTEN_ONLY          0
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
#define LEGTEST_ENABLE_CLOSED_LOOP   1

/*
 * Delay before arming, in ticks (ms). A reset must never energise motors
 * instantly - this gives you time to see the countdown and pull power.
 */
#define LEGTEST_ARM_DELAY_MS         3000u

/*
 * STOP BUTTON - the blue USER button (B1) on the Nucleo.
 *
 * Press it at any time and the axis is commanded to IDLE and stays there.
 * There is no un-press: recovering means a reset, which is the correct
 * behaviour for a stop. Something that can be un-stopped by fumbling the same
 * button is not a stop button.
 *
 * This is NOT a substitute for an e-stop that cuts motor power. It travels
 * over the same CAN bus that might be the thing that failed. It is the
 * convenient stop for a bench test that is behaving oddly; the power switch is
 * the one for a bench test that is behaving dangerously.
 */
#define LEGTEST_STOP_BUTTON          1

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
 * How many gait cycles to play, then hold the final pose. 0 = loop forever.
 *
 * One cycle is the right first move: a complete, bounded piece of motion you
 * can watch start and finish, and if the direction or scaling is wrong it
 * stops on its own instead of repeating the mistake indefinitely.
 */
#define LEGTEST_GAIT_CYCLES          5u

/*
 * VELOCITY FEEDFORWARD
 *
 * Send the trajectory's own velocity alongside each position setpoint, in
 * bytes 4-5 of Set_Input_Pos.
 *
 * Without it the position loop has to MANUFACTURE the motion out of tracking
 * error: the only way the drive knows to move is by first falling behind. That
 * costs a lag proportional to speed over pos_gain, and on a joint with real
 * stiction it costs more than that, because the error has to grow large enough
 * to break the joint loose before anything happens at all.
 *
 * With it, the velocity the trajectory calls for is handed to the velocity
 * loop directly and the position loop only corrects the residual. This is free
 * accuracy on a trajectory that is known in advance - which this one entirely
 * is, since it is a table compiled into flash.
 *
 * Set to 0 to A/B it against the same run. Do that before tuning gains, not
 * after: feedforward changes what "good tracking" looks like, so gains tuned
 * without it will be too stiff once it is on.
 *
 * MEASURED MAGNITUDES, hip_pitch, straight off the spreadsheet:
 *
 *   velocity at 1.00x   -0.716 .. +0.675 turns/s
 *   velocity at 0.25x   -0.179 .. +0.169 turns/s   <- what this test sends
 *   as int16 thousandths     -179 .. +169          (field holds +/-32767)
 *
 * Two things follow. The wire quantum of 0.001 turns/s is 0.6% at the peak,
 * so resolution is a non-issue. And the trajectory is not slow: it asks for
 * 0.18 turns/s even at quarter speed, which is most of the motion the position
 * loop was previously being asked to invent out of tracking error alone.
 *
 * CHECK vel_limit BEFORE TRUSTING THIS. The drive clamps both the setpoint and
 * the feedforward to axis0.controller.config.vel_limit. Set below 0.18 and the
 * gait cannot be followed at quarter speed no matter what the gains are, and
 * the clamp is silent - it looks exactly like a tracking failure.
 */
#define LEGTEST_GAIT_VEL_FF          1

/*
 * TORQUE FEEDFORWARD - plumbed, but there is nothing to put in it.
 *
 * Bytes 6-7 of Set_Input_Pos take a torque feedforward, and it is the term
 * that would cancel gravity and limb inertia before they become tracking
 * error. It is also the term most likely to fix what this leg is doing.
 *
 * It cannot be filled from the current data. reference_gait_rleg_40ms_250hz.xlsx
 * contains five columns - time_s, hipP_r_deg, hipRoll_r_deg, knee_r_deg,
 * ankle_r_deg - and every one of them is a POSITION. There is no torque
 * anywhere in the spreadsheet, and a torque feedforward invented from a
 * position table would be a guess wearing the costume of a measurement.
 *
 * Two honest ways to get one, when it is wanted:
 *
 *   1. Re-export it. gait_ref.h records that the source was gait_library_v1.npz
 *      from Pinocchio + CasADi trajectory optimisation. An optimiser that
 *      solved for these positions necessarily computed the joint torques to
 *      achieve them - they exist in the npz and were simply not carried into
 *      the xlsx. Add a torque column, teach tools/gen_gait.py to emit it, and
 *      this becomes a table lookup like everything else.
 *
 *   2. Compute gravity compensation on the fly, from link mass and centre of
 *      mass against measured joint angle. Less complete than the optimiser's
 *      answer - it ignores inertia and contact - but it is the dominant term
 *      for a leg moving this slowly, and it needs no new data source.
 *
 * Until one of those exists this stays zero and the signature carries it, so
 * that adding it later is a value change and not a protocol change.
 */
#define LEGTEST_GAIT_TORQUE_FF       0

/*
 * TRAJECTORY CAPTURE
 *
 * Record commanded against measured for the whole run, then print it as CSV
 * when the gait finishes. The once-a-second report cannot show a 3.2 s
 * trajectory - by the time you read a line the interesting part is over.
 *
 * This is the thing to look at when the leg moves but not as far as it should:
 * the peak-to-peak summary says immediately whether the drive is tracking or
 * clipping, and the CSV shows WHERE in the cycle it gives up.
 *
 * 512 samples at 100 Hz covers 5.1 s, enough for one cycle at quarter speed
 * plus the entry ramp. 8 KB of RAM out of 448.
 */
#define LEGTEST_CAPTURE              1
#define CAPTURE_HZ                   100u
#define CAPTURE_MAX                  2048u

#if LEGTEST_CAPTURE
/* One row per sample: what we asked for, and what came back. */
typedef struct
{
    float cmd;
    float pos;
    float vel;
    float trq;
} cap_row_t;

static cap_row_t s_cap[CAPTURE_MAX];
static uint16_t  s_cap_n;
static uint8_t   s_cap_dumped;
#endif


/*
 * VELOCITY POKE - a deliberately dumb test for when position control does
 * nothing.
 *
 * Set to 1 and the drive is put in VELOCITY control and commanded a slow
 * constant spin. No trajectory, no position error, no gains involved beyond
 * the velocity loop.
 *
 *   it spins   -> the motor, encoder, current limit and calibration are all
 *                 fine, and the fault is in the POSITION control config
 *                 (pos_gain, vel_limit, input_mode)
 *   it does not -> the drive cannot produce torque at all, which is current
 *                 limit, calibration, or motor wiring - not our CAN at all
 *
 * This is the CAN equivalent of the IMU loopback: it removes everything the
 * failure could be blamed on, one layer at a time.
 */
#define LEGTEST_VEL_POKE             0
#define LEGTEST_VEL_POKE_TURNS_S     0.5f

/*
 * Classic CAN 2.0, not FD.
 *
 * MEASURED, not assumed: every frame this drive sends decodes as CLASSIC. So
 * whatever the 5 Mbit data rate in its config means, it is not transmitting FD
 * on the wire - and it will not acknowledge FD frames from us either.
 *
 * That one mismatch produced a failure that looked nothing like its cause:
 * reception was perfect (8731 frames, REC=0) while our own transmit error
 * counter climbed to 248 and took the controller BUS-OFF, which then killed
 * reception too. A receive path that works fine, appearing completely dead.
 *
 * Classic frames still run at the 1 Mbit nominal rate, which this bus is
 * already proven to carry. Set back to 1 only after confirming the drive
 * actually sends FD - the trace prints the format of every frame.
 */
#define LEGTEST_USE_CAN_FD           1

/*
 * Secondary sample point offset for CAN FD, in data time quanta of 12.5 ns.
 * Raise it if TEC climbs with FD enabled: 8 is mid-bit, 11-12 sits later and
 * tolerates a longer path.
 */
#define LEGTEST_TDC_OFFSET           8u

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
#define ODRV_CMD_SET_CTRL_MODE  0x00Bu
#define ODRV_CMD_SET_INPUT_POS  0x00Cu
#define ODRV_CMD_SET_INPUT_VEL  0x00Du

/* Set_Controller_Mode payload values. */
#define ODRV_CTRL_MODE_VELOCITY 2u
#define ODRV_CTRL_MODE_POSITION 3u
#define ODRV_INPUT_MODE_PASSTHR 1u
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
    uint8_t  fd;        /* FDCAN_FD_CAN or FDCAN_CLASSIC_CAN */
    uint8_t  brs;       /* bit rate switch used by the sender */
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

#if LEGTEST_LISTEN_ONLY
    (void)node; (void)cmd; (void)data; (void)len;
    return 1;              /* pretend it went out; the queue must still drain */
#else

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
#endif
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

/*
 * Pack a float into the int16 feedforward fields, which carry thousandths.
 *
 * Saturating rather than wrapping matters: an int16 that rolls over turns a
 * too-large forward velocity into a large REVERSE one, and hands the drive a
 * command to slam the joint backwards at full speed. Clipping merely asks for
 * less than the trajectory wanted. The trajectory never comes close to the
 * limit, which is exactly why the guard has to be here - it only ever fires
 * when something upstream has already gone wrong.
 */
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

    /* Both fields are int16 little-endian, 0.001 units: turns/s and Nm. */
    uint16_t v = (uint16_t)ff_thousandths(vel_ff);
    uint16_t q = (uint16_t)ff_thousandths(trq_ff);

    data[4] = (uint8_t)(v & 0xFFu);
    data[5] = (uint8_t)((v >> 8) & 0xFFu);
    data[6] = (uint8_t)(q & 0xFFu);
    data[7] = (uint8_t)((q >> 8) & 0xFFu);

    s_joint[j].cmd = pos;
    tx_enqueue(s_node_id[j], ODRV_CMD_SET_INPUT_POS, data, 8u);
}

/*
 * Tell the drive to actually act on positions.
 *
 * Set_Input_Pos only means something in POSITION control with PASSTHROUGH
 * input. In any other mode the drive accepts the frame, stores nothing useful,
 * and produces no torque - it sits armed and still while every counter looks
 * perfect, which is exactly what we saw: state=8, err=0, commands arriving,
 * pos unchanged and torque at 0.001 Nm.
 *
 * The mode lives in the drive's saved config, so relying on it means relying on
 * whatever odrivetool was last used to set. Sending it explicitly at arm time
 * removes the ambiguity.
 */
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
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;   /* torque_ff */

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
                s_trace[s_trace_head].fd  =
                    (hdr.FDFormat == FDCAN_FD_CAN) ? 1u : 0u;
                s_trace[s_trace_head].brs =
                    (hdr.BitRateSwitch == FDCAN_BRS_ON) ? 1u : 0u;
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

    /*
     * Check every one of these. A silently failed Start() looks identical from
     * the outside to a bus with nothing on it: no TX, no RX, and a software
     * queue filling up because the hardware FIFO never drains.
     */
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
    /*
     * Transmitter Delay Compensation. MUST be configured BEFORE Start() -
     * the peripheral only accepts it while still in initialisation mode.
     *
     * In the data phase a transmitter checks its own bits by reading them
     * back off the bus. At 5 Mbit a bit lasts 200 ns and the sample point is
     * at 150 ns, but the ISO1042 is isolated and its loop delay is 152 ns -
     * so when we look, our own bit has not returned yet. We read the
     * previous one, see a mismatch, and declare a bit error. Every FD frame
     * fails, TEC runs to 255, and the controller goes bus-off.
     *
     * Classic CAN has no data phase and never does this check, which is why
     * the bus was flawless at 1 Mbit.
     *
     * TDC moves the check to a Secondary Sample Point after the measured
     * delay. The offset is in time quanta of 12.5 ns; 8 puts the SSP at
     * 152 + 100 = 252 ns, halfway into the echoed bit. If TEC still climbs,
     * try 11 or 12 - the right value depends on the whole path, cable and
     * connectors included, not just the transceiver.
     */
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1,
                                            LEGTEST_TDC_OFFSET, 0u) != HAL_OK)
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

        /*
         * The format matters more than the bytes here. If the drive sends
         * CLASSIC frames, it is not in FD mode - and our FD frames with BRS
         * would be rejected by it, which is one of the two things that stops
         * a transmit path while leaving receive perfect.
         */
        printf("  id 0x%03lX  node %-2lu cmd 0x%02lX %-17s %s%s ",
               (unsigned long)t->id, (unsigned long)node,
               (unsigned long)cmd, cmd_name(cmd),
               t->fd ? "FD " : "CLASSIC",
               t->brs ? "+BRS" : "    ");

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
static void can_status(void);

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

    can_status();

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

#if LEGTEST_STOP_BUTTON
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);
    printf("STOP: press the blue USER button at any time to command IDLE.\r\n"
           "      It latches - reset the board to run again.\r\n"
           "      This is not an e-stop; it goes over the same CAN bus.\r\n\r\n");
#endif

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
    /*
     * These are a REQUIREMENT printed for the operator, not a readback - the
     * board has no way to query the drive's config over CAN. Do not read this
     * block as a report of what the ODrive is doing. What it is actually doing
     * shows up in the enc=/trq=/hb= counters on the per-second status line:
     * they are cumulative, so the increment between two lines is the rate in
     * Hz. 10 ms here is a ceiling, not a target - faster is better and 1 ms is
     * what the capture wants.
     */
    printf("\r\nSET THESE ON THE ODRIVE (this is a reminder, not a readback):\r\n");
    printf("  axis0.config.can.encoder_msg_rate_ms   <= 10   (1 is better)\r\n");
    printf("  axis0.config.can.torque_msg_rate_ms    <= 10   (1 is better)\r\n");
    printf("  axis0.config.can.heartbeat_msg_rate_ms  = 100\r\n");
    printf("  actual rates = the per-second growth of enc= trq= hb= below\r\n");
    printf("==============================================\r\n\r\n");
}

/* ------------------------------------------------------------------ */

/*
 * The CAN controller state, which is what actually explains a silent bus.
 *
 * CAN is a acknowledged protocol: EVERY transmitter needs at least one other
 * node to pull the ACK slot low. A lone node on the wire never gets that, so
 * it retries the same frame forever, its transmit error counter climbs, and at
 * 255 the controller takes itself BUS-OFF and stops entirely.
 *
 * From outside, bus-off looks exactly like a peripheral that was never started
 * - no TX, no RX, and a software queue filling because the hardware FIFO never
 * drains. The Last Error Code tells them apart, and Ack Error is the single
 * most useful value in this whole file: it means we DID transmit and nobody
 * answered.
 */
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
        /*
         * Recover, so a bus that comes up later is picked up instead of
         * needing a reset. Clearing INIT restarts the 128x11 recessive-bit
         * sequence the standard requires before rejoining.
         */
        printf("    -> BUS-OFF: nothing is acknowledging us. Recovering...\r\n");
        HAL_FDCAN_Start(&hfdcan1);
    }
}

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

    can_status();

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
                   "state=%u err=0x%08lX  hb=%lu enc=%lu trq=%lu  cmd=%+.4f err=%+.4f\r\n",
                   (unsigned)s_node_id[j], s_joint_name[j],
                   (double)v.pos, (double)v.vel, (double)v.torque,
                   (unsigned)v.axis_state, (unsigned long)v.axis_error,
                   (unsigned long)v.n_heartbeat, (unsigned long)v.n_encoder,
                   (unsigned long)v.n_torque, (double)v.cmd,
                   /* Command minus measured. A drive that is armed but not
                      acting on positions sits here at the full command while
                      torque stays near zero - which is not otherwise obvious. */
                   (double)(v.cmd - v.pos));
        }
    }

    /* Red LED blinks once per silent node; dark when all four answer. */
    if (silent == 0)
    {
        BSP_LED_Off(LED_RED);
    }

    printf("\r\n");
}

#if LEGTEST_CAPTURE
/*
 * Print the captured run as CSV, preceded by the number that usually
 * explains it.
 *
 * Peak-to-peak commanded against peak-to-peak measured is the whole tuning
 * signal. If the drive tracked, they match. If it moved a fraction of what
 * was asked, something is clipping - and the CSV shows where in the cycle it
 * stopped keeping up, which distinguishes a velocity limit (fails on the
 * fast sections) from a current limit (fails under load) from a low position
 * gain (lags everywhere, evenly).
 */
static void cap_dump(void)
{
    if (s_cap_dumped || (s_cap_n == 0u))
    {
        return;
    }
    s_cap_dumped = 1;

    float cmd_lo = 1e9f, cmd_hi = -1e9f;
    float pos_lo = 1e9f, pos_hi = -1e9f;
    float err_max = 0.0f, trq_max = 0.0f;

    for (uint16_t n = 0; n < s_cap_n; n++)
    {
        float c = s_cap[n].cmd, o = s_cap[n].pos;
        float e = (c > o) ? (c - o) : (o - c);
        float t = (s_cap[n].trq > 0.0f) ? s_cap[n].trq : -s_cap[n].trq;

        if (c < cmd_lo) { cmd_lo = c; }
        if (c > cmd_hi) { cmd_hi = c; }
        if (o < pos_lo) { pos_lo = o; }
        if (o > pos_hi) { pos_hi = o; }
        if (e > err_max) { err_max = e; }
        if (t > trq_max) { trq_max = t; }
    }

    float cmd_pp = cmd_hi - cmd_lo;
    float pos_pp = pos_hi - pos_lo;

    printf("\r\n===== TRAJECTORY, %u samples at %u Hz =====\r\n",
           (unsigned)s_cap_n, (unsigned)CAPTURE_HZ);
    printf("commanded travel : %+.4f .. %+.4f turns = %.2f deg\r\n",
           (double)cmd_lo, (double)cmd_hi, (double)(cmd_pp * 360.0f));
    printf("achieved  travel : %+.4f .. %+.4f turns = %.2f deg\r\n",
           (double)pos_lo, (double)pos_hi, (double)(pos_pp * 360.0f));
    printf("tracking         : %.0f%% of commanded motion\r\n",
           (double)((cmd_pp > 1e-6f) ? (pos_pp / cmd_pp * 100.0f) : 0.0f));
    printf("worst error      : %.4f turns (%.2f deg)\r\n",
           (double)err_max, (double)(err_max * 360.0f));
    printf("peak torque      : %.3f Nm\r\n\r\n", (double)trq_max);

    printf("t_s,cmd_turns,pos_turns,vel_tps,trq_Nm\r\n");
    for (uint16_t n = 0; n < s_cap_n; n++)
    {
        printf("%.3f,%.5f,%.5f,%.4f,%.4f\r\n",
               (double)n / (double)CAPTURE_HZ,
               (double)s_cap[n].cmd, (double)s_cap[n].pos,
               (double)s_cap[n].vel, (double)s_cap[n].trq);
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

#if LEGTEST_STOP_BUTTON
        /* ACTIVE LOW. BSP_PB_Init configures the pin with GPIO_PULLUP, so it
           reads 1 released and 0 pressed. Testing for non-zero latched the
           stop at boot on every run - transmitting nothing, never arming, and
           looking exactly like a dead command path. */
        if (!s_stopped && (BSP_PB_GetState(BUTTON_USER) == 0))
        {
            s_stopped = 1;

            /* Drop everything already queued first. Those are position
               setpoints that would otherwise still go out AFTER the idle
               command and could re-energise nothing but confusion. */
            s_txq_tail = s_txq_head;

            for (int j = 0; j < JOINT_COUNT; j++)
            {
                send_axis_state(j, ODRV_AXIS_STATE_IDLE);
            }

            printf("\r\n*** STOPPED by user button - axes commanded"
                   " to IDLE.\r\n    Reset the board to run again. ***\r\n\r\n");
        }
#endif

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

        /*
         * Feedforward that goes out with the position, per joint.
         *
         * These must be the derivative of what target[] is actually DOING,
         * not of the trajectory in the abstract. Every branch below that sets
         * target[] therefore sets these too, and the zero initialiser is the
         * right answer for every branch that holds station - a stationary
         * setpoint has zero velocity, whatever the table says at that phase.
         */
        float target_vel[JOINT_COUNT] = { 0.0f };
        float target_trq[JOINT_COUNT] = { 0.0f };

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
                first[j] = all[s_gait_col[j]] + s_zero_offset[j];
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

                if (s_entry_ok)
                {
                    /*
                     * Print the ramp explicitly. It is a move-to-start, not
                     * part of the trajectory, and a large one means the
                     * drive s zero and the gait s zero disagree - which is
                     * the number s_zero_offset exists to absorb.
                     */
                    float g0[GAIT_JOINTS];
                    gait_sample(0.0f, g0);

                    for (int k = 0; k < JOINT_COUNT; k++)
                    {
                        float want = g0[s_gait_col[k]] + s_zero_offset[k];

                        printf("  %s: at %+.4f, gait starts at %+.4f"
                               " -> ramp %+.4f turns (%+.1f deg)\r\n",
                               s_joint_name[k], (double)s_entry_from[k],
                               (double)want, (double)(want - s_entry_from[k]),
                               (double)((want - s_entry_from[k]) * 360.0f));

                        printf("     to play the gait around where the leg is"
                               " now, set s_zero_offset[%d] = %+.4f\r\n",
                               k, (double)(s_entry_from[k]
                                           - g0[s_gait_col[k]]));

                        /* Sweep the whole cycle to find the travel. Knowing
                           how far this joint will move BEFORE it moves is
                           worth four hundred table lookups. */
                        float lo = 1e9f, hi = -1e9f;

                        for (int s = 0; s < GAIT_SAMPLES; s++)
                        {
                            float v = g_gait_turns[s][s_gait_col[k]];
                            if (v < lo) { lo = v; }
                            if (v > hi) { hi = v; }
                        }

                        printf("     travel %+.4f .. %+.4f turns"
                               " = %.1f deg of output\r\n",
                               (double)(lo + s_zero_offset[k]),
                               (double)(hi + s_zero_offset[k]),
                               (double)((hi - lo) * 360.0f));
                    }
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

                /* A straight line has one constant velocity: the whole
                   distance over the whole ramp. The entry deserves the same
                   feedforward as the gait - it is the move most likely to be
                   large, and the one where the drive is coldest. */
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
                /* Free-running phase clock. gait_sample wraps it, so this can
                   count up forever without special-casing the seam. */
                float t = (float)(since_arm - LEGTEST_GAIT_ENTRY_MS) * 0.001f;

                s_gait_running = 1;      /* entry ramp is over; capture now */
                s_gait_phase = (t * LEGTEST_GAIT_SPEED) / GAIT_CYCLE_S;

#if (LEGTEST_GAIT_CYCLES > 0u)
                /*
                 * Stop after the requested cycles and hold. Freezing the PHASE
                 * rather than the output means the hold pose is a real point on
                 * the trajectory, so resuming later would not step.
                 */
                if (s_gait_phase >= (float)LEGTEST_GAIT_CYCLES)
                {
                    s_gait_phase = (float)LEGTEST_GAIT_CYCLES;

                    if (!s_gait_done)
                    {
                        s_gait_done = 1;
                        printf("\r\ngait: %u cycle(s) complete -"
                               " holding final pose\r\n",
                               (unsigned)LEGTEST_GAIT_CYCLES);

#if LEGTEST_CAPTURE
                        cap_dump();
#endif
                    }
                }
#endif

                float all_vel[GAIT_JOINTS];

                gait_sample_vel(s_gait_phase, all, all_vel);

                /*
                 * Two scalings, and both are mandatory.
                 *
                 * LEGTEST_GAIT_SPEED, because the phase clock is turning at
                 * that fraction of nominal and the velocity has to agree with
                 * the position it accompanies. Feeding unscaled velocity at
                 * quarter speed would ask for four times the motion the
                 * setpoint is making, and the drive would run away from a
                 * setpoint it is simultaneously being told to track.
                 *
                 * Zero once the cycles are done, because holding the final
                 * pose freezes the PHASE, not the table. The trajectory still
                 * has a velocity at that phase; the command no longer does.
                 * Sending the table's value there would drive the joint off
                 * a stationary setpoint for as long as the test is left
                 * running - a slow push with nothing to stop it.
                 */
                float vscale = s_gait_done ? 0.0f : LEGTEST_GAIT_SPEED;

                for (int j = 0; j < JOINT_COUNT; j++)
                {
                    target[j]     = all[s_gait_col[j]] + s_zero_offset[j];
                    target_vel[j] = all_vel[s_gait_col[j]] * vscale;
                }
            }
#else
            float t     = (float)since_arm * 0.001f;
            float w     = 2.0f * 3.14159265f * LEGTEST_FREQ_HZ;
            float phase = w * t;
            float v     = LEGTEST_AMPLITUDE_TURNS * sinf(phase);
            float dv    = LEGTEST_AMPLITUDE_TURNS * w * cosf(phase);

            for (int j = 0; j < JOINT_COUNT; j++)
            {
                target[j]     = v;
                target_vel[j] = dv;
            }
#endif
        }
#endif

        /* Once stopped, send nothing further. Commanding IDLE and then
           continuing to stream positions would leave the drive one stray
           re-arm away from moving again. */
        if (!s_stopped && ((s_tick % LEGTEST_TX_DIV) == 0u))
        {
#if (LEGTEST_TX_DIV == 1u)
            /* Full rate: command every joint on every tick. */
            for (int j = 0; j < JOINT_COUNT; j++)
            {
#if LEGTEST_VEL_POKE
                send_input_vel(j, (s_tick > LEGTEST_ARM_DELAY_MS)
                                      ? LEGTEST_VEL_POKE_TURNS_S : 0.0f);
#else
                /* The toggles fold at compile time; keeping them here rather
                   than in the branches above means every regime gets the same
                   treatment and none can be forgotten when one is added. */
                send_input_pos(j, target[j],
                               LEGTEST_GAIT_VEL_FF    ? target_vel[j] : 0.0f,
                               LEGTEST_GAIT_TORQUE_FF ? target_trq[j] : 0.0f);
#endif
            }
#else
            /* Reduced rate: one joint per tick, round-robin. */
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
        /* Sample joint 0 at CAPTURE_HZ. Only while the axis is live - before
           arming there is nothing to compare. */
        /*
         * Only while the GAIT is playing. Capturing the entry ramp too made
         * the summary meaningless: it compared the measured travel of
         * ramp+gait against the commanded range of the gait alone, and
         * reported "249% tracking" for a leg that was following properly.
         */
        if ((s_cap_n < CAPTURE_MAX) && s_gait_running &&
            (s_joint[0].axis_state == ODRV_AXIS_STATE_CLOSED_LOOP) &&
            ((s_tick % (1000u / CAPTURE_HZ)) == 0u))
        {
            s_cap[s_cap_n].cmd = s_joint[0].cmd;
            s_cap[s_cap_n].pos = s_joint[0].pos;
            s_cap[s_cap_n].vel = s_joint[0].vel;
            s_cap[s_cap_n].trq = s_joint[0].torque;
            s_cap_n++;
        }
#endif


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
        if (!s_stopped && s_scan_ok && (s_tick > LEGTEST_ARM_DELAY_MS) &&
            ((s_tick % 2000u) == 500u))
        {
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                if ((s_joint[j].axis_state != ODRV_AXIS_STATE_CLOSED_LOOP) &&
                    (s_joint[j].axis_error == 0u) &&
                    (s_joint[j].n_heartbeat > 0u))
                {
                    printf("arming node %u (%s): position control,"
                           " passthrough, then closed loop\r\n",
                           (unsigned)s_node_id[j], s_joint_name[j]);

                    /*
                     * Mode BEFORE state. Set_Input_Pos only means anything
                     * in POSITION control with PASSTHROUGH input; in any
                     * other mode the drive accepts every frame, produces no
                     * torque, and sits armed and still while all the
                     * counters look perfect. Arming first would energise it
                     * into whatever odrivetool last saved.
                     */
                    send_controller_mode(j);
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
