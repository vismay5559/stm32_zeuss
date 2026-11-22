#include "app.h"
#include "gait_ref.h"
#include "main.h"
#include "link_proto.h"
#include "link_usb.h"
#include "imu_bno085.h"
#include "enc_as5047p.h"
#include "act_odrive.h"
#include "contact.h"
#include "critical.h"
#include "health.h"
#include "safety.h"
#include "robot_config.h"
#include "watchdog.h"
#include "nexus_mode.h"
#include "fusion.h"
#include "soft_start.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

/*
 * Periodic loop-timing report on the serial console. OFF by default.
 *
 * At 115200 8N1 a character is 86.8 us, so the line used to block for ~9.5 ms
 * and miss nine ticks every two seconds. A diagnostic that causes nine of the
 * missed ticks it reports is not a diagnostic, and it set the floor for the
 * watchdog period besides.
 *
 * Every counter it printed now ships in the state packet instead, at 1 kHz,
 * where the Pi can plot it against everything else that happened at the same
 * moment. What is left here is the handful with no room in the packet - CAN
 * error counters, the not-in-closed-loop mask, encoder and IMU recovery counts
 * - which makes the line LONGER, about 200 characters and ~17 ms.
 *
 * That is fine, because it is now opt-in and meant for a bench with no Pi
 * attached, and 17 ms still sits well inside wdg_timeout_ms(). It is not fine
 * to turn on while the robot is walking. Check that budget again if the line
 * grows further.
 *
 * Turn this back on only when there is no Pi attached:
 *
 *     NEXUS_LOOP_STATS=1 cmake --preset Debug
 */
/*
 * The robot loop is not allowed to be built unoptimised.
 *
 * Measured 2026-11-22 on the robot, ROBOT mode with only the IMU live:
 *
 *     -O0 (Debug)     loop worst 4.1-5.4 ms   estimator 2.4-3.1 ms   ~1000 overruns/s
 *     -Os (Release)   loop worst 0.61-0.67 ms estimator 0.41 ms      0
 *
 * At -O0 the board ticked at 303 Hz instead of 1000 and the link test failed
 * on a perfect cable - an evening spent suspecting USB, when the firmware had
 * simply been built Debug, which is what Appli/build is configured as. An
 * unoptimised build is not a slower robot, it is a different one: every
 * timing figure the estimator and the safety logic assume is wrong.
 *
 * So refuse to compile it. Build flight firmware with --preset Release (see
 * README, "Flight firmware"). To single-step the robot loop in a debugger
 * anyway, knowing it will overrun, define NEXUS_ALLOW_UNOPTIMISED_ROBOT.
 */
#if (NEXUS_MODE == NEXUS_MODE_ROBOT) && !defined(__OPTIMIZE__) && \
    !defined(NEXUS_ALLOW_UNOPTIMISED_ROBOT)
#error "ROBOT mode built without optimisation: the 1 kHz loop overruns ~4x at -O0. Use --preset Release (or -DCMAKE_BUILD_TYPE=Release). Define NEXUS_ALLOW_UNOPTIMISED_ROBOT only to debug."
#endif

#ifndef NEXUS_LOOP_STATS
#define NEXUS_LOOP_STATS  0
#endif

static volatile uint32_t s_tick_pending;
static uint32_t          s_seq;
static uint32_t          s_overruns;
static uint32_t          s_loop_us_max;
static uint32_t          s_startup_grace;
#if NEXUS_LOOP_STATS
static uint32_t          s_console_us_max;
#endif

/* Ticks to ignore at startup before overruns count as real faults. */
#define STARTUP_GRACE_TICKS  50u

static nexus_state_t     s_state;

/* Last spring deflection that passed every check, per encoder, and how many
   reads were refused for lying outside the spring's travel. */
static float    s_spring_last[NEXUS_NUM_ENCODERS];
static uint32_t s_spring_out_of_range;

uint32_t app_overruns(void)
{
    return s_overruns;
}


/*
 * Every DMA buffer in this project lives in the "noncacheable_buffer" linker
 * section, but a section name alone changes nothing - what actually makes it
 * non-cacheable is MPU Region 2, configured in the *Boot* project
 * (CORTEX_M7_BOOT in CubeMX). If those two ever disagree, the CPU reads stale
 * cached copies while DMA writes real RAM, and sensor data goes intermittently
 * corrupt in a way that looks exactly like bad wiring.
 *
 * CubeMX will happily regenerate Region 2 back to its default, so verify the
 * agreement at boot rather than trusting it. A hang here means Region 2 no
 * longer covers the buffers - go fix it in CubeMX, not here.
 */
#define NONCACHEABLE_MPU_BASE  0x24070000u
#define NONCACHEABLE_MPU_SIZE  0x2000u

extern uint32_t __NONCACHEABLEBUFFER_BEGIN;
extern uint32_t __NONCACHEABLEBUFFER_END;

static void check_noncacheable_region(void)
{
    uint32_t begin = (uint32_t)&__NONCACHEABLEBUFFER_BEGIN;
    uint32_t end   = (uint32_t)&__NONCACHEABLEBUFFER_END;

    if ((begin < NONCACHEABLE_MPU_BASE) ||
        (end > (NONCACHEABLE_MPU_BASE + NONCACHEABLE_MPU_SIZE)))
    {
        /* main() has not reached its BSP_LED_Init calls yet, and Error_Handler
           spins forever with interrupts off - so light the LED here or this
           failure is completely silent. */
        BSP_LED_Init(LED_RED);
        BSP_LED_On(LED_RED);
        Error_Handler();
    }
}

/*
 * Health LEDs - the only outward sign the firmware is alive, since everything
 * else goes out over USB to the Pi.
 *
 *   LD1 green : 1 Hz heartbeat, driven from the 1 kHz tick. A steady, even
 *               blink is direct proof the control loop is running at the rate
 *               it is supposed to. If it stops, stutters, or visibly changes
 *               rate, the loop is in trouble - watch this during bring-up.
 *   LD3 red   : latched fault. Comes on and stays on the first time a tick is
 *               missed or a CAN frame is dropped, so a fault that happened
 *               seconds ago is still visible when you look up.
 */
#define HEARTBEAT_TICKS  500u   /* toggle every 500 ms -> 1 Hz full cycle */

/*
 * Red LED blink code. Rather than "something is wrong", it blinks the number
 * of the first faulted subsystem, pauses, and repeats - so one LED diagnoses
 * six things from across the room:
 *
 *   1 = IMU stale    2 = encoders invalid   3 = CAN bus 1 silent
 *   4 = CAN2 silent  5 = Pi link stale      6 = loop overrun
 *
 * Timing: 150 ms on, 150 ms off per blink, then a 1 s gap so the count is
 * easy to read. Driven from the 1 kHz tick, so these are just tick counts.
 */
#define BLINK_ON_TICKS    150u
#define BLINK_OFF_TICKS   150u
#define BLINK_GAP_TICKS  1000u

static void update_health_leds(void)
{
    static uint32_t beat;
    static uint32_t phase;      /* ticks into the current blink sequence */
    static uint8_t  shown_code; /* code being blinked out right now      */

    /* Green heartbeat: independent of everything else, always runs. */
    if (++beat >= HEARTBEAT_TICKS)
    {
        beat = 0;
        BSP_LED_Toggle(LED_GREEN);
    }

    uint8_t code = health_blink_code();

    if (code == 0u)
    {
        BSP_LED_Off(LED_RED);
        phase      = 0;
        shown_code = 0;
        return;
    }

    /* Latch onto a code for one full sequence so the count stays readable
       even if a different fault appears midway through. */
    if (shown_code == 0u)
    {
        shown_code = code;
        phase      = 0;
    }

    uint32_t blink_len = BLINK_ON_TICKS + BLINK_OFF_TICKS;
    uint32_t seq_len   = ((uint32_t)shown_code * blink_len) + BLINK_GAP_TICKS;
    uint32_t pos       = phase % seq_len;

    if (pos < ((uint32_t)shown_code * blink_len))
    {
        if ((pos % blink_len) < BLINK_ON_TICKS)
        {
            BSP_LED_On(LED_RED);
        }
        else
        {
            BSP_LED_Off(LED_RED);
        }
    }
    else
    {
        BSP_LED_Off(LED_RED);   /* the gap between repetitions */
    }

    if (++phase >= seq_len)
    {
        phase      = 0;
        shown_code = 0;         /* re-evaluate which fault to show */
    }
}

/* ===================================================================== */
/*  The on-board reference walk                                          */
/* ===================================================================== */

/*
 * Build all ten joint references for one instant of the stride.
 *
 * gait_ref.c holds ONE leg - four columns, generated for the right leg from
 * the trajectory optimisation. The other leg is that same trajectory half a
 * cycle later, which is what a symmetric walk is.
 *
 * Two things about the mirror are worth stating rather than leaving in the
 * arithmetic. The pitch joints (hip_pitch, knee, ankle) swing in the sagittal
 * plane and mirror onto the left leg unchanged. hip_roll does not: it moves
 * out of that plane, so the mirrored leg needs the opposite sign. Getting that
 * sign wrong gives a robot that leans both hips the same way and falls over
 * sideways, which looks like a balance problem rather than a table problem.
 *
 * Joint indices are the NEXUS_J_* map in link_proto.h:
 *
 *     bus 0: 0 L hip_pitch  1 L hip_roll  2 L knee  3 L ankle
 *     bus 1: 4 R hip_pitch  5 R hip_roll  6 R knee  7 R ankle
 *
 * Each row below says, by name, which table column drives which joint on each
 * side. Table columns are NOT in node order (see gait_ref.h), so this is the
 * only place that pairing is made.
 *
 * Every joint in this build is a leg joint, so every index gets a reference.
 * (The waist used to sit at indices 4 and 9 with no column of its own and was
 * held at zero; this build has no waist actuators at all.)
 */
/*
 * Stride playback rate. 1.0 plays the trajectory at the speed it was optimised
 * for; GAIT_CYCLE_S is that period. Lower is slower and safer to watch.
 */
#define APP_GAIT_SPEED   1.0f

static float s_phase;
static float s_ref_turns[NEXUS_NUM_JOINTS];

/*
 * The reference the DRIVES are given, which is not the one in the packet.
 *
 * s_ref_turns is the gait, and that is what goes out as ref_angle - the policy
 * reads it as part of its observation and rl_policy_node refuses to enable if
 * it differs from its own copy by more than a degree, so it has to stay the
 * pure gait.
 *
 * s_drive_ref_turns is the same thing with the series springs' pre-deflection
 * taken off, and is what safety.c bounds and the drives are commanded to. The
 * two differ by a fraction of a degree, which is small enough to be invisible
 * and exactly the sort of difference that is worth two variables rather than
 * one.
 */
static float s_drive_ref_turns[NEXUS_NUM_JOINTS];
static float s_gait_vel_turns[NEXUS_NUM_JOINTS];
static float s_gait_tau_nm[NEXUS_NUM_JOINTS];

/*
 * Soft start - see soft_start.h for why arming can no longer go straight to
 * the gait.
 *
 * s_cmd_ref_turns is what an accepted command is built on: the ramp, the hold
 * pose, or the gait, as soft_start_tick() decides. s_hold_pose_turns is the
 * reference at SOFT_START_PHASE, without spring pre-deflection; it is a
 * property of the compiled gait table, computed once.
 *
 * s_meas_turns is the pose the drives report, and s_pose_ok whether every
 * joint has a finite, recent one. An arm has to ramp FROM somewhere: with a
 * joint missing there is no honest starting point, so the arm waits.
 */
static float   s_cmd_ref_turns[NEXUS_NUM_JOINTS];
static float   s_hold_pose_turns[NEXUS_NUM_JOINTS];
static float   s_meas_turns[NEXUS_NUM_JOINTS];
static uint8_t s_pose_ok;
static uint8_t s_zero_prev;          /* ZERO_ANKLES on the previous command   */
static uint8_t s_ankles_zeroed;      /* ZERO_ANKLES carried out since boot    */
static uint16_t s_arm_blocked;       /* ticks left to report ARM_BLOCKED      */

/* A joint whose last encoder frame is older than this has no pose to ramp
   from. The drives send at 1 kHz; 50 ms is fifty missed frames. */
#define POSE_STALE_TICKS     50u

/* How long ARM_BLOCKED stays visible in the packet after a refused arm, so a
   Pi sampling at 250 Hz or a person reading a plot cannot miss it. */
#define ARM_BLOCKED_TICKS   200u
static uint8_t s_gains_seq;   /* echo of the last gains message applied */

static void build_reference(float phase,
                            float ref_turns[NEXUS_NUM_JOINTS],
                            float vel_turns_s[NEXUS_NUM_JOINTS],
                            float tau_nm[NEXUS_NUM_JOINTS])
{
    /* The right leg plays the table as generated; the left mirrors roll only. */
    static const struct
    {
        uint8_t left;
        uint8_t right;
        uint8_t col;
        float   left_sign;
    } map[4] = {
        { NEXUS_J_L_HIP_PITCH,   NEXUS_J_R_HIP_PITCH,   GAIT_COL_HIP_PITCH,  1.0f },
        { NEXUS_J_L_HIP_ROLL,    NEXUS_J_R_HIP_ROLL,    GAIT_COL_HIP_ROLL,  -1.0f },
        { NEXUS_J_L_KNEE_PITCH,  NEXUS_J_R_KNEE_PITCH,  GAIT_COL_KNEE,       1.0f },
        { NEXUS_J_L_ANKLE_PITCH, NEXUS_J_R_ANKLE_PITCH, GAIT_COL_ANKLE,      1.0f },
    };

    enum { MAP_CLASSES = (int)(sizeof(map) / sizeof(map[0])) };

    /*
     * The loop below zeroes every joint first and then fills only the classes
     * in `map`, so a joint the map forgets is not left out of the packet - it
     * is published as a confident ref_angle of 0.0 and driven there, which
     * looks exactly like a joint the gait genuinely holds still. The asserts
     * tie the map to the drive count and to the table's own width rather than
     * to a literal.
     */
    _Static_assert(MAP_CLASSES == ROBOT_LEG_MOTORS,
                   "every driven joint needs a gait column: a joint missing "
                   "from map[] is silently commanded to the gait's zero");
    _Static_assert(GAIT_JOINTS == ROBOT_LEG_MOTORS,
                   "the gait table and the drive count disagree");

    float right[GAIT_JOINTS],  left[GAIT_JOINTS];
    float rvel[GAIT_JOINTS],   lvel[GAIT_JOINTS];
    float rtau[GAIT_JOINTS],   ltau[GAIT_JOINTS];

    gait_sample_all(phase,        right, rvel, rtau);
    gait_sample_all(phase + 0.5f, left,  lvel, ltau);   /* half a stride behind */

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        ref_turns[j]   = 0.0f;
        vel_turns_s[j] = 0.0f;
        tau_nm[j]      = 0.0f;
    }

    for (int k = 0; k < MAP_CLASSES; k++)
    {
        uint8_t L = map[k].left, R = map[k].right;
        uint8_t c = map[k].col;
        float   sg = map[k].left_sign;

        /*
         * The sign flip is the LEG's, not the quantity's, so velocity and
         * torque take it as well as position. A left roll whose position
         * mirrors while its torque does not is a joint being pushed one way
         * and commanded the other. The gait library is asserted to be
         * half-stride mirror symmetric under exactly this operation - see
         * tools/gen_gait_from_library.py.
         */
        ref_turns[L]   = left[c] * sg;
        vel_turns_s[L] = lvel[c] * sg;
        tau_nm[L]      = ltau[c] * sg;

        ref_turns[R]   = right[c];
        vel_turns_s[R] = rvel[c];
        tau_nm[R]      = rtau[c];
    }

    /*
     * The gait's velocity is quoted at nominal playback - one stride per
     * GAIT_CYCLE_S. Playing the clock at any other rate has to scale it by the
     * same factor, or the drive is told to travel at a speed the position
     * setpoint is not going to reach.
     */
    if (APP_GAIT_SPEED != 1.0f)
    {
        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            vel_turns_s[j] *= APP_GAIT_SPEED;
        }
    }
}

/*
 * Take the series springs' wind-up off the targets the drives are given.
 *
 * The ODrive holds the MOTOR side of a series-elastic joint and the limb sits
 * behind the spring, so under torque tau the spring winds up by tau/K and the
 * limb ends up SHORT of where the motor was told to be:
 *
 *     limb = motor + tau / K        =>        motor = ref - tau / K
 *
 * Without this the limb trails the reference by the wind-up for the whole of
 * stance, every stride, in the same direction - a systematic bias, which the
 * policy cannot learn around because its simulator did not have one. zeus_env.py
 * applies the same correction, which is why the board must.
 *
 * Clamped to the spring's mechanical travel, because beyond that the spring is
 * on its stop and a larger correction describes a deflection that cannot
 * exist. A wrong torque number therefore costs at most 1.7 deg of target.
 */
static void apply_spring_predeflection(const float ref_turns[NEXUS_NUM_JOINTS],
                                       const float tau_nm[NEXUS_NUM_JOINTS],
                                       float out_turns[NEXUS_NUM_JOINTS])
{
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        out_turns[j] = ref_turns[j];

        if (!g_joint_is_sea[j])
        {
            continue;
        }

        float d = tau_nm[j] / ROBOT_SPRING_K_NM_PER_RAD;     /* rad at output */

        if (!isfinite(d))
        {
            continue;
        }
        if (d >  ROBOT_SPRING_MAX_DEFLECTION_RAD) { d =  ROBOT_SPRING_MAX_DEFLECTION_RAD; }
        if (d < -ROBOT_SPRING_MAX_DEFLECTION_RAD) { d = -ROBOT_SPRING_MAX_DEFLECTION_RAD; }

        out_turns[j] -= d / 6.28318530718f;                  /* rad -> turns  */
    }
}

void app_init(void)
{
    /* Clear the marker Boot left on. Doing this first - before anything that
       could hang - means yellow staying lit points the finger squarely at the
       Boot-to-Appli handover rather than at anything in here.
       main() has not run its BSP_LED_Init calls yet, hence the init here. */
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Off(LED_YELLOW);

    printf("APPLI: mode %s, protocol v%u\r\n",
           NEXUS_MODE_NAME, (unsigned)NEXUS_PROTO_VERSION);

    /* Read RCC_RSR before anything else can clear it - a board that has been
       quietly rebooting itself mid-run must say so on the very next line. */
    wdg_init_reset_cause();
    if (wdg_reset_was_watchdog())
    {
        printf("APPLI: *** last reset came from the WATCHDOG ***\r\n");
    }

    check_noncacheable_region();

    memset(&s_state, 0, sizeof(s_state));
    s_tick_pending = 0;
    s_seq          = 0;
    s_overruns      = 0;
    s_loop_us_max   = 0;
    s_startup_grace = 0;

    /*
     * Watch everything HEALTH_EXPECTED_MASK names - by default the whole
     * robot. safety.c refuses to arm until all of it is healthy, so narrowing
     * this during bring-up is a build-time decision (see health.h) rather
     * than something to edit in and forget to edit out.
     */
    health_init(HEALTH_EXPECTED_MASK);
    safety_init();
    fusion_init();

    soft_start_idle();
    {
        float v[NEXUS_NUM_JOINTS], t[NEXUS_NUM_JOINTS];
        build_reference(SOFT_START_PHASE, s_hold_pose_turns, v, t);
    }
    s_pose_ok       = 0;
    s_zero_prev     = 0;
    s_ankles_zeroed = 0;
    s_arm_blocked   = 0;

    contact_init();
    enc_init();
    act_init();
    link_usb_init();
    /*
     * The UNCALIBRATED gyro. The calibrated report has the BNO085's own,
     * continually revised bias taken out, which is a second estimator of the
     * one quantity the InEKF's bias state exists to estimate: each time the
     * sensor revises its bias the rate steps, and the filter sees a rotation
     * nobody made. Measured still, the raw gyro's bias barely moves within an
     * hour (docs/IMU_NOISE.md), so the filter has an easy job on its own.
     */
    imu_use_uncal_gyro(1u);
    /*
     * And no rotation vector. The estimator derives its own orientation; the
     * sensor's is only a comparison channel (imu_quat in the packet), and
     * producing it starves the accelerometer. Measured 2026-11-22 in ROBOT
     * mode with it on: gyro 371/s, accelerometer 127/s - and the filter only
     * predicts when BOTH are new, so it was running at ~127 Hz, not 400.
     * imu_quat stays at identity with this off.
     */
    imu_skip_quaternion(1u);
    imu_init();

    /*
     * Start the watchdog last, and only now.
     *
     * Everything above blocks for a long time - imu_init() alone spends the
     * better part of a second on the sensor's reset sequence - and none of it
     * can refresh a watchdog. Arming it here means the first thing it ever
     * supervises is the control loop, which is the only thing it should be
     * supervising.
     */
    if (wdg_start())
    {
        printf("APPLI: watchdog armed, %lu ms\r\n",
               (unsigned long)wdg_timeout_ms());
    }
    else
    {
        /* No watchdog means no recovery from a hang. Say so loudly rather
           than running on and looking healthy. */
        printf("APPLI: *** WATCHDOG FAILED TO START - LSI not running ***\r\n");

        /* main() does not run its BSP_LED_Init calls until after app_init(),
           so init here or this warning has no visible half. */
        BSP_LED_Init(LED_RED);
        BSP_LED_On(LED_RED);
    }

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);
}

void app_on_tick(void)
{
    s_tick_pending++;
}

/* ODrive reports turns; the policy block is radians on the output side. */
#define TURNS_TO_RAD   6.28318531f

/*
 * Hands back the IMU sample and encoder validity it fetched, so health_tick()
 * can judge the same readings that went into the packet rather than fetching
 * a second, slightly different set a few microseconds later.
 */
static void build_and_send_state(imu_sample_t *imu_out, uint8_t *enc_valid_out)
{
    imu_sample_t    imu;
    act_telemetry_t act;

    imu_get(&imu);
    act_get(&act);

    s_state.seq          = s_seq++;
    s_state.timestamp_us = __HAL_TIM_GET_COUNTER(&htim2);

    /* ---- raw IMU, passed through for logging and cross-checks -------- */
    memcpy(s_state.imu_quat,  imu.quat,  sizeof(s_state.imu_quat));
    memcpy(s_state.imu_accel, imu.accel, sizeof(s_state.imu_accel));
    memcpy(s_state.imu_gyro,  imu.gyro,  sizeof(s_state.imu_gyro));
    s_state.imu_seq = imu.seq;

    /* Policy block wants body-frame angular velocity, which is exactly what
       the gyro reports - no rotation, no bias removal. The estimator's bias
       estimate is sent separately if the Pi ever wants to apply it. */
    memcpy(s_state.gyro, imu.gyro, sizeof(s_state.gyro));

    /* ---- after-spring encoders -------------------------------------- */
    uint16_t enc_raw[NEXUS_NUM_ENCODERS];
    float    spring_rad[NEXUS_NUM_ENCODERS];   /* aligned copy for fusion.c */
    uint8_t  enc_valid;

    enc_get(enc_raw, &enc_valid);
    for (int e = 0; e < NEXUS_NUM_ENCODERS; e++)
    {
        /*
         * These measure SPRING DEFLECTION, not an absolute joint angle - the
         * encoder sits after the series spring, so it reads how far the spring
         * has wound up. Torque is deflection times the spring constant, done
         * on the Pi where the constant can be tuned without reflashing.
         *
         * Which is why the raw count is referenced to a zero and wrapped into
         * +/-pi rather than sent as a bare 0..2pi angle: deflection is a small
         * SIGNED quantity either side of rest, and a joint whose rest position
         * sits near the wrap point would otherwise jump a full turn between
         * two adjacent ticks - straight into the Pi's torque estimate.
         *
         * The zeros live in robot_config.h (measured 2026-11-22).
         *
         * And a deflection the spring cannot physically reach is a bad read,
         * not a deflection. The bus delivers occasional replies with good
         * parity and a wrong angle - 0, or the true value shifted a bit - in
         * BURSTS, so the driver's repeat test alone lets some through. Outside
         * twice the spring's travel around its zero (3.4 degrees; the
         * glitches land tens of degrees away), the read is marked invalid
         * this tick and the last good deflection is sent instead - so the
         * policy, which reads spring_angle every step, never sees one.
         */
        float d = robot_spring_deflection((uint8_t)e, enc_raw[e]);
        uint8_t bit = (uint8_t)(1u << e);

        if (fabsf(d) > (2.0f * ROBOT_SPRING_MAX_DEFLECTION_RAD))
        {
            enc_valid &= (uint8_t)~bit;
            s_spring_out_of_range++;
        }
        if (enc_valid & bit)
        {
            s_spring_last[e] = d;
        }
        spring_rad[e] = s_spring_last[e];
        s_state.spring_angle[e] = s_spring_last[e];
    }
    s_state.enc_valid = enc_valid;

    /* ---- actuators, turns -> radians on the output side -------------- */
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_state.joint_pos[j] = act.pos[j] * TURNS_TO_RAD;
        s_state.joint_vel[j] = act.vel[j] * TURNS_TO_RAD;
    }

    /*
     * What each drive was actually told this tick - the number to plot
     * joint_pos against when tuning. NaN while nothing is being driven: a
     * held-over target would look like a command the joint is ignoring.
     */
    float sent_turns[NEXUS_NUM_JOINTS];
    uint8_t driving = act_get_sent_targets(sent_turns);

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_state.act_target[j] = driving ? (sent_turns[j] * TURNS_TO_RAD) : (float)NAN;
    }

    /* ---- contacts: one switch per foot, as floats for the observation - */
    uint8_t sw = contact_switches();

    /* The mask used to carry the switch bits OR-ed with two derived "this foot
       is down" bits. One switch per foot makes those the same bits, so the
       derived pair is gone and this is just the switches. */
    s_state.contacts = sw;
    for (int c = 0; c < NEXUS_NUM_CONTACTS; c++)
    {
        s_state.contact[c] = (sw & (1u << c)) ? 1.0f : 0.0f;
    }

    s_state.health           = (uint8_t)health_faults();
    s_state.safety_state     = (uint8_t)safety_state();

    /* ---- diagnostics ------------------------------------------------- */
    s_state.overruns    = s_overruns;
    s_state.usb_dropped = link_usb_tx_dropped();

    for (int b = 0; b < 2; b++)
    {
        uint32_t drop = act_tx_dropped((uint8_t)b);

        s_state.can_dropped[b] = (drop > 0xFFFFu) ? 0xFFFFu : (uint16_t)drop;

        uint32_t off = act_bus_off_count((uint8_t)b);

        s_state.can_bus_off[b] = (off > 0xFFu) ? 0xFFu : (uint8_t)off;
    }

    /*
     * Worst cycle since the last packet, then reset - so this is a per-packet
     * maximum rather than a since-boot one. At 1 kHz that makes it a live
     * signal the Pi can plot, instead of a high-water mark that never comes
     * back down after one bad moment.
     */
    s_state.loop_us_max = (s_loop_us_max > 0xFFFFu) ? 0xFFFFu
                                                    : (uint16_t)s_loop_us_max;
    s_loop_us_max = 0;

    uint32_t stalls = enc_stalls();
    s_state.enc_stalls = (stalls > 0xFFFFu) ? 0xFFFFu : (uint16_t)stalls;

    s_state.reserved0 = 0;
    s_state.reserved1 = 0;
    s_state.gains_seq = s_gains_seq;
    s_state.contact_ticks[0] = contact_stable_ticks(0);
    s_state.contact_ticks[1] = contact_stable_ticks(1);

    /*
     * The stride clock. It free-runs at 1 kHz whether or not the Pi is
     * talking, so the reference in the packet is always defined and the policy
     * can see where in the stride it is before it sends anything.
     *
     * It does NOT move the robot on its own. Targets only reach the drives
     * from an accepted command (reference + residual), so a Pi that stops
     * sending leaves the last target held until the 200 ms link fault idles
     * the actuators.
     */
    /* The pose the drives report, and whether it is complete enough to arm
       from - see s_pose_ok. */
    s_pose_ok = 1u;
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_meas_turns[j] = act.pos[j];
        if (!isfinite(act.pos[j]) || (act.pos_age[j] > POSE_STALE_TICKS))
        {
            s_pose_ok = 0u;
        }
    }

    /* Not armed - for any reason - means the next arm ramps again. */
    if (safety_state() != SAFETY_ARMED)
    {
        soft_start_idle();
    }

    /*
     * The clock stands still at the start phase through a held ramp and the
     * hold itself (soft_start.h), so ref_angle and phase in the packet show
     * the pose being held and the point the stride will start from.
     */
    if (soft_start_clock_frozen())
    {
        s_phase = SOFT_START_PHASE;
    }
    else
    {
        s_phase += (0.001f * APP_GAIT_SPEED) / GAIT_CYCLE_S;
        if (s_phase >= 1.0f)
        {
            s_phase -= 1.0f;
        }
    }

    build_reference(s_phase, s_ref_turns, s_gait_vel_turns, s_gait_tau_nm);
    apply_spring_predeflection(s_ref_turns, s_gait_tau_nm, s_drive_ref_turns);

    /*
     * Hand the drives this tick's velocity and torque. Every tick, because
     * the gait clock moves every tick while a command only arrives every
     * fourth one - latching these with the command would feed a value up to
     * 4 ms stale.
     *
     * Armed, they come through the soft start: the ramp's own velocity, the
     * gait's feedforward blended in (or none at all while held). Disarmed,
     * nothing - the first segment after an arm must not inherit a stride's
     * worth of torque from a gait the drives are not following yet.
     */
    float ff_vel[NEXUS_NUM_JOINTS], ff_tau[NEXUS_NUM_JOINTS];

    if (safety_state() == SAFETY_ARMED)
    {
        soft_start_tick(s_drive_ref_turns, s_gait_vel_turns, s_gait_tau_nm,
                        s_hold_pose_turns, s_cmd_ref_turns, ff_vel, ff_tau);
    }
    else
    {
        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            s_cmd_ref_turns[j] = s_drive_ref_turns[j];
            ff_vel[j] = 0.0f;
            ff_tau[j] = 0.0f;
        }
    }
    act_set_feedforward(ff_vel, ff_tau);

    /* The packet reports the reference in radians on the output side, the same
       units and convention as joint_pos, so the Pi can subtract them. The
       command residual stays in turns, matching what the drives take. */
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_state.ref_angle[j] = s_ref_turns[j] * 6.28318530718f;
    }
    s_state.phase        = s_phase;

    uint8_t flags = NEXUS_STREAM_GAIT_LIVE;
    switch (soft_start_mode())
    {
    case SOFT_RAMP: flags |= NEXUS_STREAM_RAMP; break;
    case SOFT_HOLD: flags |= NEXUS_STREAM_HOLD; break;
    case SOFT_RETURN: flags |= NEXUS_STREAM_RAMP | NEXUS_STREAM_RETURN; break;
    case SOFT_HOME: flags |= NEXUS_STREAM_RETURN; break;
    default: break;
    }
    if (s_ankles_zeroed)
    {
        flags |= NEXUS_STREAM_ANKLES_ZEROED;
    }
    if (s_arm_blocked != 0u)
    {
        flags |= NEXUS_STREAM_ARM_BLOCKED;
        s_arm_blocked--;
    }
    s_state.stream_flags = flags;

    /* Estimate before packing, so the packet carries this tick's fused state
       rather than the previous one. */
    fusion_tick(&imu, &act, spring_rad, enc_valid, s_state.contacts, s_state.timestamp_us);
    fusion_fill_state(&s_state);

    /* ---- actuator diagnostics --------------------------------------- */
    memcpy(s_state.act_torque, act.torque,     sizeof(s_state.act_torque));
    memcpy(s_state.act_error,  act.axis_error, sizeof(s_state.act_error));
    memcpy(s_state.act_state,  act.axis_state, sizeof(s_state.act_state));
    memcpy(s_state.act_flags,  act.flags,      sizeof(s_state.act_flags));

    link_usb_send_state(&s_state);

    *imu_out       = imu;
    *enc_valid_out = enc_valid;
}

void app_run(void)
{
    nexus_cmd_t cmd;

    for (;;)
    {
        imu_service();

        /* Keep feeding the 3-deep hardware CAN FIFO from the software queue.
           This runs far more often than once per tick, so the five frames
           queued each tick reach the wire well inside that tick. */
        act_tx_pump();

        /*
         * Gains from the Pi: rare, and only between runs. act_set_gains()
         * refuses while the robot is driving and the echo below simply does
         * not move, which is how the Pi is told it was refused.
         */
        nexus_gains_t gains;

        if (link_usb_take_gains(&gains))
        {
            drive_gains_t g[NEXUS_NUM_JOINTS];

            for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
            {
                g[j].pos_gain     = gains.pos_gain[j];
                g[j].vel_gain     = gains.vel_gain[j];
                g[j].vel_int_gain = gains.vel_int_gain[j];
            }
            if (act_set_gains(g))
            {
                s_gains_seq = (uint8_t)(gains.seq & 0xFFu);
            }
        }

        if (link_usb_take_command(&cmd))
        {
            float targets[NEXUS_NUM_JOINTS];

            /*
             * Nothing reaches the actuators without passing safety.c first.
             *
             * The CRC in link_usb.c proves the bytes survived the wire. It
             * says nothing about whether the policy that produced them is
             * still sane, whether this board is allowed to be moving, or
             * whether the frame is one we have already acted on - and a NaN
             * that gets as far as act_set_targets() poisons the interpolator
             * permanently.
             */
            /*
             * The residual is added to the reference INSIDE safety.c, so the
             * bounds are applied to the number the drives will actually be
             * given rather than to the correction on its own.
             */
            /*
             * ZERO_ANKLES: once per rising edge, and never while armed - a
             * drive told it is somewhere else mid-stride would leap there.
             * Handled before the enable logic, because the command carrying
             * it is normally an ENABLE-off one.
             */
            uint8_t zero = (cmd.flags & NEXUS_CMD_ZERO_ANKLES) ? 1u : 0u;

            if (zero && !s_zero_prev && (safety_state() != SAFETY_ARMED))
            {
                uint8_t l = act_set_abs_pos(NEXUS_J_L_ANKLE_PITCH, 0.0f);
                uint8_t r = act_set_abs_pos(NEXUS_J_R_ANKLE_PITCH, 0.0f);

                if (l && r)
                {
                    s_ankles_zeroed = 1u;
                }
            }
            s_zero_prev = zero;

            uint8_t enabled = (cmd.flags & NEXUS_CMD_ENABLE) ? 1u : 0u;
            uint8_t hold    = (cmd.flags & NEXUS_CMD_HOLD) ? 1u : 0u;
            uint8_t armed   = (safety_state() == SAFETY_ARMED) ? 1u : 0u;

            if (enabled && !armed && !s_pose_ok)
            {
                /*
                 * No complete pose to ramp from: a drive reporting NaN (the
                 * ankles after a reboot) or nothing at all. Without this the
                 * first segment would seed from the target instead of the leg
                 * - the jump the soft start exists to prevent. Not a rejection
                 * either: nothing is wrong with the command, so nothing counts
                 * towards a fault. The board just stays IDLE and says why.
                 */
                s_arm_blocked = ARM_BLOCKED_TICKS;
            }
            else
            {
                /*
                 * Not armed yet, a command is built on the MEASURED pose, so
                 * the arming target is where the robot already is and the
                 * ramp starts from rest. Armed, on whatever the soft start
                 * says this tick. And the residual is ignored until the gait
                 * is actually running.
                 */
                nexus_cmd_t c = cmd;
                const float *ref = armed ? s_cmd_ref_turns : s_meas_turns;

                if (!armed || soft_start_ignores_residual())
                {
                    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
                    {
                        c.residual[j] = 0.0f;
                    }
                }

                if (safety_accept_command(&c, ref, targets))
                {
                    act_set_targets(targets);

                    if (!armed)
                    {
                        soft_start_armed(ref, hold);
                    }
                    soft_start_hold_flag(hold);
                    soft_start_return_flag((cmd.flags & NEXUS_CMD_RETURN) ? 1u : 0u);
                }
            }
        }

        if (s_tick_pending == 0u)
        {
            continue;
        }

        /* Drop backlog rather than replaying stale ticks if a cycle ever
           overruns, but count what was dropped - a rising s_overruns is the
           first sign the 1 kHz budget is being exceeded. Claiming the counter
           has to be atomic against the TIM6 ISR that increments it. */
        uint32_t primask = critical_enter();
        uint32_t pending = s_tick_pending;
        s_tick_pending = 0;
        critical_exit(primask);

        /* Ignore the first few ticks. The very first cycle runs with a cold
           I-cache and does one-time work, and is legitimately slow (~1.4 ms
           measured) - counting it would latch the fault LED on every boot for
           a condition that is not a fault. */
        if (s_startup_grace < STARTUP_GRACE_TICKS)
        {
            s_startup_grace++;
        }
        else if (pending > 1u)
        {
            s_overruns += (pending - 1u);
        }

        /* Time the actual work of one tick, in microseconds (TIM2 runs at
           1 MHz). Measured BEFORE any printf so the diagnostic does not
           pollute the number it is reporting. */
        uint32_t t_start = __HAL_TIM_GET_COUNTER(&htim2);

        contact_poll();
        act_bus_service();
        act_tick_1khz();

        imu_sample_t tick_imu;
        uint8_t      tick_enc_valid;

        build_and_send_state(&tick_imu, &tick_enc_valid);
        health_tick(&tick_imu, tick_enc_valid);

        /* Act on what health_tick() just found. This is the step that takes
           the actuators away when nobody is flying the robot any more. */
        safety_tick(health_faults());

        /*
         * A wedged IMU used to be permanent: the recovery sequence existed,
         * was proven, and only ever ran once at boot. Run it again when the
         * sensor goes quiet - but never while the actuators are armed, since
         * it transmits to the sensor and discards the parser state, and the
         * estimator is mid-stride on that data.
         */
        if ((health_faults() & HEALTH_IMU) && (safety_state() != SAFETY_ARMED))
        {
            if (imu_recover_step())
            {
                /*
                 * A resync burst is 64 byte-spaced writes, about 8 ms, so the
                 * step that sends one deliberately blows the tick budget.
                 * Discard the backlog WITHOUT counting it, exactly as the
                 * status printf used to: those missed ticks are the recovery's
                 * doing, not the loop's, and letting them latch a timing fault
                 * would leave the robot unable to arm after fixing its own IMU.
                 */
                uint32_t pm = critical_enter();
                s_tick_pending = 0;
                critical_exit(pm);
            }
        }

        update_health_leds();

        /*
         * Refresh the watchdog HERE and nowhere else.
         *
         * This point is reached only by a cycle that ran every stage above to
         * completion. Refreshing from the idle loop instead would keep the
         * dog fed while the control loop was stalled - which is the exact
         * failure the watchdog exists to catch.
         */
        /*
         * Start the next tick's encoder read LAST, not first.
         *
         * It used to be kicked off at the top of the tick and the result read
         * microseconds later, in build_and_send_state(). Four 16-bit words at
         * 6.25 MHz have not landed by then, so the values used were normally
         * the PREVIOUS tick's - except on a long tick, when they were
         * sometimes this one's. The sample age jittered between 0 and 1 ms
         * with nothing in the packet to say which, straight into the contact
         * update.
         *
         * Starting it here gives the transfer the whole inter-tick gap to
         * complete, so the age is a fixed one tick every time. Deterministic
         * beats fresh: a constant 1 ms is something the estimator can be told
         * about, a coin flip is not.
         */
        enc_start_read();

        wdg_refresh();

        uint32_t dt = __HAL_TIM_GET_COUNTER(&htim2) - t_start;
        if (dt > s_loop_us_max)
        {
            s_loop_us_max = dt;
        }
#if NEXUS_LOOP_STATS
        /*
         * A second high-water mark for the console, because the packet's one
         * is reset every millisecond as it goes out - which makes it a live
         * signal for the Pi and useless for a line printed every two seconds.
         */
        if (dt > s_console_us_max)
        {
            s_console_us_max = dt;
        }
#endif

        /* Status line every 2 s. printf over a 115200 UART costs ~87 us per
           character and will itself cause overruns, so the counters are
           snapshotted first and the max is reset after reporting. */
#if NEXUS_LOOP_STATS
        static uint32_t s_report_tick;

        if (++s_report_tick >= 2000u)
        {
            s_report_tick = 0;

            uint32_t ovr = s_overruns;
            uint32_t mx  = s_console_us_max;

            s_console_us_max = 0;

            /*
             * The counters that did not earn a place in the packet go here.
             * TEC is the leading indicator that a CAN bus is failing -
             * act_rx_count() going flat is the lagging one - and the
             * not-in-closed-loop mask is the first thing worth knowing when
             * the robot refuses to arm.
             */
            printf("%s | loop max %lu us | overruns %lu | can drop %lu/%lu | "
                   "tec %u/%u | notCL 0x%03X | usb drop %lu | rej %lu | "
                   "enc stall %lu/err %lu | imu rec %lu | "
                   "health 0x%02lX watching 0x%02lX blink %u\r\n",
                   safety_state_name(),
                   (unsigned long)mx, (unsigned long)ovr,
                   (unsigned long)act_tx_dropped(0),
                   (unsigned long)act_tx_dropped(1),
                   (unsigned)act_tx_error_count(0),
                   (unsigned)act_tx_error_count(1),
                   (unsigned)act_not_closed_loop_mask(),
                   (unsigned long)link_usb_tx_dropped(),
                   (unsigned long)safety_rejected(),
                   (unsigned long)enc_stalls(),
                   (unsigned long)enc_errors(),
                   (unsigned long)imu_recoveries(),
                   (unsigned long)health_faults(),
                   (unsigned long)health_expected(),
                   (unsigned)health_blink_code());

            /* That printf blocks for ~4 ms at 115200 baud and unavoidably
               misses 3-4 ticks. Discard the backlog WITHOUT counting it:
               those are an artifact of the diagnostic itself, not a real
               overrun, and letting them accumulate lights the fault LED and
               makes the counter meaningless. Real overruns still register on
               every other tick. */
            uint32_t pm = critical_enter();
            s_tick_pending = 0;
            critical_exit(pm);
        }
#endif /* NEXUS_LOOP_STATS */
    }
}
