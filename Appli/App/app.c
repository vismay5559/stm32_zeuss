#include "app.h"
#include "main.h"
#include "link_proto.h"
#include "link_usb.h"
#include "imu_bno085.h"
#include "enc_as5048a.h"
#include "act_odrive.h"
#include "contact.h"
#include "critical.h"
#include "health.h"
#include "safety.h"
#include "robot_config.h"
#include "watchdog.h"
#include "nexus_mode.h"
#include "fusion.h"
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

    contact_init();
    enc_init();
    act_init();
    link_usb_init();
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
         * The zeros live in robot_config.h and have not been measured yet.
         */
        s_state.spring_angle[e] = robot_spring_deflection((uint8_t)e, enc_raw[e]);
    }
    s_state.enc_valid = enc_valid;

    /* ---- actuators, turns -> radians on the output side -------------- */
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_state.joint_pos[j] = act.pos[j] * TURNS_TO_RAD;
        s_state.joint_vel[j] = act.vel[j] * TURNS_TO_RAD;
    }

    /* ---- contacts: four switches, as floats for the observation ------ */
    uint8_t sw = contact_switches();

    s_state.contacts = (uint8_t)(sw | contact_feet());
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
    s_state.contact_ticks[0] = contact_stable_ticks(0);
    s_state.contact_ticks[1] = contact_stable_ticks(1);

    /*
     * Reference angles and gait phase come from the gait library, which does
     * not run on the STM32 yet - only the leg test plays the trajectory. Space
     * is reserved in the packet so the Pi side can be written against the
     * final layout now.
     *
     * The old note here said the Pi could tell "because phase never advances",
     * which is indistinguishable from a gait legitimately parked at phase 0.
     * NEXUS_STREAM_GAIT_LIVE says it outright. Set the bit when the library
     * lands; silence that has to be inferred is not a protocol.
     */
    memset(s_state.ref_angle, 0, sizeof(s_state.ref_angle));
    s_state.phase        = 0.0f;
    s_state.stream_flags = 0u;

    /* Estimate before packing, so the packet carries this tick's fused state
       rather than the previous one. */
    fusion_tick(&imu, &act, s_state.contacts, s_state.timestamp_us);
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
            if (safety_accept_command(&cmd, targets))
            {
                act_set_targets(targets);
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
