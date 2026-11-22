#include "health.h"
#include "app.h"
#include "imu_bno085.h"
#include "enc_as5047p.h"
#include "act_odrive.h"
#include "link_usb.h"
#include "link_proto.h"

/*
 * Every check works the same way: watch a counter that only moves when real
 * data arrives, and fault if it stops moving for too long. Counters are used
 * rather than "last seen" timestamps so nothing depends on clock wrap.
 *
 * Thresholds are in 1 kHz ticks, so the numbers are milliseconds.
 */
#define STALE_IMU_TICKS    50u   /* IMU runs at 400 Hz -> a sample every 2.5 ms */
#define STALE_CAN_TICKS   200u   /* ODrive heartbeat defaults to 100 ms         */
#define STALE_LINK_TICKS  200u   /* Pi commands expected at 250 Hz              */

/*
 * Encoders are read every tick, so a brief glitch should not raise a fault -
 * but a persistently bad parity/error flag should.
 *
 * 200 ms, not 20. These are the SPRING encoders: nothing commands a drive
 * from them. A bad read is already dropped in enc_as5047p.c and app.c, and
 * the estimator and the policy get the last good deflection. What a fault
 * here does is idle every axis - and on the robot the left knee's sensor
 * goes bad in bursts that outlast 20 ms (it was 87% valid on 2026-11-22), so
 * at 20 ms the robot would go limp mid-step over a sensor hiccup, which is
 * more dangerous than carrying a spring reading 0.2 s stale. A sensor that is
 * genuinely gone still faults the robot, a fifth of a second later.
 */
#define ENC_BAD_TICKS     200u

/*
 * Timing: a leaky budget of missed ticks, not "any miss is fatal".
 *
 * Every missed tick adds one to a bucket that drains by one every
 * TIMING_DRAIN_TICKS; the fault latches when the bucket passes
 * TIMING_BUDGET. So:
 *   - an isolated miss is counted (app_overruns() still shows it) but does not
 *     disarm the robot. On 2026-11-22 the first START armed, then faulted 60 ms
 *     later on exactly ONE missed tick (work max 510 us): the first run of the
 *     arming and ramp code, fetched cold from external flash (XIP). The same
 *     cold miss would hit again on the first WALK. One late Set_Input_Pos is
 *     invisible to a drive that holds its last target; idling every axis of a
 *     robot standing on its feet is not.
 *   - a stall of more than TIMING_BUDGET ms in one go faults at once.
 *   - a loop that is genuinely too slow faults within a few ms: an -O0 build
 *     at 300 Hz misses ~700 ticks a second, far past one per 100 ms.
 * Still latched once raised, and still cleared only by the ENABLE-off
 * handshake.
 */
#define TIMING_BUDGET       3u
#define TIMING_DRAIN_TICKS  100u

#define ENC_ALL_VALID     ((uint8_t)((1u << NEXUS_NUM_ENCODERS) - 1u))

static uint32_t s_expected;
static uint32_t s_faults;

static uint32_t s_prev_imu_seq;
static uint32_t s_prev_can_rx[2];
static uint32_t s_prev_link_cmds;
static uint32_t s_prev_overruns;
static uint32_t s_timing_bucket;
static uint16_t s_timing_drain;

static uint16_t s_idle_imu;
static uint16_t s_idle_can[2];
static uint16_t s_idle_link;
static uint16_t s_bad_enc;

/* Advance an idle counter, saturating so it cannot wrap back into "healthy". */
static void bump(uint16_t *idle)
{
    if (*idle < 0xFFFFu)
    {
        (*idle)++;
    }
}

static void set_fault(uint32_t bit, uint8_t faulted)
{
    if (faulted)
    {
        s_faults |= bit;
    }
    else
    {
        s_faults &= ~bit;
    }
}

void health_init(uint32_t expected_mask)
{
    s_expected       = expected_mask;
    s_faults         = 0;
    s_prev_imu_seq   = 0;
    s_prev_can_rx[0] = 0;
    s_prev_can_rx[1] = 0;
    s_prev_link_cmds = 0;
    s_prev_overruns  = 0;
    s_timing_bucket  = 0;
    s_timing_drain   = 0;
    s_idle_imu       = 0;
    s_idle_can[0]    = 0;
    s_idle_can[1]    = 0;
    s_idle_link      = 0;
    s_bad_enc        = 0;
}

void health_set_expected(uint32_t mask)
{
    s_expected = mask;
}

uint32_t health_expected(void)
{
    return s_expected;
}

void health_tick(const imu_sample_t *imu, uint8_t enc_valid)
{
    /* --- IMU: does the sample sequence number keep advancing? ------------ */

    /* The shared counter, deliberately: this asks "is the sensor alive at
       all", not "did a particular report update". The estimator's prediction
       gate is the one that needs the per-report counters. */
    if (imu->seq != s_prev_imu_seq)
    {
        s_prev_imu_seq = imu->seq;
        s_idle_imu     = 0;
    }
    else
    {
        bump(&s_idle_imu);
    }
    set_fault(HEALTH_IMU, (s_idle_imu > STALE_IMU_TICKS) ? 1u : 0u);

    /* --- Encoders: are all of them returning parity-clean angles? -------- */
    if (enc_valid == ENC_ALL_VALID)
    {
        s_bad_enc = 0;
    }
    else
    {
        bump(&s_bad_enc);
    }
    set_fault(HEALTH_ENC, (s_bad_enc > ENC_BAD_TICKS) ? 1u : 0u);

    /* --- CAN: is anything at all being received on each bus? ------------- */
    for (uint8_t b = 0; b < 2u; b++)
    {
        uint32_t rx = act_rx_count(b);

        if (rx != s_prev_can_rx[b])
        {
            s_prev_can_rx[b] = rx;
            s_idle_can[b]    = 0;
        }
        else
        {
            bump(&s_idle_can[b]);
        }
        set_fault((b == 0u) ? HEALTH_CAN1 : HEALTH_CAN2,
                  (s_idle_can[b] > STALE_CAN_TICKS) ? 1u : 0u);
    }

    /* --- Link: is the Pi still sending commands? ------------------------- */
    uint32_t cmds = link_usb_cmd_count();

    if (cmds != s_prev_link_cmds)
    {
        s_prev_link_cmds = cmds;
        s_idle_link      = 0;
    }
    else
    {
        bump(&s_idle_link);
    }
    set_fault(HEALTH_LINK, (s_idle_link > STALE_LINK_TICKS) ? 1u : 0u);

    /* --- Timing: latched, because a single missed tick still matters ----- */

    /*
     * Measured against the count already seen, not against zero: comparing to
     * zero meant one miss locked the robot out until a power cycle. Clearing
     * goes through the ENABLE-off handshake the Pi already performs.
     */
    uint32_t ov  = app_overruns();
    uint32_t new_misses = ov - s_prev_overruns;
    s_prev_overruns = ov;

    if (new_misses > 0u)
    {
        s_timing_bucket += new_misses;
    }
    else if (s_timing_bucket > 0u && ++s_timing_drain >= TIMING_DRAIN_TICKS)
    {
        s_timing_drain = 0;
        s_timing_bucket--;
    }

    if (s_timing_bucket > TIMING_BUDGET)
    {
        s_faults |= HEALTH_TIMING;
    }
}

void health_clear_latched(void)
{
    s_prev_overruns = app_overruns();
    s_timing_bucket = 0;
    s_timing_drain  = 0;
    s_faults &= ~(uint32_t)HEALTH_TIMING;
}

uint32_t health_faults(void)
{
    return s_faults & s_expected;
}

uint8_t health_blink_code(void)
{
    uint32_t f = health_faults();

    for (uint8_t i = 0; i < HEALTH_COUNT; i++)
    {
        if (f & (1u << i))
        {
            return (uint8_t)(i + 1u);   /* lowest-numbered fault wins */
        }
    }

    return 0u;
}
