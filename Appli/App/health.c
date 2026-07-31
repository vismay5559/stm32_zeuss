#include "health.h"
#include "app.h"
#include "imu_bno085.h"
#include "enc_as5048a.h"
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

/* Encoders are read every tick, so a brief glitch should not raise a fault -
   but a persistently bad parity/error flag should. */
#define ENC_BAD_TICKS      20u

#define ENC_ALL_VALID     ((uint8_t)((1u << NEXUS_NUM_ENCODERS) - 1u))

static uint32_t s_expected;
static uint32_t s_faults;

static uint32_t s_prev_imu_seq;
static uint32_t s_prev_can_rx[2];
static uint32_t s_prev_link_cmds;

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

void health_tick(void)
{
    /* --- IMU: does the sample sequence number keep advancing? ------------ */
    imu_sample_t imu;
    imu_get(&imu);

    if (imu.seq != s_prev_imu_seq)
    {
        s_prev_imu_seq = imu.seq;
        s_idle_imu     = 0;
    }
    else
    {
        bump(&s_idle_imu);
    }
    set_fault(HEALTH_IMU, (s_idle_imu > STALE_IMU_TICKS) ? 1u : 0u);

    /* --- Encoders: are all of them returning parity-clean angles? -------- */
    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint8_t  valid;
    enc_get(angle, &valid);

    if (valid == ENC_ALL_VALID)
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
    if (app_overruns() != 0u)
    {
        s_faults |= HEALTH_TIMING;
    }
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
