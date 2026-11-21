#include "soft_start.h"

#include <string.h>

#define RAMP_TICKS   ((uint32_t)(SOFT_START_RAMP_S * 1000.0f + 0.5f))

static soft_mode_t s_mode;
static uint8_t     s_hold;          /* HOLD as it stands now                  */
static uint8_t     s_ramp_held;     /* the ramp's goal is the frozen pose     */
static uint32_t    s_t;             /* ticks into the ramp                    */
static float       s_from[NEXUS_NUM_JOINTS];

void soft_start_idle(void)
{
    s_mode      = SOFT_IDLE;
    s_hold      = 0;
    s_ramp_held = 0;
    s_t         = 0;
    memset(s_from, 0, sizeof(s_from));
}

void soft_start_armed(const float from[NEXUS_NUM_JOINTS], uint8_t hold)
{
    memcpy(s_from, from, sizeof(s_from));
    s_hold      = (hold != 0u);
    s_ramp_held = s_hold;
    s_t         = 0;
    s_mode      = SOFT_RAMP;
}

void soft_start_hold_flag(uint8_t hold)
{
    s_hold = (hold != 0u);

    if ((s_mode == SOFT_HOLD) && !s_hold)
    {
        s_mode = SOFT_RUN;
    }
}

soft_mode_t soft_start_mode(void)
{
    return s_mode;
}

uint8_t soft_start_clock_frozen(void)
{
    /* Frozen through a held ramp and the hold itself - also when HOLD was
       cleared mid-ramp: the goal is still the frozen pose until the ramp ends,
       and starting the clock under it would move the goal it is ramping to. */
    return ((s_mode == SOFT_RAMP) && s_ramp_held) || (s_mode == SOFT_HOLD);
}

uint8_t soft_start_ignores_residual(void)
{
    return s_mode != SOFT_RUN;
}

float soft_start_progress(void)
{
    if (s_mode != SOFT_RAMP)
    {
        return 1.0f;
    }
    return (float)s_t / (float)RAMP_TICKS;
}

void soft_start_tick(const float gait_ref[NEXUS_NUM_JOINTS],
                     const float gait_vel[NEXUS_NUM_JOINTS],
                     const float gait_tau[NEXUS_NUM_JOINTS],
                     const float hold_pose[NEXUS_NUM_JOINTS],
                     float out_ref[NEXUS_NUM_JOINTS],
                     float out_vel[NEXUS_NUM_JOINTS],
                     float out_tau[NEXUS_NUM_JOINTS])
{
    switch (s_mode)
    {
    case SOFT_RAMP:
    {
        /*
         * Smoothstep: s = 3x^2 - 2x^3, ds/dt = 6x(1-x)/T. Zero velocity at
         * both ends, so the move starts and finishes without a kick, and its
         * peak speed is 1.5x the average - for a 35 degree move over 3 s,
         * about 17 deg/s.
         */
        float x  = (float)s_t / (float)RAMP_TICKS;
        float s  = x * x * (3.0f - 2.0f * x);
        float ds = 6.0f * x * (1.0f - x) / SOFT_START_RAMP_S;

        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            /* Held: the goal is the frozen start pose and nothing pushes.
               Running: the goal is the live stride, so its own velocity and
               torque come in with the blend. */
            float goal = s_ramp_held ? hold_pose[j] : gait_ref[j];
            float gv   = s_ramp_held ? 0.0f         : gait_vel[j];
            float gt   = s_ramp_held ? 0.0f         : gait_tau[j];

            out_ref[j] = s_from[j] + s * (goal - s_from[j]);
            out_vel[j] = ds * (goal - s_from[j]) + s * gv;
            out_tau[j] = s * gt;
        }

        if (++s_t >= RAMP_TICKS)
        {
            /*
             * A held ramp ends in HOLD unless HOLD was cleared on the way, in
             * which case it goes straight to RUN - from the frozen start
             * phase, which is where the clock still is.
             */
            s_mode = (s_ramp_held && s_hold) ? SOFT_HOLD : SOFT_RUN;
        }
        return;
    }

    case SOFT_HOLD:
        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            out_ref[j] = hold_pose[j];
            out_vel[j] = 0.0f;
            out_tau[j] = 0.0f;
        }
        return;

    case SOFT_RUN:
    case SOFT_IDLE:
    default:
        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            out_ref[j] = gait_ref[j];
            out_vel[j] = gait_vel[j];
            out_tau[j] = gait_tau[j];
        }
        return;
    }
}
