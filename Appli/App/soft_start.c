#include "soft_start.h"

#include <math.h>
#include <string.h>

#define RAMP_TICKS   ((uint32_t)(SOFT_START_RAMP_S * 1000.0f + 0.5f))

static soft_mode_t s_mode;
static uint8_t     s_hold;          /* HOLD as it stands now                  */
static uint8_t     s_ramp_held;     /* the ramp's goal is the frozen pose     */
static uint32_t    s_t;             /* ticks into the ramp                    */
static float       s_from[NEXUS_NUM_JOINTS];
static float       s_home[NEXUS_NUM_JOINTS];   /* where RETURN goes: all zero  */
static float       s_last[NEXUS_NUM_JOINTS];   /* the last reference handed out */
static float       s_trim[NEXUS_NUM_JOINTS];   /* HOME trim, turns past zero    */
static uint16_t    s_trim_stall[NEXUS_NUM_JOINTS]; /* ticks at full trim, no progress */
static float       s_trim_at_max[NEXUS_NUM_JOINTS];/* where it was when trim maxed  */
static uint8_t     s_trim_off[NEXUS_NUM_JOINTS];   /* gave up on this joint          */
static float       s_meas[NEXUS_NUM_JOINTS];
static uint8_t     s_meas_ok;

void soft_start_idle(void)
{
    s_mode      = SOFT_IDLE;
    s_hold      = 0;
    s_ramp_held = 0;
    s_t         = 0;
    memset(s_from, 0, sizeof(s_from));
    memset(s_home, 0, sizeof(s_home));
    memset(s_last, 0, sizeof(s_last));
    memset(s_trim, 0, sizeof(s_trim));
    memset(s_trim_stall, 0, sizeof(s_trim_stall));
    memset(s_trim_off, 0, sizeof(s_trim_off));
    s_meas_ok = 0u;
}

void soft_start_armed(const float from[NEXUS_NUM_JOINTS], uint8_t hold)
{
    memcpy(s_from, from, sizeof(s_from));
    memset(s_home, 0, sizeof(s_home));   /* the standing pose - see soft_start.h */
    memcpy(s_last, from, sizeof(s_last));
    memset(s_trim, 0, sizeof(s_trim));
    memset(s_trim_stall, 0, sizeof(s_trim_stall));
    memset(s_trim_off, 0, sizeof(s_trim_off));
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

void soft_start_measured(const float meas[NEXUS_NUM_JOINTS], uint8_t ok)
{
    memcpy(s_meas, meas, sizeof(s_meas));
    s_meas_ok = (ok != 0u);
}

static uint8_t is_ankle(int j)
{
    return (j == NEXUS_J_L_ANKLE_PITCH) || (j == NEXUS_J_R_ANKLE_PITCH);
}

/* One tick of the HOME trim for joint j - see soft_start.h. */
static void home_trim(int j)
{
    if (is_ankle(j) || !s_meas_ok || s_trim_off[j])
    {
        return;
    }
    float err = s_home[j] - s_meas[j];

    /*
     * Give up on a joint that full trim cannot move. Friction breaks within a
     * second at full trim; a joint that has not moved half a degree after
     * SOFT_TRIM_GIVEUP_TICKS is not stuck on friction but jammed - on
     * 2026-11-24 a jammed right knee sat at 5.6 deg under 36 Nm of trim for
     * as long as HOME lasted, pushing into whatever had caught it. Let go:
     * the joint holds where it is with the plain PD toward zero, and nothing
     * escalates. Reset on the next arm.
     */
    if (fabsf(s_trim[j]) >= SOFT_TRIM_MAX)
    {
        if (s_trim_stall[j] == 0u)
        {
            s_trim_at_max[j] = s_meas[j];
        }
        if (fabsf(s_meas[j] - s_trim_at_max[j]) > SOFT_TRIM_DEADBAND)
        {
            s_trim_stall[j] = 0u;                 /* moving: friction, keep going */
        }
        else if (++s_trim_stall[j] >= SOFT_TRIM_GIVEUP_TICKS)
        {
            s_trim[j]     = 0.0f;
            s_trim_off[j] = 1u;
            return;
        }
    }

    if (err * s_trim[j] < 0.0f)
    {
        s_trim[j] = 0.0f;          /* crossed zero: stop pushing, let it brake */
        return;
    }
    if (fabsf(err) > SOFT_TRIM_DEADBAND)
    {
        s_trim[j] += SOFT_TRIM_RATE * err * 0.001f;
        if (s_trim[j] >  SOFT_TRIM_MAX) { s_trim[j] =  SOFT_TRIM_MAX; }
        if (s_trim[j] < -SOFT_TRIM_MAX) { s_trim[j] = -SOFT_TRIM_MAX; }
    }
}

void soft_start_return_flag(uint8_t ret)
{
    if (!ret || (s_mode == SOFT_RETURN) || (s_mode == SOFT_HOME) || (s_mode == SOFT_IDLE))
    {
        return;
    }

    if (s_mode == SOFT_RUN)
    {
        /*
         * From the gait (since 2026-11-24: BACK TO STANDING works from
         * anywhere). The last target included a policy residual this module
         * never saw, and the legs can be 10-30 deg off it mid-stride, so start
         * from where the joints ARE: the target steps toward the robot, which
         * only ever lowers the torque. Without a measured pose, the last gait
         * reference is the next best thing.
         */
        memcpy(s_from, s_meas_ok ? s_meas : s_last, sizeof(s_from));
    }
    else
    {
        /* A ramp (held or not) or the hold: from the reference the drives
           were last given, so the way back starts exactly where the way out
           stopped - mid-ramp included. */
        memcpy(s_from, s_last, sizeof(s_from));
    }
    memset(s_trim, 0, sizeof(s_trim));
    memset(s_trim_stall, 0, sizeof(s_trim_stall));
    memset(s_trim_off, 0, sizeof(s_trim_off));
    s_t    = 0;
    s_mode = SOFT_RETURN;
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
    return ((s_mode == SOFT_RAMP) && s_ramp_held) || (s_mode == SOFT_HOLD)
        || (s_mode == SOFT_RETURN) || (s_mode == SOFT_HOME);
}

uint8_t soft_start_ignores_residual(void)
{
    return s_mode != SOFT_RUN;
}

float soft_start_progress(void)
{
    if ((s_mode != SOFT_RAMP) && (s_mode != SOFT_RETURN))
    {
        return 1.0f;
    }
    return (float)s_t / (float)RAMP_TICKS;
}

void soft_start_tick(const float gait_ref[NEXUS_NUM_JOINTS],
                     const float gait_vel[NEXUS_NUM_JOINTS],
                     const float gait_tau[NEXUS_NUM_JOINTS],
                     const float hold_pose[NEXUS_NUM_JOINTS],
                     const float hold_tau[NEXUS_NUM_JOINTS],
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
            /* Held: the goal is the frozen start pose, and its torque blends
               in with it (soft_start.h). Running: the goal is the live
               stride, so its own velocity and torque come in with the blend. */
            float goal = s_ramp_held ? hold_pose[j] : gait_ref[j];
            float gv   = s_ramp_held ? 0.0f         : gait_vel[j];
            float gt   = s_ramp_held ? (SOFT_HOLD_TAU_SCALE * hold_tau[j]) : gait_tau[j];

            out_ref[j] = s_from[j] + s * (goal - s_from[j]);
            out_vel[j] = ds * (goal - s_from[j]) + s * gv;
            out_tau[j] = s * gt;
        }
        memcpy(s_last, out_ref, sizeof(s_last));

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
            out_tau[j] = SOFT_HOLD_TAU_SCALE * hold_tau[j];

            /* Free ankles (soft_start.h): the target is wherever the ankle
               is, so the PD has no spring - only the drive's damping. */
            if (SOFT_HOLD_ANKLES_FREE && is_ankle(j) && s_meas_ok && isfinite(s_meas[j]))
            {
                /* ...but only within SOFT_HOLD_ANKLE_WINDOW of the start
                   pose: past it the target stops following and the PD pulls
                   back, so a free ankle cannot drift out of what WALK allows. */
                float lo = hold_pose[j] - SOFT_HOLD_ANKLE_WINDOW;
                float hi = hold_pose[j] + SOFT_HOLD_ANKLE_WINDOW;
                float m  = s_meas[j];
                out_ref[j] = (m < lo) ? lo : ((m > hi) ? hi : m);
                out_tau[j] = 0.0f;
            }
        }
        memcpy(s_last, out_ref, sizeof(s_last));
        return;

    case SOFT_RETURN:
    {
        /* The arming ramp in reverse: same smoothstep, nothing pushing. */
        float x  = (float)s_t / (float)RAMP_TICKS;
        float s  = x * x * (3.0f - 2.0f * x);
        float ds = 6.0f * x * (1.0f - x) / SOFT_START_RAMP_S;

        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            out_ref[j] = s_from[j] + s * (s_home[j] - s_from[j]);
            out_vel[j] = ds * (s_home[j] - s_from[j]);
            out_tau[j] = 0.0f;
        }
        memcpy(s_last, out_ref, sizeof(s_last));
        if (++s_t >= RAMP_TICKS)
        {
            s_mode = SOFT_HOME;
        }
        return;
    }

    case SOFT_HOME:
        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            home_trim(j);
            out_ref[j] = s_home[j] + s_trim[j];
            out_vel[j] = 0.0f;
            out_tau[j] = 0.0f;
        }
        memcpy(s_last, out_ref, sizeof(s_last));
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
        memcpy(s_last, out_ref, sizeof(s_last));   /* a RETURN's fallback start */
        return;
    }
}
