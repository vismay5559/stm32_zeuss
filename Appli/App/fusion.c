#include "fusion.h"
#include "kinematics.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

/* ===================================================================== */
/*  ROBOT WIRING                                                          */
/* ===================================================================== */

/*
 * Which drive is which joint, the link lengths, and the encoder zeros all
 * live in robot_config.h now. They used to be spread across this file and
 * kinematics.c as placeholders that the estimator then believed completely.
 *
 * The important change is not where they live but what they are: forward
 * kinematics now takes ALL FOUR joint angles per leg from the ODrives.
 * hip_pitch and knee used to come from the AS5048A encoders, which sit after
 * the series springs and measure DEFLECTION - a small signed wind-up, not an
 * absolute joint angle. Both the README and app.c say so explicitly; only
 * this file disagreed, and it was the one feeding the filter.
 */

#define TURNS_TO_RAD   6.28318531f

/* ===================================================================== */
/*  CONVERGENCE                                                           */
/* ===================================================================== */

/*
 * The filter starts with 30 degrees of orientation uncertainty and 1 m/s of
 * velocity uncertainty, so its first outputs are meaningless. fused_valid only
 * reaches OK once the covariance of the states the Pi actually uses - velocity
 * and height - has come down and STAYED down.
 *
 * Thresholds are variances. 0.01 m^2/s^2 is a 0.1 m/s standard deviation;
 * 0.0025 m^2 is 5 cm. Both are loose enough to reach quickly and tight enough
 * that a policy can act on the numbers.
 */
#define CONV_VEL_VAR      0.02f     /* sum over 3 axes: ~0.08 m/s per axis */
#define CONV_HOLD_TICKS   500u      /* half a second of staying converged  */

/*
 * There is deliberately NO height threshold.
 *
 * Absolute position is unobservable (paper section 5.4): the contact update
 * pins the body RELATIVE to the foot, but the foot's own world height was
 * initialised from the body's uncertain position, so the pair drift together.
 * Measured here, height variance grows steadily - 1.3e-2 to 3.5e-2 over 20 s -
 * while velocity variance converges to 6.5e-3 and stays. Waiting on a height
 * threshold would mean never reporting OK.
 *
 * What IS well determined is height above the stance foot, which is what a
 * walking controller actually needs, and that is what anchoring below gives.
 */

/* If the IMU stops, the estimate is dead reckoning on nothing. */
#define IMU_STALE_TICKS   50u

/* ===================================================================== */

static inekf_t      s_f;
static kin_params_t s_kin;

static uint32_t s_prev_accel_seq;
static uint32_t s_prev_gyro_seq;
static uint32_t s_last_imu_us;
static uint16_t s_imu_idle;
static uint8_t  s_have_imu_time;

static uint8_t  s_foot_down[2];        /* what the filter currently believes */
static uint8_t  s_ground_anchored;     /* has the z datum been established?   */
static uint32_t s_converged_ticks;
static uint8_t  s_status;

/* ------------------------------------------------------------------ */

void fusion_init(void)
{
    inekf_init(&s_f, NULL);
    kin_defaults(&s_kin);

    s_prev_accel_seq  = 0;
    s_prev_gyro_seq   = 0;
    s_last_imu_us     = 0;
    s_imu_idle        = 0;
    s_have_imu_time   = 0;
    s_foot_down[0]    = 0;
    s_foot_down[1]    = 0;
    s_converged_ticks = 0;
    s_ground_anchored = 0;
    s_status          = NEXUS_FUSION_INVALID;
}

/*
 * Put the world z datum on the ground.
 *
 * The filter starts with the body at the origin, so without this the first
 * foot gets anchored 0.65 m BELOW zero and every height the Pi receives is
 * offset by a leg length. Shifting the body up so the first contact lands at
 * z = 0 makes fused_pos[2] mean "height above the ground I am standing on",
 * which is the quantity a walking policy wants.
 *
 * This is a choice of coordinate origin, not a measurement, so it is done once
 * and the covariance is left alone.
 */
static void anchor_ground(const inekf_real_t *p_body)
{
    inekf_real_t foot_world[3];
    lg_mat3_vec(foot_world, s_f.R, p_body);
    s_f.p[2] -= foot_world[2] + s_f.p[2];   /* body z such that foot z == 0 */
    s_ground_anchored = 1;
}

/* Gather one leg's four joint angles from the drives. */
static void leg_angles(const joint_src_t *map,
                       const act_telemetry_t *act,
                       float *q_out)
{
    for (int j = 0; j < KIN_LEG_JOINTS; j++)
    {
        float raw = act->pos[map[j].act_index] * TURNS_TO_RAD;

        q_out[j] = map[j].sign * raw + map[j].offset;
    }
}

/* Are all four angles for this leg coming from working, talking drives? */
static uint8_t leg_sources_ok(const joint_src_t *map, const act_telemetry_t *act)
{
    for (int j = 0; j < KIN_LEG_JOINTS; j++)
    {
        uint8_t idx = map[j].act_index;

        /* An axis in a fault state is not reporting a trustworthy angle. */
        if (act->axis_error[idx] != 0u)
        {
            return 0;
        }

        /*
         * Nor is one that has stopped reporting at all. When a bus goes quiet
         * act->pos simply stops changing: the last value persists and
         * axis_error stays 0, so without this the FK produces a confident foot
         * position from angles that are seconds old.
         */
        if (act->pos_age[idx] > ACT_POS_STALE_TICKS)
        {
            return 0;
        }
    }
    return 1;
}

/*
 * Did any of this leg's joints report a NEW position since the last tick?
 *
 * The contact update used to run every tick at 1 kHz while the underlying
 * joint angles only change at the drives' telemetry rate. Feeding the same
 * measurement into a Kalman update over and over, each time with independent
 * noise, shrinks the covariance far faster than the information justifies -
 * and an over-tight P makes the convergence gate in update_status() pass early
 * and stay passed, which is the wrong direction for a gate whose whole job is
 * to say "you may trust this now".
 *
 * pos_age is zeroed in the FDCAN ISR and aged once per tick in
 * act_tick_1khz(), which runs before this, so zero means "arrived since the
 * last tick" - exactly the "is this a new measurement" question.
 */
static uint8_t leg_has_new_measurement(const joint_src_t *map,
                                       const act_telemetry_t *act)
{
    for (int j = 0; j < KIN_LEG_JOINTS; j++)
    {
        if (act->pos_age[map[j].act_index] == 0u)
        {
            return 1;
        }
    }
    return 0;
}

/* Foot position in the body frame, refreshed every tick for foot_z. */
static inekf_real_t s_foot_body[2][3];
static uint8_t      s_foot_ok[2];

static void update_status(void)
{
    /* Variance of the states the Pi consumes. */
    inekf_real_t vvar = s_f.P[IDX(INEKF_IDX_V + 0, INEKF_IDX_V + 0)] +
                        s_f.P[IDX(INEKF_IDX_V + 1, INEKF_IDX_V + 1)] +
                        s_f.P[IDX(INEKF_IDX_V + 2, INEKF_IDX_V + 2)];

    uint8_t healthy = (s_imu_idle <= IMU_STALE_TICKS) &&
                      (inekf_num_contacts(&s_f) > 0);

    /*
     * A NaN anywhere means the filter has diverged. Comparing a NaN against
     * anything is false, so the check below rejects it naturally - but say so
     * explicitly, because a diverged filter must never report OK.
     */
    uint8_t sane = !(isnan((float)vvar) || isnan(s_f.p[2]) || isnan(s_f.v[0]));

    if (!sane)
    {
        inekf_reset(&s_f);          /* start over rather than emit garbage */
        s_foot_down[0] = 0;
        s_foot_down[1] = 0;
        s_ground_anchored = 0;
        s_converged_ticks = 0;
        s_status = NEXUS_FUSION_INVALID;
        return;
    }

    if (!healthy)
    {
        s_converged_ticks = 0;
        s_status = NEXUS_FUSION_INVALID;
        return;
    }

    if (vvar < CONV_VEL_VAR)
    {
        if (s_converged_ticks < 0xFFFFFFFFu)
        {
            s_converged_ticks++;
        }
    }
    else
    {
        s_converged_ticks = 0;
    }

    if (s_converged_ticks < CONV_HOLD_TICKS)
    {
        s_status = NEXUS_FUSION_CONVERGING;
        return;
    }

    /*
     * Converged is not the same as correct.
     *
     * The covariance says how well the filter agrees with ITSELF. It says
     * nothing about whether the leg geometry it is agreeing about matches the
     * robot. With placeholder link lengths, an unverified joint map and no
     * joint zeros, the filter converges beautifully onto a foot position that
     * does not exist - and the Pi has no way to tell that apart from a good
     * estimate.
     *
     * So OK requires someone to have measured the machine and said so in
     * robot_config.h. Until then the Pi sees CONVERGING forever, which is the
     * honest answer: running, plausible, not to be trusted.
     */
    s_status = robot_config_is_calibrated() ? NEXUS_FUSION_OK
                                            : NEXUS_FUSION_CONVERGING;
}

void fusion_tick(const imu_sample_t *imu,
                 const act_telemetry_t *act,
                 uint8_t contacts,
                 uint32_t now_us)
{
    /* ---- 1. predict, but only on a genuinely new INERTIAL sample ----- */

    /*
     * Both the accelerometer and the gyro must have moved on. The shared
     * imu->seq advances on any of the three reports, and the rotation vector
     * runs at 100 Hz independently of the two that matter here - so keying off
     * it propagated ~100 times a second over a fresh dt using gyro and
     * accelerometer readings that had already been integrated once.
     */
    uint8_t inertial_new = (imu->accel_seq != s_prev_accel_seq) &&
                           (imu->gyro_seq  != s_prev_gyro_seq);

    if (inertial_new)
    {
        s_prev_accel_seq = imu->accel_seq;
        s_prev_gyro_seq  = imu->gyro_seq;
        s_imu_idle       = 0;

        if (s_have_imu_time)
        {
            /* Unsigned subtraction gives the right answer across the 32-bit
               wrap of the 1 MHz counter (every ~71 minutes). */
            uint32_t d_us = now_us - s_last_imu_us;

            /* The BNO085 runs at 400 Hz, so ~2500 us. Anything wildly outside
               that is a dropped burst or a stall; propagating over it would
               inject a huge spurious motion, so skip and resynchronise. */
            if ((d_us > 200u) && (d_us < 50000u))
            {
                inekf_predict(&s_f, imu->gyro, imu->accel,
                              (inekf_real_t)d_us * 1e-6f);
            }
        }

        s_last_imu_us   = now_us;
        s_have_imu_time = 1;
    }
    else if (s_imu_idle < 0xFFFFu)
    {
        s_imu_idle++;
    }

    /* ---- 2. contact events and updates ------------------------------- */
    const uint8_t foot_mask[2] = { NEXUS_CONTACT_L_FOOT, NEXUS_CONTACT_R_FOOT };
    const joint_src_t *maps[2] = { g_leg_joints[0], g_leg_joints[1] };
    const inekf_real_t *hips[2] = { s_kin.left_hip_offset, s_kin.right_hip_offset };

    for (int leg = 0; leg < 2; leg++)
    {
        uint8_t down = (contacts & foot_mask[leg]) ? 1u : 0u;
        uint8_t ok   = leg_sources_ok(maps[leg], act);

        /*
         * A foot with unreadable joint angles is treated as lifted. Anchoring
         * a contact from a bad forward-kinematic position is worse than having
         * no contact at all: the filter would pull the whole state towards a
         * point that does not exist.
         */
        if (!ok)
        {
            down = 0;
        }

        float q[KIN_LEG_JOINTS], p_body[3], J[3 * KIN_LEG_JOINTS];

        /*
         * Compute forward kinematics every tick, not only when the foot is
         * planted. The contact update needs it when down, but foot_z is
         * reported continuously - a foot height that goes stale the moment the
         * leg leaves the ground would be worse than useless to a gait policy,
         * which cares most about the swing foot.
         */
        leg_angles(maps[leg], act, q);
        kin_foot(&s_kin, hips[leg], q, p_body, J);

        s_foot_body[leg][0] = p_body[0];
        s_foot_body[leg][1] = p_body[1];
        s_foot_body[leg][2] = p_body[2];
        s_foot_ok[leg]      = ok;

        if (down && !s_foot_down[leg])
        {
            if (!s_ground_anchored)
            {
                anchor_ground(p_body);
            }
            /* Touchdown. The FK at THIS instant fixes where the foot is
               anchored in the world, so it must use the angles from this tick
               and not a stale copy. */
            inekf_add_contact(&s_f, leg, p_body, J);
            s_foot_down[leg] = 1;
        }
        else if (!down && s_foot_down[leg])
        {
            inekf_remove_contact(&s_f, leg);
            s_foot_down[leg] = 0;
        }
        else if (down && leg_has_new_measurement(maps[leg], act))
        {
            /* Only on a genuinely new measurement - see the note on
               leg_has_new_measurement(). */
            inekf_update_contact(&s_f, leg, p_body, J);
        }
    }

    /* ---- 3. is any of this trustworthy yet? -------------------------- */
    update_status();
}

void fusion_fill_state(nexus_state_t *st)
{
    /* ---- estimator internals, for logging ---------------------------- */
    st->fused_pos[0] = s_f.p[0];
    st->fused_pos[1] = s_f.p[1];
    st->fused_pos[2] = s_f.p[2];

    inekf_velocity_world(&s_f, st->fused_vel);

    memcpy(st->fused_gyro_bias,  s_f.bg, sizeof(st->fused_gyro_bias));
    memcpy(st->fused_accel_bias, s_f.ba, sizeof(st->fused_accel_bias));

    st->fused_valid = s_status;

    /* ---- policy block ------------------------------------------------ */
    inekf_quaternion(&s_f, st->quat);
    st->pelvis_z = s_f.p[2];

    /*
     * World velocity rotated into the HEADING frame - world turned about z by
     * the robot's own yaw, so "forward" means where the robot faces rather
     * than where the world's x axis points.
     *
     * Yaw is the one part of the pose the filter cannot observe (paper 5.4),
     * so it drifts. That does not matter here: the policy only ever sees
     * velocity relative to the current heading, and the same drifting yaw is
     * used to define that heading. The error cancels.
     */
    {
        const float w = st->quat[0], x = st->quat[1];
        const float y = st->quat[2], z = st->quat[3];

        float yaw = atan2f(2.0f * (w * z + x * y),
                           1.0f - 2.0f * (y * y + z * z));
        float c = cosf(yaw), s = sinf(yaw);

        float vx = st->fused_vel[0], vy = st->fused_vel[1];

        st->vel_hdg[0] = -s * vx + c * vy;   /* lateral  */
        st->vel_hdg[1] =  c * vx + s * vy;   /* forward  */
        st->vel_hdg[2] =  st->fused_vel[2];  /* vertical */
    }

    /*
     * Foot height in the world: body position plus the foot offset rotated out
     * of the body frame. Since fused_pos[2] is anchored so the first contact
     * sits at z = 0, this reads as height above the stance ground.
     *
     * Table order is right then left; leg 0 is left internally.
     */
    {
        const inekf_real_t *R = s_f.R;
        const int leg_of[2] = { 1, 0 };      /* foot_z[0]=right, [1]=left */

        st->fk_valid = 0u;

        for (int i = 0; i < 2; i++)
        {
            int leg = leg_of[i];
            const inekf_real_t *b = s_foot_body[leg];

            if (s_foot_ok[leg])
            {
                /* Third row of R times the body-frame offset. */
                st->foot_z[i] = (float)(s_f.p[2] +
                                        R[6] * b[0] + R[7] * b[1] + R[8] * b[2]);
                st->fk_valid |= (uint8_t)(1u << i);
            }
            else
            {
                /*
                 * NaN, not 0.0.
                 *
                 * Zero is the single most misleading value available here: to
                 * a gait policy it reads as "this foot is exactly on the
                 * ground", which is precisely the wrong conclusion to draw
                 * from a leg whose joint angles are unreadable. A NaN survives
                 * the CRC, propagates visibly through any arithmetic, and
                 * cannot be mistaken for a measurement.
                 *
                 * fk_valid carries the same information as a bit, for callers
                 * that would rather branch than test for NaN.
                 */
                st->foot_z[i] = NAN;
            }
        }
    }
}

uint8_t fusion_status(void)
{
    return s_status;
}

uint32_t fusion_converged_ticks(void)
{
    return s_converged_ticks;
}
