#include "fusion.h"
#include "kinematics.h"
#include <math.h>
#include <string.h>

/* ===================================================================== */
/*  ROBOT WIRING - joint angles come from two different subsystems         */
/* ===================================================================== */

/*
 * Forward kinematics wants four angles per leg, in chain order:
 *
 *     [ hip_pitch, hip_roll, knee_pitch, ankle_pitch ]
 *
 * On Zeus those four come from two different places (matching the joint map
 * in zeus_26's kinematics.py):
 *
 *   SEA joints  0, 2, 5, 7  <- AS5048A encoders, already radians
 *   QDD joints  1, 3, 6, 8  <- ODrive act_pos, in TURNS
 *
 * So per leg: hip_pitch and knee come off the springs, hip_roll and ankle come
 * off CAN. Getting this table wrong produces a foot position that is confidently
 * wrong, which the filter will then trust - so it is written explicitly rather
 * than computed.
 */

#define SRC_ENCODER   0
#define SRC_ACTUATOR  1

typedef struct
{
    uint8_t source;   /* SRC_ENCODER or SRC_ACTUATOR */
    uint8_t index;    /* encoder index, or joint index into act_pos           */
    float   sign;     /* +1 or -1, to match the FK sign convention            */
    float   offset;   /* radians added after scaling: the zero of that joint  */
} joint_src_t;

/*
 * offset and sign CANNOT be determined without the hardware. They are the
 * calibration: put the leg in a known pose, read the raw values, and solve for
 * them. Until that is done the FK output is offset by however wrong these are,
 * and the filter will happily believe it.
 */
static const joint_src_t s_left[KIN_LEG_JOINTS] = {
    /* hip_pitch   */ { SRC_ENCODER,  0, 1.0f, 0.0f },   /* SEA joint 0 */
    /* hip_roll    */ { SRC_ACTUATOR, 1, 1.0f, 0.0f },   /* QDD joint 1 */
    /* knee_pitch  */ { SRC_ENCODER,  1, 1.0f, 0.0f },   /* SEA joint 2 */
    /* ankle_pitch */ { SRC_ACTUATOR, 3, 1.0f, 0.0f },   /* QDD joint 3 */
};

static const joint_src_t s_right[KIN_LEG_JOINTS] = {
    /* hip_pitch   */ { SRC_ENCODER,  2, 1.0f, 0.0f },   /* SEA joint 5 */
    /* hip_roll    */ { SRC_ACTUATOR, 6, 1.0f, 0.0f },   /* QDD joint 6 */
    /* knee_pitch  */ { SRC_ENCODER,  3, 1.0f, 0.0f },   /* SEA joint 7 */
    /* ankle_pitch */ { SRC_ACTUATOR, 8, 1.0f, 0.0f },   /* QDD joint 8 */
};

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

static uint32_t s_prev_imu_seq;
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

    s_prev_imu_seq    = 0;
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

/* Gather one leg's four joint angles from wherever they actually live. */
static void leg_angles(const joint_src_t *map,
                       const float *enc_rad,
                       const act_telemetry_t *act,
                       float *q_out)
{
    for (int j = 0; j < KIN_LEG_JOINTS; j++)
    {
        float raw;

        if (map[j].source == SRC_ENCODER)
        {
            raw = enc_rad[map[j].index];            /* already radians */
        }
        else
        {
            raw = act->pos[map[j].index] * TURNS_TO_RAD;
        }

        q_out[j] = map[j].sign * raw + map[j].offset;
    }
}

/* Are all four angles for this leg coming from working sensors? */
static uint8_t leg_sources_ok(const joint_src_t *map, uint8_t enc_valid,
                              const act_telemetry_t *act)
{
    for (int j = 0; j < KIN_LEG_JOINTS; j++)
    {
        if (map[j].source == SRC_ENCODER)
        {
            if ((enc_valid & (1u << map[j].index)) == 0u)
            {
                return 0;
            }
        }
        else
        {
            /* An axis in a fault state is not reporting a trustworthy angle. */
            if (act->axis_error[map[j].index] != 0u)
            {
                return 0;
            }
        }
    }
    return 1;
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

    s_status = (s_converged_ticks >= CONV_HOLD_TICKS) ? NEXUS_FUSION_OK
                                                      : NEXUS_FUSION_CONVERGING;
}

void fusion_tick(const imu_sample_t *imu,
                 const float *enc_rad, uint8_t enc_valid,
                 const act_telemetry_t *act,
                 uint8_t contacts,
                 uint32_t now_us)
{
    /* ---- 1. predict, but only on a genuinely new IMU sample ---------- */
    if (imu->seq != s_prev_imu_seq)
    {
        s_prev_imu_seq = imu->seq;
        s_imu_idle     = 0;

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
    const joint_src_t *maps[2] = { s_left, s_right };
    const inekf_real_t *hips[2] = { s_kin.left_hip_offset, s_kin.right_hip_offset };

    for (int leg = 0; leg < 2; leg++)
    {
        uint8_t down = (contacts & foot_mask[leg]) ? 1u : 0u;
        uint8_t ok   = leg_sources_ok(maps[leg], enc_valid, act);

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
        leg_angles(maps[leg], enc_rad, act, q);
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
        else if (down)
        {
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

        for (int i = 0; i < 2; i++)
        {
            int leg = leg_of[i];
            const inekf_real_t *b = s_foot_body[leg];

            /* Third row of R times the body-frame offset. */
            st->foot_z[i] = s_foot_ok[leg]
                ? (float)(s_f.p[2] + R[6] * b[0] + R[7] * b[1] + R[8] * b[2])
                : 0.0f;
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
