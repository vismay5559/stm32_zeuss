#include "fusion.h"
#include "robot_config.h"
#include "zeus_kinematics.h"
#include <math.h>
#include <string.h>

/* ===================================================================== */
/*  ROBOT WIRING                                                          */
/* ===================================================================== */

/*
 * Which drive and encoder is which joint, and their signs and zeros, live in
 * robot_config.h. The leg geometry comes from the URDF, through
 * zeus_kinematics.h: each leg has two contact points, toe and heel, one per
 * foot switch, both seen from the IMU through the waist and the whole leg.
 *
 * Per leg the kinematics take eight angles. Four are the leg's drives, two are
 * the spring deflections of hip pitch and knee - ADDED to those joints' motor
 * side, because the drives measure before the spring and the AS5047Ps measure
 * only the spring (robot_config.h has the history of getting that wrong both
 * ways) - and two are the waist joints.
 *
 * THE WAIST IS BOLTED in this build: there are no waist actuators (link_proto.h
 * says why), so both waist angles are held at zero. They stay in the chain
 * because the IMU is mounted above them, and they are given a small variance
 * rather than none - a bolted bracket is stiff, not infinitely stiff, and
 * claiming certainty the robot does not have is how a filter talks itself into
 * a wrong answer.
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
/*  JOINT NOISE                                                           */
/* ===================================================================== */

/*
 * How far each joint angle is trusted, as a standard deviation in radians.
 * fusion.c pushes these through the leg Jacobian to get how far each contact
 * point is trusted, which is what the filter weighs against the IMU.
 *
 * They are not just sensor resolution. The drives' encoders resolve far finer
 * than a degree, but an uncalibrated zero, backlash and a flexing link all
 * show up at the foot as angle error too, so the motor figure is deliberately
 * loose. Tune on recorded data, not by datasheet.
 */
#define NOISE_MOTOR_RAD            0.0175f   /* 1 deg: the leg drives           */
#define NOISE_SPRING_RAD           0.005f    /* 0.3 deg: AS5047P, 14-bit        */
#define NOISE_WAIST_BOLTED_RAD     0.0087f   /* 0.5 deg: play in a bolted joint */

/* A spring that cannot be read could be anywhere within its travel. Using 0
   with this much doubt is honest; using 0 as if it were measured is not. */
#define NOISE_SPRING_UNKNOWN_RAD   (0.5f * ROBOT_SPRING_MAX_DEFLECTION_RAD)

/* ===================================================================== */

static inekf_t      s_f;

static uint32_t s_prev_accel_seq;
static uint32_t s_prev_gyro_seq;
static uint32_t s_last_imu_us;
static uint16_t s_imu_idle;
static uint8_t  s_have_imu_time;

static uint8_t  s_down[INEKF_MAX_CONTACTS];   /* what the filter currently believes */
static uint8_t  s_ground_anchored;     /* has the z datum been established?   */
static uint32_t s_converged_ticks;
static uint8_t  s_status;

/* ------------------------------------------------------------------ */

void fusion_init(void)
{
    inekf_init(&s_f, NULL);

    s_prev_accel_seq  = 0;
    s_prev_gyro_seq   = 0;
    s_last_imu_us     = 0;
    s_imu_idle        = 0;
    s_have_imu_time   = 0;
    memset(s_down, 0, sizeof(s_down));
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

/* One drive-sourced angle, in the URDF's convention. */
static float drive_angle(const joint_src_t *src, const act_telemetry_t *act)
{
    return src->sign * (act->pos[src->act_index] * TURNS_TO_RAD) + src->offset;
}

/*
 * Is this drive reporting an angle worth believing?
 *
 * Not if its axis is in a fault state. Nor if it has stopped reporting at all:
 * when a bus goes quiet act->pos simply stops changing - the last value
 * persists and axis_error stays 0 - so without the age check the kinematics
 * would produce a confident foot position from angles that are seconds old.
 */
static uint8_t drive_ok(const joint_src_t *src, const act_telemetry_t *act)
{
    return (act->axis_error[src->act_index] == 0u) &&
           (act->pos_age[src->act_index] <= ACT_POS_STALE_TICKS);
}

typedef struct
{
    float   q[ZEUS_KIN_NQ];
    float   var[ZEUS_KIN_NQ];      /* per-joint variance, rad^2             */
    uint8_t ok;                    /* every drive in the chain is believable */
    uint8_t fresh;                 /* some drive in the chain reported anew  */
} leg_input_t;

/*
 * Gather one leg's eight angles.
 *
 * "fresh" answers whether this tick carries a NEW measurement, and only the
 * drives count. The contact update used to run every tick while the drives
 * report slower than that; feeding the same angles into a Kalman update over
 * and over, each time as if its noise were independent, shrinks the covariance
 * far faster than the information justifies - and an over-tight P passes the
 * convergence gate early. The spring encoders are read every tick, but a leg
 * whose drives have not moved on has not told the filter anything new.
 * (pos_age is zeroed in the FDCAN ISR and aged in act_tick_1khz(), which runs
 * before this, so zero means "arrived since the last tick".)
 */
static void leg_input(int leg,
                      const act_telemetry_t *act,
                      const float spring_rad[NEXUS_NUM_ENCODERS],
                      uint8_t spring_valid,
                      leg_input_t *in)
{
    static const uint8_t motor_q[ROBOT_LEG_MOTORS] = {
        [ROBOT_JOINT_HIP_PITCH] = ZEUS_KIN_Q_HIP_PITCH,
        [ROBOT_JOINT_HIP_ROLL]  = ZEUS_KIN_Q_HIP_ROLL,
        [ROBOT_JOINT_KNEE]      = ZEUS_KIN_Q_KNEE_PITCH,
        [ROBOT_JOINT_ANKLE]     = ZEUS_KIN_Q_ANKLE_PITCH,
    };
    static const uint8_t waist_q[2] = { ZEUS_KIN_Q_WAIST_PITCH, ZEUS_KIN_Q_WAIST_ROLL };
    static const uint8_t spring_q[2] = {
        [ROBOT_SPRING_HIP]  = ZEUS_KIN_Q_HIP_PITCH_SPRING,
        [ROBOT_SPRING_KNEE] = ZEUS_KIN_Q_KNEE_PITCH_SPRING,
    };

    const float motor_var = NOISE_MOTOR_RAD * NOISE_MOTOR_RAD;

    in->ok    = 1u;
    in->fresh = 0u;

    for (int j = 0; j < ROBOT_LEG_MOTORS; j++)
    {
        const joint_src_t *src = &g_leg_joints[leg][j];

        in->q[motor_q[j]]   = drive_angle(src, act);
        in->var[motor_q[j]] = motor_var;
        in->ok    = (uint8_t)(in->ok & drive_ok(src, act));
        in->fresh = (uint8_t)(in->fresh | (act->pos_age[src->act_index] == 0u));
    }

    /* The waist is bolted at its zero pose - no drive to read. */
    for (int j = 0; j < 2; j++)
    {
        in->q[waist_q[j]]   = 0.0f;
        in->var[waist_q[j]] = NOISE_WAIST_BOLTED_RAD * NOISE_WAIST_BOLTED_RAD;
    }

    for (int k = 0; k < 2; k++)
    {
        uint8_t e = g_leg_springs[leg][k];
        float   d = spring_rad[e];

        if (((spring_valid & (1u << e)) != 0u) && (fabsf(d) <= ROBOT_SPRING_MAX_DEFLECTION_RAD))
        {
            in->q[spring_q[k]]   = d;
            in->var[spring_q[k]] = NOISE_SPRING_RAD * NOISE_SPRING_RAD;
        }
        else
        {
            in->q[spring_q[k]]   = 0.0f;
            in->var[spring_q[k]] = NOISE_SPRING_UNKNOWN_RAD * NOISE_SPRING_UNKNOWN_RAD;
        }
    }
}

/* Covariance of a contact point in the body frame: J * diag(var) * J^T. */
static void point_cov(float C[9], const float J[3 * ZEUS_KIN_NQ], const float var[ZEUS_KIN_NQ])
{
    for (int r = 0; r < 3; r++)
    {
        for (int c = r; c < 3; c++)
        {
            float s = 0.0f;
            for (int i = 0; i < ZEUS_KIN_NQ; i++)
            {
                s += J[r * ZEUS_KIN_NQ + i] * var[i] * J[c * ZEUS_KIN_NQ + i];
            }
            C[r * 3 + c] = s;
            C[c * 3 + r] = s;
        }
    }
}

/* Toe and heel in the body frame, refreshed every tick for foot_z. */
static float   s_point_body[2][ZEUS_KIN_POINTS][3];
static uint8_t s_leg_ok[2];

static void update_status(void)
{
    /* Variance of the states the Pi consumes. */
    inekf_real_t vvar = s_f.P[IDX(INEKF_IDX_V + 0, INEKF_IDX_V + 0)] +
                        s_f.P[IDX(INEKF_IDX_V + 1, INEKF_IDX_V + 1)] +
                        s_f.P[IDX(INEKF_IDX_V + 2, INEKF_IDX_V + 2)];

    uint8_t healthy = (s_imu_idle <= IMU_STALE_TICKS) &&
                      (inekf_num_contacts(&s_f) > 0);

    /*
     * A NaN anywhere means the filter has diverged, and a diverged filter must
     * never report OK.
     *
     * This used to check vvar, p[2] and v[0] - three numbers out of a state
     * that is a rotation matrix, three vectors, two contact positions and a
     * 21x21 covariance. A NaN in the rotation or in a contact position
     * survived all three checks and went out in the packet. Sweeping the whole
     * thing is 40-odd comparisons on a tick that already does tens of
     * thousands of multiply-accumulates.
     */
    uint8_t sane = 1;

    for (int i = 0; i < 9; i++)
    {
        if (isnan((float)s_f.R[i])) { sane = 0; }
    }
    for (int i = 0; i < 3; i++)
    {
        if (isnan((float)s_f.v[i]) || isnan((float)s_f.p[i]) ||
            isnan((float)s_f.bg[i]) || isnan((float)s_f.ba[i]))
        {
            sane = 0;
        }
    }
    for (int k = 0; k < INEKF_MAX_CONTACTS; k++)
    {
        if (!s_f.active[k])
        {
            continue;
        }
        for (int i = 0; i < 3; i++)
        {
            if (isnan((float)s_f.d[k][i])) { sane = 0; }
        }
    }

    /*
     * The covariance diagonal too. A negative variance is not a NaN but is
     * just as impossible, and it is the first visible sign of the covariance
     * losing positive-definiteness - which is what the Joseph form and the
     * explicit symmetrisation exist to prevent, and worth catching if they
     * ever fail to.
     */
    for (int i = 0; i < INEKF_ERR_MAX; i++)
    {
        inekf_real_t d = s_f.P[IDX(i, i)];

        if (isnan((float)d) || (d < 0.0f))
        {
            sane = 0;
        }
    }

    if (isnan((float)vvar))
    {
        sane = 0;
    }

    if (!sane)
    {
        inekf_reset(&s_f);          /* start over rather than emit garbage */
        memset(s_down, 0, sizeof(s_down));
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
                 const float spring_rad[NEXUS_NUM_ENCODERS],
                 uint8_t spring_valid,
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
    for (int leg = 0; leg < 2; leg++)
    {
        leg_input_t in;
        leg_input(leg, act, spring_rad, spring_valid, &in);

        /*
         * Which of this leg's two points are planted. A leg with unreadable
         * joint angles has none: anchoring a contact from a bad forward-
         * kinematic position is worse than having no contact at all, because
         * the filter would pull the whole state towards a point that does not
         * exist.
         */
        uint8_t down[ZEUS_KIN_POINTS];
        uint8_t any_down = 0u;

        for (int k = 0; k < ZEUS_KIN_POINTS; k++)
        {
            int slot = ZEUS_KIN_CONTACT(leg, k);

            down[k]  = (uint8_t)(in.ok && ((contacts & (1u << (unsigned)slot)) != 0u));
            any_down = (uint8_t)(any_down | down[k]);
        }

        /*
         * Kinematics every tick, not only when planted: foot_z is reported
         * continuously, and a swing foot is the one a gait policy cares about
         * most. The Jacobian is only wanted when a point is planted.
         */
        float p_body[ZEUS_KIN_POINTS][3];
        float J[ZEUS_KIN_POINTS][3 * ZEUS_KIN_NQ];

        (void)zeus_kin_foot((zeus_kin_side_t)leg, in.q, p_body, any_down ? J : NULL);

        memcpy(s_point_body[leg], p_body, sizeof(p_body));
        s_leg_ok[leg] = in.ok;

        for (int k = 0; k < ZEUS_KIN_POINTS; k++)
        {
            int slot = ZEUS_KIN_CONTACT(leg, k);

            if (down[k] && !s_down[slot])
            {
                float C[9];
                point_cov(C, J[k], in.var);

                if (!s_ground_anchored)
                {
                    anchor_ground(p_body[k]);
                }
                /* Touchdown. The kinematics at THIS instant fix where the
                   point is anchored in the world. */
                inekf_add_contact(&s_f, slot, p_body[k], C);
                s_down[slot] = 1u;
            }
            else if (!down[k] && s_down[slot])
            {
                inekf_remove_contact(&s_f, slot);
                s_down[slot] = 0u;
            }
            else if (down[k] && in.fresh)
            {
                float C[9];
                point_cov(C, J[k], in.var);
                inekf_update_contact(&s_f, slot, p_body[k], C);
            }
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

    /* nexus_state_t is packed for the wire, so taking the address of a member
     * to hand to a function is an unaligned-pointer risk in the general case —
     * these two members happen to sit on 4-byte offsets today, but nothing
     * stops a field being inserted ahead of them. Fill an aligned local and
     * copy, the same way the bias arrays below already do. */
    inekf_real_t vel_world[3];
    inekf_velocity_world(&s_f, vel_world);
    memcpy(st->fused_vel, vel_world, sizeof(st->fused_vel));

    memcpy(st->fused_gyro_bias,  s_f.bg, sizeof(st->fused_gyro_bias));
    memcpy(st->fused_accel_bias, s_f.ba, sizeof(st->fused_accel_bias));

    st->fused_valid = s_status;

    /* ---- policy block ------------------------------------------------ */
    inekf_real_t quat_wxyz[4];
    inekf_quaternion(&s_f, quat_wxyz);
    memcpy(st->quat, quat_wxyz, sizeof(st->quat));
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
     * Foot height in the world: the LOWER of toe and heel, each being body
     * position plus its offset rotated out of the body frame. Since
     * fused_pos[2] is anchored so the first contact sits at z = 0, this reads
     * as height above the stance ground - 0 for a planted foot however it is
     * tilted, positive for a lifted one.
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

            if (s_leg_ok[leg])
            {
                float lowest = 0.0f;

                for (int k = 0; k < ZEUS_KIN_POINTS; k++)
                {
                    const float *b = s_point_body[leg][k];
                    /* Third row of R times the body-frame offset. */
                    float z = s_f.p[2] + R[6] * b[0] + R[7] * b[1] + R[8] * b[2];

                    lowest = (k == 0 || z < lowest) ? z : lowest;
                }
                st->foot_z[i] = lowest;
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

uint8_t fusion_num_contacts(void)
{
    return (uint8_t)inekf_num_contacts(&s_f);
}
