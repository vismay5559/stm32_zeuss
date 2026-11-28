#include "robot_config.h"

#include <math.h>

/*
 * Which telemetry index feeds each FK slot. The indices are the NEXUS_J_*
 * joint map in link_proto.h - node 1 hip_pitch, node 2 hip_roll, node 3 knee,
 * node 4 ankle, left leg on bus 0 and right on bus 1. Node 5 on each bus would
 * be a waist joint; this build has none.
 *
 * Every sign here is +1, and that is NOT "not measured": the drives' own
 * directions are corrected once, at the CAN boundary, by g_joint_dir below,
 * so act->pos is already in the URDF's convention by the time it gets here.
 * A -1 here would undo that correction for the estimator alone.
 *
 * Offsets are 0 because the drives are zeroed in the URDF's zero pose. Hip
 * pitch, hip roll and knee are non-backdrivable (47:1, 26:1, 47:1) with the
 * encoder on the output, so that zero survives a power cycle. The ankle is
 * 15:1 and backdrivable, so its zero is NOT yet trustworthy - still open.
 */
const joint_src_t g_leg_joints[2][ROBOT_LEG_MOTORS] = {
    /* --- left leg, bus 0 --- */
    {
        /* hip_pitch   */ { NEXUS_J_L_HIP_PITCH,   1.0f, 0.0f },   /* node 1 */
        /* hip_roll    */ { NEXUS_J_L_HIP_ROLL,    1.0f, 0.0f },   /* node 2 */
        /* knee_pitch  */ { NEXUS_J_L_KNEE_PITCH,  1.0f, 0.0f },   /* node 3 */
        /* ankle_pitch */ { NEXUS_J_L_ANKLE_PITCH, 1.0f, 0.0f },   /* node 4 */
    },
    /* --- right leg, bus 1 --- */
    {
        /* hip_pitch   */ { NEXUS_J_R_HIP_PITCH,   1.0f, 0.0f },   /* node 1 */
        /* hip_roll    */ { NEXUS_J_R_HIP_ROLL,    1.0f, 0.0f },   /* node 2 */
        /* knee_pitch  */ { NEXUS_J_R_KNEE_PITCH,  1.0f, 0.0f },   /* node 3 */
        /* ankle_pitch */ { NEXUS_J_R_ANKLE_PITCH, 1.0f, 0.0f },   /* node 4 */
    },
};

/*
 * Starting gains, per joint, in the NEXUS_J_* order. Hip pitch and knee carry
 * the bench leg's tuned numbers; hip roll has never been tuned and starts
 * where hip pitch did.
 *
 * GEAR RATIOS, confirmed on the assembled robot 2026-10-31:
 *
 *     hip pitch, knee pitch   47:1   encoder on the load (output)
 *     hip roll                26:1   encoder on the load (confirmed 2026-11-22)
 *     ankle pitch             15:1   load side on the RIGHT, MOTOR (commutation) on the
 *                                LEFT - see g_joint_enc_ratio (2026-11-22)
 *
 * The ankle read 9:1 here and in docs/ODRIVE_COMMANDS.md until that date, from
 * the first bench leg. It is 15:1. Anything scaled by the old 9 travels 9/15 =
 * 0.6 of what it was asked for, which looks exactly like a gain that needs
 * turning up. The other live copy of this number is s_cmd_scale[] in
 * test_leg_can.c, which still reads 9.0f.
 *
 * WHICH SHAFT EACH NUMBER REFERS TO is the open question, and it decides
 * whether these gains are right or out by a factor of the ratio squared:
 *
 *   - the ODrive's position and velocity are turns of whatever shaft THAT
 *     drive's encoder sits on, which is not the same shaft on every joint;
 *   - its torque is always MOTOR torque;
 *   - so pos_gain = KP/KD whichever shaft it is - a ratio of encoder units, so
 *     the gearbox cancels - while vel_gain is KD*2pi/N on a load-side encoder
 *     and KD*2pi/N^2 on a motor-side one.
 *
 * Note that robot_config.h above states every drive reports output-shaft
 * position with the ratio already applied, which contradicts the MOTOR entry
 * for the ankle. The drives themselves are the authority; settle it before
 * typing a tuned table in. Hang a known mass on a known lever arm and compare
 * the drive's reported torque with m*g*r.
 */
/*
 * THE POLICY'S GAINS, 2026-11-22.
 *
 * z1_softland_tol05 was trained with every joint as a pure PD on the gearbox
 * output, no integrator (zeus_env.py, manifest KP/KD):
 *
 *     hip pitch, knee   KP 150 Nm/rad   KD 12  Nm s/rad
 *     hip roll          KP  60          KD  4
 *     ankle             KP  30          KD  1.2
 *
 * The ODrive's cascade (pos_gain into vel_gain, torque in MOTOR Nm) is that
 * PD exactly when, ignoring the integrator,
 *
 *     KP = M vel_gain pos_gain / 2pi      KD = M vel_gain / 2pi
 *
 * with M the gear ratio N for a load-side encoder and N^2 for a motor-side
 * one (the left ankle, g_joint_enc_ratio). So pos_gain = KP/KD and
 * vel_gain = KD 2pi / M, and vel_integrator_gain = 0 because training had
 * none. The bench values these replace (30 / 10 / 50 on hips and knees,
 * 17 / 0.3 / 1.5 on ankles) were 6-15x stiffer than anything the policy saw.
 *
 * Assumes each drive's torque is what it is told (torque constant right).
 * Check by hanging a known mass on a known lever and comparing the drive's
 * torque estimate with m g r.
 *
 * KNOWN RISK: with no integrator, friction leaves a steady error of
 * friction / KP - about 2 deg per Nm on an ankle. The builder has seen the
 * hip rolls and ankles stall without one on the bench. First run in the air;
 * if tracking error on those joints sits above a couple of degrees, give
 * THOSE joints a small integrator rather than stiffening everything.
 */
const drive_gains_t g_drive_gains[NEXUS_NUM_JOINTS] = {
    /*                   pos_gain  vel_gain   vel_int     KP / KD out   */
    /* L hip_pitch   */ { 12.5f,  1.60420f,   0.0f },  /* 150 / 12, N 47     */
    /* L hip_roll    */ { 15.0f,  0.96664f,   0.0f },  /*  60 / 4,  N 26     */
    /* L knee_pitch  */ { 12.5f,  1.60420f,   0.0f },  /* 150 / 12, N 47     */
    /* L ankle_pitch */ { 25.0f,  0.033510f,  0.0f },  /*  30 / 1.2, N^2 225 */
    /* R hip_pitch   */ { 12.5f,  1.60420f,   0.0f },  /* 150 / 12, N 47     */
    /* R hip_roll    */ { 15.0f,  0.96664f,   0.0f },  /*  60 / 4,  N 26     */
    /* R knee_pitch  */ { 12.5f,  1.60420f,   0.0f },  /* 150 / 12, N 47     */
    /* R ankle_pitch */ { 25.0f,  0.033510f,  0.0f },  /*  30 / 1.2, N^2 225 */
};

/*
 * Friction feed-forward (robot_config.h). From the first ground walk,
 * 2026-11-23 (zeus_20261123_234848.flt, 998.8-1002.8 s), a least-squares fit
 * of drive torque x gear against sign(velocity) while moving:
 *
 *                 measured Coulomb    used    ~fraction
 *   hip pitch     15.5 L / 15.7 R      9.0      58%
 *   knee          10.9 L /  4.2 R      4.0      37% of L - R fit is load-confounded
 *   hip roll       2.2 L /  4.1 R      1.5      68% of L
 *   ankle          0.8 / -0.3          0        within noise
 *
 * Under on purpose (app.c). If hips still stall short with this, raise hip
 * pitch in steps of 2 Nm and watch for a joint that moves on its own.
 */
/*
 * OFF since 2026-11-25 - back to the first ground walk's configuration. The
 * robot walked 4 s on 2026-11-23 without it, and not past 1-3 s since several
 * changes went in together; they come back one at a time, each tested on its
 * own. The values to restore are 9 / 1.5 / 4 / 0 (hip pitch, hip roll, knee,
 * ankle), per the table above.
 */
const float g_friction_ff_nm[NEXUS_NUM_JOINTS] = {
    /* L hip_pitch */ 0.0f, /* L hip_roll */ 0.0f, /* L knee */ 0.0f, /* L ankle */ 0.0f,
    /* R hip_pitch */ 0.0f, /* R hip_roll */ 0.0f, /* R knee */ 0.0f, /* R ankle */ 0.0f,
};

const uint8_t g_leg_springs[2][2] = {
    { NEXUS_ENC_L_HIP_PITCH, NEXUS_ENC_L_KNEE_PITCH },
    { NEXUS_ENC_R_HIP_PITCH, NEXUS_ENC_R_KNEE_PITCH },
};

/*
 * Gear ratio per joint. 47:1 hip pitch and knee, 26:1 hip roll, 15:1 ankle -
 * confirmed on the assembled robot, and the same values the policy trained
 * against (zeus_model.py: N_SEA, N_ROLL, N_ANK = 47, 26, 15).
 *
 * The ankle read 9:1 in this repo until 2026-10-31, from the first bench leg.
 * test_leg_can.c's s_cmd_scale[] is the other live copy of this number.
 */
/*
 * Drive direction against the URDF, measured 2026-11-21 by hand on the
 * assembled robot. See robot_config.h for the convention.
 *
 *   left  hip pitch  count rises as the leg swings back       +1
 *   left  hip roll   count rises as the foot moves outward    +1 (= robot left)
 *   left  knee       count rises as the shin swings back      +1
 *   right hip pitch  count rises as the leg swings FORWARD    -1
 *   right hip roll   count rises as the foot moves inward     +1 (= robot left)
 *   right knee       count rises as the shin swings back      +1
 *   left  ankle      count rises as the toes go down          +1
 *   right ankle      count rises as the toes go UP            -1  (2026-11-26)
 *
 * The right ankle was +1 until 2026-11-26. That was measured on its LOAD-side
 * encoder; the drive moved to its commutation encoder the same day and the
 * direction was never re-measured, as this comment warned. It counts the
 * other way through the box. On the robot, both feet flat, ankles backdriven
 * by hand - where both ankles must turn the same way:
 *   - rocked fore and aft: left -13.5, right +13.8 deg at the same instant;
 *   - tipped FORWARD and held: left -28.8 (toes up against the shin, which
 *     leaning forward on planted feet must give; toes-down is + in the URDF),
 *     right +28.5.
 * So every walk from 2026-11-22 to 2026-11-26 drove the right ankle as the
 * mirror of what the policy asked - toes up for down - and its reading was
 * mirrored to match, so nothing on the Pi could tell.
 */
const float g_joint_dir[NEXUS_NUM_JOINTS] = {
    /* L hip_pitch   */ +1.0f,
    /* L hip_roll    */ +1.0f,
    /* L knee_pitch  */ +1.0f,
    /* L ankle_pitch */ +1.0f,
    /* R hip_pitch   */ -1.0f,
    /* R hip_roll    */ +1.0f,
    /* R knee_pitch  */ +1.0f,
    /* R ankle_pitch */ -1.0f,   /* was +1 until 2026-11-26, see above */
};

/* Encoder turns per output turn - see robot_config.h. Left ankle only. */
const float g_joint_enc_ratio[NEXUS_NUM_JOINTS] = {
    /* L hip_pitch   */  1.0f,
    /* L hip_roll    */  1.0f,
    /* L knee_pitch  */  1.0f,
    /* L ankle_pitch */ 15.0f,   /* commutation encoder, 15:1 */
    /* R hip_pitch   */  1.0f,
    /* R hip_roll    */  1.0f,
    /* R knee_pitch  */  1.0f,
    /* R ankle_pitch */ 15.0f,   /* commutation encoder, 15:1 (since 2026-11-22) */
};

const float g_joint_gear[NEXUS_NUM_JOINTS] = {
    /* L hip_pitch   */ 47.0f,
    /* L hip_roll    */ 26.0f,
    /* L knee_pitch  */ 47.0f,
    /* L ankle_pitch */ 15.0f,
    /* R hip_pitch   */ 47.0f,
    /* R hip_roll    */ 26.0f,
    /* R knee_pitch  */ 47.0f,
    /* R ankle_pitch */ 15.0f,
};

/*
 * Which joints have a spring between the motor and the limb. Only these get
 * their target pre-deflected, and only these have an AS5047P - which is why
 * NEXUS_NUM_ENCODERS is 4 and not 8.
 */
const uint8_t g_joint_is_sea[NEXUS_NUM_JOINTS] = {
    /* L hip_pitch   */ 1u,
    /* L hip_roll    */ 0u,
    /* L knee_pitch  */ 1u,
    /* L ankle_pitch */ 0u,
    /* R hip_pitch   */ 1u,
    /* R hip_roll    */ 0u,
    /* R knee_pitch  */ 1u,
    /* R ankle_pitch */ 0u,
};

/*
 * Per-joint command envelope. See robot_config.h for how these were derived
 * and what still has to be measured.
 *
 * Policy z1_softland_tol05, gait row v = 0.30 m/s, stride 1.857507 s.
 */
const joint_limit_t g_joint_limit[NEXUS_NUM_JOINTS] = {
    /* L hip_pitch   */ { -0.1107f, +0.1077f },   /* -39.8 .. +38.8 deg */
    /* L hip_roll    */ { -0.0457f, +0.0334f },   /* -16.4 .. +12.0 deg */
    /* L knee_pitch  */ { -0.1009f, +0.1475f },   /* -36.3 .. +53.1 deg */
    /* L ankle_pitch */ { -0.1094f, +0.0424f },   /* -39.4 .. +15.3 deg */
    /* R hip_pitch   */ { -0.1107f, +0.1077f },   /* -39.8 .. +38.8 deg */
    /* R hip_roll    */ { -0.0334f, +0.0457f },   /* -12.0 .. +16.4 deg */
    /* R knee_pitch  */ { -0.1009f, +0.1475f },   /* -36.3 .. +53.1 deg */
    /* R ankle_pitch */ { -0.1094f, +0.0424f },   /* -39.4 .. +15.3 deg */
};

/*
 * Spring zeros, measured 2026-11-22: robot in the URDF zero pose on its
 * supports, legs unloaded, ROBOT mode. The median of ~10 000 reads per sensor,
 * and the same value to the count in every run that day (ENC_SCAN at 1 MHz,
 * SENSORS, and ROBOT before and after glitch rejection), so these are the
 * springs' rest positions rather than one lucky sample.
 *
 * SIGNS, measured 2026-11-23 by hand: drives idle, so the non-backdrivable
 * gearboxes hold the motor side and a push on the foot winds only the spring.
 * Each foot pushed backward (positive hip pitch in the URDF: the leg swings
 * back) and each knee bent (positive knee), then the reverse, ~3 s each.
 * Resting noise 0.002 deg; the pushes read 0.05-0.18 deg.
 *
 *   L hip   +0.046 then -0.085   L knee  +0.107 then -0.127    -> +1
 *   R hip   -0.064 then +0.181   R knee  -0.107 then +0.127    -> -1
 *
 * The right leg is the left one mirrored, and so are its spring magnets: the
 * raw count runs the other way. Cross-check in the same recording: bending
 * the right knee also moved the right hip spring negative, straightening it
 * positive - consistent only with both right signs inverted.
 */
const spring_enc_cal_t g_spring_enc[NEXUS_NUM_ENCODERS] = {
    /* L hip   (PF1)  */ { 13586u, 1.0f },
    /* L knee  (PD15) */ {  9411u, 1.0f },
    /* R hip   (PD14) */ {  7730u, -1.0f },
    /* R knee  (PF5)  */ {  6033u, -1.0f },
};

#define ENC_COUNTS       16384u          /* AS5047P is 14-bit over a full turn */
#define ENC_MASK         (ENC_COUNTS - 1u)
#define ENC_TO_RAD       (6.28318531f / (float)ENC_COUNTS)
#define TWO_PI           6.28318531f
#define PI_F             3.14159265f

float robot_spring_deflection(uint8_t enc_index, uint16_t raw_counts)
{
    if (enc_index >= NEXUS_NUM_ENCODERS)
    {
        return 0.0f;
    }

    /*
     * Subtract the zero in COUNTS, not in radians. Counts are integers and
     * wrap exactly at 16384; doing it in radians first would leave the
     * subtraction to float arithmetic that has already rounded.
     */
    uint16_t rel = (uint16_t)((raw_counts - g_spring_enc[enc_index].zero_counts)
                              & ENC_MASK);

    float a = (float)rel * ENC_TO_RAD;

    /* Fold 0..2pi onto -pi..+pi so a deflection either side of rest is
       continuous through zero instead of jumping a full turn. */
    if (a > PI_F)
    {
        a -= TWO_PI;
    }

    return a * g_spring_enc[enc_index].sign;
}

/*
 * IMU mounting, measured 2026-11-21: chip X left, chip Y back, chip Z up.
 * See robot_config.h.
 *
 *   robot X = -chip Y      robot Y = chip X      robot Z = chip Z
 */
/*
 * The 90 degree yaw (robot_config.h), then a LEVEL correction: the board sits
 * 8.81 deg rolled on the torso. Measured 2026-11-24 from the accelerometer on
 * six separate occasions with every joint within 1 deg of zero, both foot
 * switches pressed and the robot still (zeus_20261124_200738.flt): gravity
 * read (0.022, -1.487, 9.598) m/s^2, i.e. roll -8.7 to -9.0 every time, pitch
 * scattered about 0 (mean -0.13, not corrected). The builder confirmed the
 * robot stands straight there. Uncorrected, the tilt guard started every run
 * 9 deg into its 20, and the policy - tested in sim to +-3 deg of IMU mount
 * error - would have been told it was falling sideways.
 *
 *     R = Rx(-8.805 deg) * Rz(+90 deg)
 *
 * RE-LEVELLED 2026-11-26: standing straight at the zero pose, both feet down,
 * gravity through that table read (0.419, -1.376, 9.876) - another 7.93 deg
 * of roll. The board has moved on its mount (several falls in between): it
 * is LOOSE, and must be fixed rigidly and re-levelled; a tilt reading that
 * drifts is a policy steering on a lie. Roll only, as before; pitch read
 * -2.4, consistent with the ankles at +2.5 deg.
 *
 *     R = Rx(-7.932 deg) * Rx(-8.805 deg) * Rz(+90 deg)
 *
 * MEASURED IN FULL 2026-11-26 (night), replacing the roll-only corrections
 * above. Two positions, ankles backdriven, both feet flat:
 *   1. standing straight, every joint within 0.6 deg of zero, still:
 *        chip gravity (-2.467, -0.921, 9.545) m/s^2            -> robot +Z
 *   2. the whole robot tipped FORWARD on its feet and held (left ankle -28.8):
 *        chip gravity (-2.114, 3.401, 8.797) m/s^2            -> 25.96 deg
 *   With no ankle roll a robot on flat feet can only pitch, so (2) fixes
 *   forward exactly. Z from (1); X from the part of (2) at right angles to
 *   it (nose down: world up leans to the robot's back); Y = Z x X.
 * Chip X -> robot (-0.012, 0.968, -0.249), chip Y -> (-0.995, -0.036,
 * -0.093): X left, Y back, as the board's markings say, tilted about 15 deg - 6.2 deg from
 * the roll-only table, mostly the pitch that table never corrected. The
 * board did NOT move that evening: straight read the same at 23:01 and 23:08
 * to 0.03 deg.
 *
 * MEASURING IT AGAIN: imu_accel in the state packet is ALREADY rotated by
 * this table (imu_bno085.c, to_body). Undo it - chip = R^T * imu_accel -
 * before building a new one. Treating it as chip data on 2026-11-26 produced
 * a "21.5 deg" tilt that was this table applied twice.
 *
 * RE-MOUNTED AND MEASURED 2026-11-28 - this table replaces the temporary
 * 2026-11-27 offset. The board was re-fixed rigidly (the cause of three days
 * of drifting level) and turned: chip X now points to the robot's RIGHT,
 * Y FORWARD, Z up. The previous table would have read a forward lean as
 * BACKWARD (-15.7 deg for this +15.9) - the policy would have pushed the
 * wrong way from the first step. Two positions, ankles backdriven, both
 * feet down, each held still (gyro < 2 deg/s); chip gravity = the packet's
 * imu_accel with the flashed table undone:
 *   1. standing straight:       (-0.555, -0.028, 9.768) m/s^2  -> robot +Z
 *   2. tipped FORWARD, held:    (-0.701, -2.724, 9.431) m/s^2  -> 15.93 deg
 *      both ankles -19.4 / -18.2 deg (toes up against the shin, together);
 *      the legs put the torso at +17.2 deg on both sides, 1.3 deg more than
 *      the IMU - the feet settling into the mat.
 * Chip X -> robot (0.061, -0.997, -0.057), Y -> (0.998, 0.061, -0.003),
 * Z -> (0.006, -0.056, 0.998): within 5 deg of right / forward / up.
 *
 * LEVELLED 20261128_191817 against a level on the torso at 0 (zeus imu-level,
 * tools/imu_level.py): the old table read that as pitch -4.39, roll +2.36
 * deg; turned by the smallest rotation that reads it level. Chip gravity
 * then: (-0.9075, 0.7447, 9.7106) m/s^2.
 */
const float g_imu_mount_R[9] = {
    +0.0667540f, +0.9953046f, -0.0700915f,
    -0.9934464f, +0.0597683f, -0.0974273f,
    -0.0927806f, +0.0761357f, +0.9927714f,
};

/* R_chip_body = R^T as (w, x, y, z). test_robot_config.c checks it against
   the matrix. */
const float g_imu_mount_q[4] = { +0.7278897f, -0.0596117f, -0.0077928f, +0.6830537f };
