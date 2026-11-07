#include "robot_config.h"

#include <math.h>

/*
 * Which telemetry index feeds each FK slot. The indices are the NEXUS_J_*
 * joint map in link_proto.h - node 1 hip_pitch, node 2 hip_roll, node 3 knee,
 * node 4 ankle, left leg on bus 0 and right on bus 1. Node 5 on each bus would
 * be a waist joint; this build has none.
 *
 * Every sign is +1 and every offset 0, which is another way of writing "not
 * measured". Both matter: the sign says whether the drive's positive
 * direction agrees with the URDF's convention (pitch about +Y, roll about
 * +X), and the offset says where the drive's homing position sits relative to
 * the URDF's zero pose.
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
 *     hip roll                26:1   encoder on the load - ASSUMED, not measured
 *     ankle pitch             15:1   encoder on the MOTOR
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
const drive_gains_t g_drive_gains[NEXUS_NUM_JOINTS] = {
    /* L hip_pitch   */ { 30.0f, 10.0f, 50.0f },
    /* L hip_roll    */ { 30.0f, 10.0f, 50.0f },
    /* L knee_pitch  */ { 30.0f, 10.0f, 50.0f },
    /* L ankle_pitch */ { 17.0f,  0.3f,  1.5f },
    /* R hip_pitch   */ { 30.0f, 10.0f, 50.0f },
    /* R hip_roll    */ { 30.0f, 10.0f, 50.0f },
    /* R knee_pitch  */ { 30.0f, 10.0f, 50.0f },
    /* R ankle_pitch */ { 17.0f,  0.3f,  1.5f },
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
 * Zero at count 0 means "no zero has been measured" - the deflection is then
 * just the raw angle wrapped into +/-pi, which is at least signed and
 * continuous rather than a 0..2pi ramp with a cliff in it.
 */
const spring_enc_cal_t g_spring_enc[NEXUS_NUM_ENCODERS] = {
    { 0u, 1.0f },
    { 0u, 1.0f },
    { 0u, 1.0f },
    { 0u, 1.0f },
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
