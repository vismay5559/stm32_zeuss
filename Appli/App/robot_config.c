#include "robot_config.h"

#include <math.h>

/*
 * Which telemetry index feeds each FK slot. The indices are the NEXUS_J_*
 * joint map in link_proto.h - node 1 hip_pitch, node 2 hip_roll, node 3 knee,
 * node 4 ankle, left leg on bus 0 and right on bus 1. Node 5 on each bus is a
 * waist joint (g_waist_joints below).
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

const joint_src_t g_waist_joints[2] = {
    /* pitch */ { NEXUS_J_WAIST_PITCH, 1.0f, 0.0f },   /* bus 1 node 5 */
    /* roll  */ { NEXUS_J_WAIST_ROLL,  1.0f, 0.0f },   /* bus 0 node 5 */
};

const uint8_t g_leg_springs[2][2] = {
    { NEXUS_ENC_L_HIP_PITCH, NEXUS_ENC_L_KNEE_PITCH },
    { NEXUS_ENC_R_HIP_PITCH, NEXUS_ENC_R_KNEE_PITCH },
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
