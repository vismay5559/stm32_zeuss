#include "robot_config.h"

#include <math.h>

/*
 * Joint index = bus * 5 + (node - 1), with gait_ref.h's node layout:
 *
 *   node 1 hip_roll   node 2 hip_pitch   node 3 knee   node 4 ankle
 *
 * Left leg is bus 0 (indices 0..4), right leg bus 1 (indices 5..9).
 *
 * Every sign is +1 and every offset 0, which is another way of writing "not
 * measured". Both matter: the sign says whether the drive's positive
 * direction agrees with the FK convention (Z-up, pitch about +Y, roll about
 * +X), and the offset says where the drive's homing position sits relative to
 * the pose FK calls zero.
 */
const joint_src_t g_leg_joints[2][KIN_LEG_JOINTS] = {
    /* --- left leg, bus 0 --- */
    {
        /* hip_pitch   */ { 1u, 1.0f, 0.0f },   /* node 2 */
        /* hip_roll    */ { 0u, 1.0f, 0.0f },   /* node 1 */
        /* knee_pitch  */ { 2u, 1.0f, 0.0f },   /* node 3 */
        /* ankle_pitch */ { 3u, 1.0f, 0.0f },   /* node 4 */
    },
    /* --- right leg, bus 1 --- */
    {
        /* hip_pitch   */ { 6u, 1.0f, 0.0f },   /* node 2 */
        /* hip_roll    */ { 5u, 1.0f, 0.0f },   /* node 1 */
        /* knee_pitch  */ { 7u, 1.0f, 0.0f },   /* node 3 */
        /* ankle_pitch */ { 8u, 1.0f, 0.0f },   /* node 4 */
    },
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

#define ENC_COUNTS       16384u          /* AS5048A is 14-bit over a full turn */
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
