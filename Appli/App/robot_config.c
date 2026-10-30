#include "robot_config.h"

#include <math.h>

/*
 * Which telemetry index feeds each FK slot. The indices are the NEXUS_J_*
 * joint map in link_proto.h - node 1 hip_pitch, node 2 hip_roll, node 3 knee,
 * node 4 ankle_pitch, node 5 ankle_roll, left leg on bus 0 and right on bus 1.
 * Node 5 was the waist on an older robot, which this one does not have.
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
        /* ankle_roll  */ { NEXUS_J_L_ANKLE_ROLL,  1.0f, 0.0f },   /* node 5 */
    },
    /* --- right leg, bus 1 --- */
    {
        /* hip_pitch   */ { NEXUS_J_R_HIP_PITCH,   1.0f, 0.0f },   /* node 1 */
        /* hip_roll    */ { NEXUS_J_R_HIP_ROLL,    1.0f, 0.0f },   /* node 2 */
        /* knee_pitch  */ { NEXUS_J_R_KNEE_PITCH,  1.0f, 0.0f },   /* node 3 */
        /* ankle_pitch */ { NEXUS_J_R_ANKLE_PITCH, 1.0f, 0.0f },   /* node 4 */
        /* ankle_roll  */ { NEXUS_J_R_ANKLE_ROLL,  1.0f, 0.0f },   /* node 5 */
    },
};

/*
 * Starting gains, per joint, in the NEXUS_J_* order.
 *
 * THESE ARE STILL THE BENCH LEG'S, AND THEY ARE NOT WHAT THE POLICY TRAINED
 * AGAINST. They are here so a freshly flashed drive comes up with something
 * sane; the walk needs the numbers below instead.
 *
 * The policy's handoff (firmware_handoff/FIRMWARE_CHANGES.md item 5) gives the
 * trained stiffness, converted to the ODrive's own units on the OUTPUT shaft:
 *
 *     joint class                 KP      KD    pos_gain  vel_gain
 *     hip pitch, knee (47:1)     150      12      12.5      75.4
 *     hip roll (26:1)             60       4      15.0      25.1
 *     ankle pitch, ankle roll     30     1.2      25.0      7.54
 *
 * with vel_integrator_gain 0 - training had no integrator at all, so the 50.0f
 * below is not a smaller version of the trained gain, it is a term that did not
 * exist. Simulation survives between x0.7 and x2.0 of the trained stiffness and
 * falls at x0.5, and today's hip value is about x0.32 of it.
 *
 * WHY THEY ARE NOT TYPED IN YET: the conversion assumes the drive's vel_gain
 * and Torque_FF are in OUTPUT Nm. If a drive reports motor-side torque instead,
 * both divide by that joint's gear ratio - a factor of 47 on the hip pitch -
 * and a gain wrong by 47x on a leg carrying the robot is not a tuning problem.
 * Confirm per drive which shaft its torque and its encoder refer to, then
 * replace this table wholesale.
 *
 * The ankle numbers below are worse than untuned: they were measured on a 9:1
 * ankle and this robot's ankles are 15:1. Ankle roll simply repeats ankle
 * pitch, which is also what the simulation assumed for it.
 */
const drive_gains_t g_drive_gains[NEXUS_NUM_JOINTS] = {
    /* L hip_pitch   */ { 30.0f, 10.0f, 50.0f },
    /* L hip_roll    */ { 30.0f, 10.0f, 50.0f },
    /* L knee_pitch  */ { 30.0f, 10.0f, 50.0f },
    /* L ankle_pitch */ { 17.0f,  0.3f,  1.5f },
    /* L ankle_roll  */ { 17.0f,  0.3f,  1.5f },
    /* R hip_pitch   */ { 30.0f, 10.0f, 50.0f },
    /* R hip_roll    */ { 30.0f, 10.0f, 50.0f },
    /* R knee_pitch  */ { 30.0f, 10.0f, 50.0f },
    /* R ankle_pitch */ { 17.0f,  0.3f,  1.5f },
    /* R ankle_roll  */ { 17.0f,  0.3f,  1.5f },
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
