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
 * The policy trained with KP/KD on the OUTPUT shaft, per joint class:
 *
 *     class                     KP      KD     gear   encoder on
 *     hip pitch, knee          150    12.0     47:1   load
 *     hip roll                  60     4.0     26:1   load
 *     ankle pitch, ankle roll   30     1.2     15:1   MOTOR
 *
 * The ODrive's own law is  tau_motor = vel_gain * vel_err  and
 * vel_setpoint = pos_gain * pos_err, both in turns of WHATEVER SHAFT THAT
 * DRIVE'S ENCODER SITS ON, and its torque is always MOTOR torque. Converting:
 *
 *     pos_gain = KP / KD                      always - see below
 *     vel_gain = KD * 2pi / N                 encoder on the load
 *     vel_gain = KD * 2pi / N^2               encoder on the MOTOR
 *
 *     class                     pos_gain   vel_gain   vel_int
 *     hip pitch, knee              12.5     1.6042        0
 *     hip roll                     15.0     0.9666        0
 *     ankle pitch, ankle roll      25.0     0.0335        0
 *
 * pos_gain is KP/KD whichever shaft the encoder is on, because it is a ratio
 * of encoder units to encoder units per second - the gearbox cancels. vel_gain
 * does NOT cancel: it carries motor Nm on one side and encoder turns/s on the
 * other, so the ratio enters once for the torque and again, for the ankle
 * only, because that encoder is before the gearbox. Hence 1/N^2 and an ankle
 * vel_gain of 0.03 rather than 7.54.
 *
 * vel_integrator_gain is 0 because training had no integrator at all. The
 * 50.0f below is not a smaller version of the trained gain, it is a term that
 * did not exist. Simulation survives x0.7 to x2.0 of the trained stiffness and
 * falls at x0.5.
 *
 * Torque feedforward follows the same rule: g_gait_tau is output Nm, so what
 * goes into Torque_FF is g_gait_tau / N. Peaks become 0.29 Nm at hip pitch,
 * 0.94 at hip roll, 0.23 at knee and 0.43 at the ankles. Nowhere near the
 * int16 saturation at +/-32.767 Nm that mattered when these were thought to be
 * output Nm - the live concern is now RESOLUTION, and 1 mNm against a 0.29 Nm
 * peak is about 0.3%, which is fine.
 *
 * STILL TO CONFIRM PER DRIVE: which shaft each encoder is on. The table above
 * takes hip pitch, knee and ankle from docs/ODRIVE_COMMANDS.md, which was
 * written for the single-leg bench test - and HIP ROLL IS NOT IN IT, so its
 * "load" is an assumption. Note too that robot_config.h above claims every
 * drive reports output-shaft position with the ratio already applied, which
 * contradicts that document's MOTOR entry for the ankle. One of the two is
 * wrong and the drives themselves are the authority. Get this right before
 * typing the table in: a vel_gain wrong by N^2 is a factor of 225.
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
