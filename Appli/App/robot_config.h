#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>
#include "link_proto.h"
#include "kinematics.h"

/*
 * Everything about THIS robot that the firmware cannot work out for itself.
 *
 * These numbers were previously scattered across three files - link lengths in
 * kinematics.c, the joint source map in fusion.c, nothing at all for the
 * encoders - and every one of them was a placeholder that the estimator then
 * trusted completely. Gathering them here does not make them correct, but it
 * makes them findable, and it lets one flag say whether they have been
 * measured yet.
 *
 * ---------------------------------------------------------------------------
 * NOTHING BELOW HAS BEEN MEASURED ON THE ROBOT.
 *
 * While ROBOT_CONFIG_CALIBRATED is 0 the estimator will not report
 * NEXUS_FUSION_OK, no matter how well its covariance converges. That is
 * deliberate: a converged filter built on guessed geometry is confidently
 * wrong, which is worse than one that admits it does not know. Measure the
 * values, set the flag to 1, and the estimator becomes usable.
 * ---------------------------------------------------------------------------
 */
#define ROBOT_CONFIG_CALIBRATED  0

/* ===================================================================== */
/*  Which drive is which joint                                            */
/* ===================================================================== */

/*
 * Forward kinematics needs four angles per leg, in chain order:
 *
 *     [ hip_pitch, hip_roll, knee_pitch, ankle_pitch ]
 *
 * ALL FOUR COME FROM THE ODRIVES.
 *
 * This is a correction. fusion.c used to take hip_pitch and knee from the
 * AS5048A encoders, but those encoders sit AFTER the series springs and
 * measure spring deflection, not an absolute joint angle - which both the
 * README ("spring_angle is deflection, not a joint angle") and app.c say
 * explicitly. Feeding a 0..2pi raw deflection in as a hip angle put the foot
 * somewhere it had never been, and the contact update then dragged the whole
 * state towards it.
 *
 * The drives report output-shaft position with the gear ratio already applied
 * (see gait_ref.h), which is exactly the joint angle FK wants.
 *
 * ---------------------------------------------------------------------------
 * THE INDICES BELOW ARE NOT CONFIRMED.
 *
 * Two places in this repo describe the node layout and they disagree.
 * gait_ref.h - generated from the real drive configuration - says
 *
 *     node 1 = hip_roll   node 2 = hip_pitch   node 3 = knee   node 4 = ankle
 *
 * which, with joint index = bus * 5 + (node - 1), gives the table below. The
 * old map in fusion.c instead had hip_roll at index 1, where gait_ref.h puts
 * hip_pitch. gait_ref.h is followed here because it is generated rather than
 * hand-written, but ONE OF THEM IS WRONG and only the robot can say which.
 *
 * Getting this wrong produces a foot position that is confidently wrong, so it
 * is gated behind ROBOT_CONFIG_CALIBRATED with everything else.
 * ---------------------------------------------------------------------------
 */

typedef struct
{
    uint8_t act_index;   /* index into act_telemetry_t.pos                */
    float   sign;        /* +1 or -1, to match the FK sign convention     */
    float   offset;      /* radians added after the sign: the joint zero  */
} joint_src_t;

/* Chain-order slots into the tables below. */
#define ROBOT_JOINT_HIP_PITCH    0
#define ROBOT_JOINT_HIP_ROLL     1
#define ROBOT_JOINT_KNEE         2
#define ROBOT_JOINT_ANKLE        3

extern const joint_src_t g_leg_joints[2][KIN_LEG_JOINTS];   /* [0]=left [1]=right */

/* ===================================================================== */
/*  Spring encoders                                                       */
/* ===================================================================== */

/*
 * The AS5048A is 14-bit over a full turn and reports an absolute 0..2pi
 * angle. Spring deflection is a small SIGNED quantity either side of a
 * mechanical zero, so the raw reading has to be referenced and wrapped:
 *
 *     deflection = wrap_pi((raw - zero) * 2pi / 16384 * sign)
 *
 * Without the zero, a joint whose rest position happens to sit near the
 * wrap point jumps by a full turn between two adjacent ticks - and since the
 * Pi computes torque as deflection x spring constant, that is a 6.28 rad
 * step straight into the torque estimate.
 *
 * TO MEASURE: unload the leg so the spring is at rest, read the raw counts
 * off the console, and put them here. One reading per encoder.
 */
typedef struct
{
    uint16_t zero_counts;   /* raw AS5048A reading at zero deflection */
    float    sign;          /* +1 or -1, so positive means wind-up     */
} spring_enc_cal_t;

extern const spring_enc_cal_t g_spring_enc[NEXUS_NUM_ENCODERS];

/* ===================================================================== */
/*  Geometry                                                              */
/* ===================================================================== */

/*
 * Link lengths and hip offsets. Previously hard-coded in kin_defaults() as
 * suspiciously round numbers - 0.30 / 0.30 / 0.05 - which is what a
 * placeholder looks like. TO MEASURE: hip pivot to knee pivot, knee pivot to
 * ankle pivot, ankle pivot to the sole's contact point, and the lateral
 * offset from the IMU/body origin to each hip.
 */
#define ROBOT_THIGH_LENGTH_M    0.30f
#define ROBOT_SHANK_LENGTH_M    0.30f
#define ROBOT_FOOT_HEIGHT_M     0.05f
#define ROBOT_HIP_OFFSET_Y_M    0.05f

/* Convenience for anything that wants to say "this is not trustworthy yet". */
static inline uint8_t robot_config_is_calibrated(void)
{
    return (uint8_t)ROBOT_CONFIG_CALIBRATED;
}

/* Raw encoder counts -> signed spring deflection in radians. */
float robot_spring_deflection(uint8_t enc_index, uint16_t raw_counts);

#endif /* ROBOT_CONFIG_H */
