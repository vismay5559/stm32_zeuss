#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>
#include "link_proto.h"

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
 * The leg GEOMETRY is no longer here: it comes from the robot's URDF, via
 * tools/gen_kinematics.py into zeus_kinematics_model.h. What is left is what a
 * CAD model cannot know - which way each sensor counts and where its zero is.
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
 * Forward kinematics (zeus_kinematics.h) needs eight angles per leg:
 *
 *   hip_pitch, hip_roll, knee_pitch, ankle_pitch   from that leg's ODrives
 *   hip_pitch_spring, knee_pitch_spring            from that leg's AS5047Ps
 *   waist_pitch, waist_roll                        from the waist ODrives
 *
 * A series-elastic joint's real angle is the motor side PLUS the spring's
 * deflection: the ODrive's encoder sits after the gearbox but before the
 * spring, and the AS5047P measures only how far the spring has wound up. The
 * kinematics model has them as two joints on one axis, which is that sum.
 *
 * (An older version took hip_pitch and knee from the AS5047Ps INSTEAD of the
 * drives, reading a deflection as a joint angle and putting the foot somewhere
 * it had never been. The fix after that dropped the springs altogether, which
 * is off by the deflection - a few degrees under load, centimetres at the
 * foot. Both halves are needed.)
 *
 * The drives report output-shaft position with the gear ratio already applied
 * (see gait_ref.h).
 *
 * ---------------------------------------------------------------------------
 * WHICH INDEX IS WHICH JOINT is defined once, in link_proto.h (NEXUS_J_*,
 * NEXUS_ENC_*), and confirmed against the wiring:
 *
 *     node 1 hip_pitch   node 2 hip_roll   node 3 knee   node 4 ankle
 *     node 5 waist       (roll on bus 0, pitch on bus 1)
 *
 * What is still NOT measured is every sign and offset in these tables. The
 * convention they must match is the URDF's: every pitch joint positive about
 * the robot's +Y (left), every roll joint positive about +X (forward), zero
 * where the URDF was exported. TO MEASURE: put the robot in the URDF's zero
 * pose, read each drive, and store the negated reading as the offset; then
 * move each joint by hand in its positive direction and flip the sign if the
 * reading went the other way.
 * ---------------------------------------------------------------------------
 */

typedef struct
{
    uint8_t act_index;   /* index into act_telemetry_t.pos                */
    float   sign;        /* +1 or -1, to match the URDF's sign convention */
    float   offset;      /* radians added after the sign: the joint zero  */
} joint_src_t;

/* Slots in g_leg_joints[leg][], same order as ZEUS_KIN_Q_* 0..3. */
#define ROBOT_JOINT_HIP_PITCH    0
#define ROBOT_JOINT_HIP_ROLL     1
#define ROBOT_JOINT_KNEE         2
#define ROBOT_JOINT_ANKLE        3
#define ROBOT_LEG_MOTORS         4

extern const joint_src_t g_leg_joints[2][ROBOT_LEG_MOTORS];   /* [0]=left [1]=right */

/* The two waist joints, shared by both legs' chains. */
#define ROBOT_WAIST_PITCH        0
#define ROBOT_WAIST_ROLL         1

extern const joint_src_t g_waist_joints[2];

/* Each leg's spring encoders, NEXUS_ENC_* indices: [leg][0] hip, [leg][1] knee. */
#define ROBOT_SPRING_HIP         0
#define ROBOT_SPRING_KNEE        1

extern const uint8_t g_leg_springs[2][2];

/*
 * The largest deflection a spring can physically reach. A reading beyond it is
 * not a deflection: an unmeasured zero (the raw angle of wherever the magnet
 * happens to sit), a slipped magnet, or a garbled read. The estimator treats
 * such a reading as unknown rather than bending the leg by it.
 *
 * TO MEASURE: the spring's mechanical travel. 20 degrees is a generous guess.
 */
#define ROBOT_SPRING_MAX_DEFLECTION_RAD   0.35f

/* ===================================================================== */
/*  Spring encoders                                                       */
/* ===================================================================== */

/*
 * The AS5047P is 14-bit over a full turn and reports an absolute 0..2pi
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
 * off the console, and put them here. One reading per encoder. Then hold the
 * motor in position and push the link in the joint's positive direction: the
 * deflection must read positive, or flip the sign.
 */
typedef struct
{
    uint16_t zero_counts;   /* raw AS5047P reading at zero deflection          */
    float    sign;          /* +1 or -1: positive when the link has turned
                               further in its joint's positive direction than
                               the motor side, so link = motor + deflection   */
} spring_enc_cal_t;

extern const spring_enc_cal_t g_spring_enc[NEXUS_NUM_ENCODERS];

/* Convenience for anything that wants to say "this is not trustworthy yet". */
/*
 * Has this robot actually been measured?
 *
 * The numbers in this file start as design values - what the drawings say the
 * robot should be. A real machine differs: springs are not exactly as stiff
 * as specified, and sensors are not mounted at exactly zero.
 *
 * Returns 0 until someone has measured a particular robot and filled in the
 * real figures. While it returns 0 the position estimate deliberately never
 * reports itself as fully trustworthy, because it is working from assumptions
 * rather than measurements.
 */
static inline uint8_t robot_config_is_calibrated(void)
{
    return (uint8_t)ROBOT_CONFIG_CALIBRATED;
}

/* Raw encoder counts -> signed spring deflection in radians. */
/*
 * Turn one raw spring-sensor reading into how far that spring is actually
 * squashed, in radians.
 *
 * The sensor reports a plain count that means nothing on its own. This
 * subtracts where that sensor sits when the spring is relaxed, converts to a
 * real angle, and handles the wrap-around when a reading crosses the point
 * where the count rolls over from its highest value back to zero.
 *
 * Multiply the result by the spring's stiffness and you have the force the
 * leg is pushing with, which is the whole reason these sensors are fitted.
 *
 * An out-of-range sensor number returns zero rather than reading memory that
 * does not belong to it.
 */
float robot_spring_deflection(uint8_t enc_index, uint16_t raw_counts);

#endif /* ROBOT_CONFIG_H */
