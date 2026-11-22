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
 *     (node 5, the waist, is not part of this build)
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
 * This is the spring's MECHANICAL TRAVEL. 0.0297 rad is +/-1.7 degrees, which
 * with a stiffness of 3437.75 Nm/rad (60 Nm/deg) is the spring this robot has.
 *
 * It was 0.35f - 20 degrees, and this comment called it "a generous guess". At
 * twelve times the real travel it rejected nothing: a garbled read of 15
 * degrees passed straight through into joint_pos and into the Pi's torque
 * estimate, which is exactly the failure a hip encoder reading nonsense
 * produces. Tightening it immediately caught the walk simulation feeding in
 * 0.06 rad of wind-up, which through this spring would be 206 Nm.
 *
 * STILL TO MEASURE on the bench, because both numbers are modelled rather than
 * weighed: hold the motor, push the limb to the spring's hard stop, read the
 * deflection. If the stop is further out, raise this - but to what was
 * measured, not to a round number.
 */
#define ROBOT_SPRING_MAX_DEFLECTION_RAD   0.0297f

/*
 * Series spring stiffness at the OUTPUT shaft, Nm per radian.
 *
 * Nothing uses it at the moment. It earned its place when the drive command
 * pre-deflected a series-elastic joint's target by tau/K, because the ODrive
 * holds the MOTOR side and the limb sits behind the spring:
 *
 *     limb = motor + tau / K     =>     motor = ref - tau / K
 *
 * That went out with the ten-joint revert, because the gait table this build
 * plays has no torque row to compute tau from. It is kept here because it is a
 * measurement of a physical part, and because the pre-deflection is the right
 * thing to do the moment there is a torque to do it with - see git history at
 * 200d598.
 */
#define ROBOT_SPRING_K_NM_PER_RAD   3437.75f

/*
 * Gear ratio per joint, in NEXUS_J_* order.
 *
 * Needed because the ODrive's Torque_FF is always MOTOR torque while the gait
 * table's torques are at the output, so what goes on the wire is
 * g_gait_tau / g_joint_gear. Getting this wrong by a factor of 47 is not a
 * tuning error.
 *
 * These are the same numbers the policy trained against: zeus_model.py has
 * N_SEA, N_ROLL, N_ANK = 47, 26, 15.
 *
 * NOT applied to position or velocity. This header states above that every
 * drive reports output-shaft position with the ratio already applied, which is
 * how the drives are expected to be configured, so positions pass through 1:1.
 * docs/ODRIVE_COMMANDS.md disagrees for the ankle - it says that encoder sits
 * on the MOTOR side - and if that is what the drive actually does then its
 * position and velocity need this ratio too. The drives are the authority;
 * confirm before the first ankle move.
 */
extern const float g_joint_gear[NEXUS_NUM_JOINTS];

/*
 * Which way each DRIVE counts, against the URDF's convention, in NEXUS_J_*
 * order. +1 when the drive's position increases as the joint moves in its
 * URDF-positive direction, -1 when it decreases.
 *
 * The URDF convention, in the robot's frame (X forward, Y left, Z up):
 *     hip pitch +  leg swings BACKWARD
 *     knee      +  shin swings backward (the knee bends the normal way)
 *     ankle     +  toes DOWN
 *     hip roll  +  foot moves toward the robot's LEFT, on both legs - so
 *                  outward on the left leg and inward on the right
 *
 * Applied in exactly ONE place, act_odrive.c, at the CAN boundary: to
 * position, velocity and torque coming in, and to the position, velocity
 * feedforward and torque feedforward going out. Everything above that - the
 * gait, safety limits, the estimator, the state packet, the policy - works in
 * the URDF's convention and never sees a drive's own direction.
 *
 * That is why g_leg_joints[].sign is +1 everywhere and must stay so: a second
 * sign there would flip the estimator's joint twice.
 *
 * Measured 2026-11-21 by moving each joint by hand and reading the drive. The
 * right hip pitch is the odd one out: its count rises as the leg swings
 * FORWARD. Both ankles count up as the toes go down, as the URDF does.
 */
extern const float g_joint_dir[NEXUS_NUM_JOINTS];

/*
 * Drive encoder turns per OUTPUT turn, in NEXUS_J_* order: 1 where the
 * drive's encoder is on the load side of the gearbox, the gear ratio where it
 * is the motor's own commutation encoder.
 *
 * Every drive on this robot reads the load side EXCEPT the two ankles, which
 * use their commutation encoders through the 15:1 box (stated by the builder,
 * 2026-11-22: the left from the start, the right moved over from a load-side
 * encoder later that day). Unscaled, such a drive reports and is commanded in motor turns:
 * a 10 degree gait command turns the ankle 0.7 degrees, and its angle in the
 * estimator is 15x too large.
 *
 * Applied with g_joint_dir at the same CAN boundary in act_odrive.c: position
 * and velocity divided by it coming in, position and velocity feedforward
 * multiplied by it going out. NOT torque: the drive's torque is motor torque
 * whichever shaft its encoder is on, and g_joint_gear already handles that.
 *
 * Two consequences outside the code. That drive's gains are in motor turns
 * (vel_gain scales with 1/N^2 for the same output damping - see
 * g_drive_gains). And its zero is NOT absolute: a commutation encoder knows
 * one motor revolution, 1/15 of an ankle turn, so the left ankle must be at
 * its zero when the drive powers up.
 */
extern const float g_joint_enc_ratio[NEXUS_NUM_JOINTS];

/*
 * 1 where a joint is series-elastic - a spring between the motor and the limb,
 * and therefore a joint whose target needs pre-deflecting. Hip pitch and knee
 * pitch on each leg; everything else is rigid and gets no correction.
 */
extern const uint8_t g_joint_is_sea[NEXUS_NUM_JOINTS];

/* ===================================================================== */
/*  Command envelope, per joint                                           */
/* ===================================================================== */

/*
 * The furthest each joint may be commanded, in ODrive turns on the OUTPUT
 * side. safety.c refuses a target outside this.
 *
 * It replaced SAFETY_POS_MIN/MAX_TURNS, a pair of scalars at +/-10 turns whose
 * own comment called them "deliberately loose bounds chosen to catch garbage,
 * NOT to describe this robot's real joint travel". At 10 turns they were
 * roughly SEVENTY TIMES wider than anything the policy can ask for, so a
 * diverged policy had a very long way to go before the board objected.
 *
 * DERIVED, not guessed, from what the policy can legally command:
 *
 *   the gait's own reach over the stride (g_gait_turns)
 *   + act_scale, the largest residual the network can emit (0.25 rad on hip
 *     pitch and knee, 0.12 on hip roll, 0.15 on the ankle)
 *   + the spring pre-deflection on the SEA joints, which only ever lowers a
 *     target, so it widens the low side alone
 *   + 2 degrees of margin, so float rounding can never reject a legal command
 *
 * It has to be at least that wide. safety.c treats repeated refusals as a
 * fault in themselves, so an envelope tighter than the policy's own authority
 * would stop a healthy robot - which is the failure its predecessor's comment
 * warned about from the other direction.
 *
 * tools/hosttest/test_robot_config.c checks this table against the compiled
 * gait table, so changing the gait without widening this fails the build's
 * tests rather than the robot.
 *
 * STILL TO MEASURE: the mechanical stops. Where a stop is TIGHTER than the row
 * below, the stop wins and the row should come in to meet it. Note that the
 * knee's high side is +53.1 deg while zeus.urdf models every joint at +/-45 -
 * itself a placeholder ("+/-45 deg on every joint for now"). If the knee
 * really does stop at 45, the policy at full residual can drive it into the
 * stop, and that is worth knowing before the first walk.
 */
typedef struct
{
    float min_turns;
    float max_turns;
} joint_limit_t;

extern const joint_limit_t g_joint_limit[NEXUS_NUM_JOINTS];

/* ===================================================================== */
/*  Drive gains                                                           */
/* ===================================================================== */

/*
 * Each ODrive closes its own position and velocity loops at 8 kHz; this board
 * only sends targets. These are the gains it writes to every drive when it
 * arms, so a drive that has been swapped, reflashed or rebooted comes up with
 * the gains that are in git rather than whatever is saved in that particular
 * drive.
 *
 * The Pi can override them while the robot is disarmed (NEXUS_MSG_GAINS) -
 * that is the tuning loop - but what is here is what the robot arms with after
 * a power cycle. When a value is settled, put it here.
 *
 * Units are the ODrive's own:
 *   pos_gain            (turn/s) per turn of error
 *   vel_gain            Nm per (turn/s) of error
 *   vel_integrator_gain Nm per turn of accumulated error
 *
 * A NEGATIVE value means "leave this drive's own saved value alone", which is
 * how a joint is taken out of the loop while tuning its neighbour.
 *
 * TO MEASURE: every one of these. The starting values are the bench leg's
 * (test_leg_can.c) - hip pitch and knee ran 30 / 10 / 50, the 9:1 ankle 17 /
 * 0.3 / 1.5 - repeated for both legs and guessed for hip roll. Tune them with
 * the robot on a stand, one joint at a time: see the README.
 */
#define ROBOT_GAIN_KEEP   (-1.0f)

typedef struct
{
    float pos_gain;
    float vel_gain;
    float vel_int_gain;
} drive_gains_t;

extern const drive_gains_t g_drive_gains[NEXUS_NUM_JOINTS];

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

/* ===================================================================== */
/*  IMU mounting                                                          */
/* ===================================================================== */

/*
 * How the BNO085's own axes sit on the robot. The estimator, the state packet
 * and the policy all work in the robot's frame - X forward, Y left, Z up - so
 * the driver rotates every reading into it once, as it arrives, and nothing
 * downstream ever sees the chip's axes.
 *
 * g_imu_mount_R is row-major R_body_chip: v_robot = R * v_chip.
 * g_imu_mount_q is the SAME rotation's inverse as a quaternion (w,x,y,z),
 * R_chip_body, for re-expressing the sensor's own orientation output:
 * q_world_body = q_world_chip * q_chip_body. test_robot_config.c checks the
 * two agree, because a matrix and a quaternion typed by hand are two chances
 * to get one sign wrong.
 *
 * Measured 2026-11-21, by reading the axis markings on the mounted board:
 *     chip X -> robot LEFT  (+Y)
 *     chip Y -> robot BACK  (-X)
 *     chip Z -> robot UP    (+Z)
 * i.e. the chip is yawed +90 degrees about vertical, which is also how the CAD
 * export models the board. The URDF keeps imu_link robot-aligned on purpose
 * (zeus_model.yaml explains why): this table is where the 90 degrees lives.
 *
 * TO VERIFY on the robot: stand it level and still - accel must read about
 * (0, 0, +9.8). Tilt it nose-down - gyro Y must go positive, and accel X
 * positive. Roll it to its left side down - gyro X must go negative.
 */
extern const float g_imu_mount_R[9];
extern const float g_imu_mount_q[4];

#endif /* ROBOT_CONFIG_H */
