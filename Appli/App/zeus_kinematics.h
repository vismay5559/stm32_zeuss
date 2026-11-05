#ifndef ZEUS_KINEMATICS_H
#define ZEUS_KINEMATICS_H

#include <stdint.h>

/*
 * Where each foot contact point is, seen from the IMU, and how it moves with
 * the joints - from the robot's URDF, not from hand-measured lengths.
 *
 * The geometry lives in zeus_kinematics_model.h, which tools/gen_kinematics.py
 * writes from zeus_26/zeus_description/urdf/zeus.urdf (itself made from the
 * Fusion 360 export). This file and zeus_kinematics.c never change when the
 * robot does: re-export, regenerate, rebuild.
 *
 * Everything is in imu_link - the IMU chip's own axes, origin at the chip -
 * which is the body frame the InEKF works in.
 *
 * Each leg has ONE contact point: the mechanical switch at the centre of the
 * sole. There used to be two, at the toe and the heel. With a single switch
 * the firmware cannot tell which part of the sole is loaded, so the point it
 * reports is the middle of the foot - the least biased place to pin the
 * estimator when that foot is down. Either edge would be a position the robot
 * only genuinely stands on for part of the stride.
 *
 * q, per leg, in ZEUS_KIN_Q_* order (rad):
 *
 *   - hip pitch and knee are series-elastic. The drive reports the motor side
 *     (HIP_PITCH, KNEE_PITCH), the AS5047P reports the spring's deflection
 *     (the *_SPRING entries), and the leg's real angle is their sum. Passing 0
 *     for a spring treats it as rigid.
 *   - there are no waist joints. Earlier models put two between the torso and
 *     the hips, held at zero because the waist was bolted, and they stayed in
 *     the chain because the IMU sits above them. The current model bolts both
 *     hips straight to the torso, so there is nothing between the IMU and the
 *     hips to carry an angle at all.
 *
 * tools/gen_kinematics.py checks this enum against its own joint list, and
 * tools/hosttest/test_zeus_kinematics.c checks the results against Pinocchio.
 */

enum
{
    ZEUS_KIN_Q_HIP_PITCH         = 0,   /* motor side                    */
    ZEUS_KIN_Q_HIP_ROLL          = 1,
    ZEUS_KIN_Q_KNEE_PITCH        = 2,   /* motor side                    */
    ZEUS_KIN_Q_ANKLE_PITCH       = 3,
    ZEUS_KIN_Q_HIP_PITCH_SPRING  = 4,   /* spring deflection             */
    ZEUS_KIN_Q_KNEE_PITCH_SPRING = 5,   /* spring deflection             */
    ZEUS_KIN_NQ                  = 6
};

typedef enum
{
    ZEUS_KIN_LEFT  = 0,
    ZEUS_KIN_RIGHT = 1
} zeus_kin_side_t;

#define ZEUS_KIN_SOLE       0
#define ZEUS_KIN_POINTS     1

/*
 * The NEXUS_CONTACT_* index of a leg's point: LEFT 0, RIGHT 1.
 *
 * One switch per foot makes this the identity, which is exactly why it is
 * still written down. The two numberings are independent - one is a kinematic
 * chain, the other a wire into a pin - and a previous version of this file
 * quietly assumed they matched when they did not. Keeping the conversion named
 * means the next person to add a point changes one macro, not every caller.
 */
#define ZEUS_KIN_CONTACT(side)          ((int)(side) * ZEUS_KIN_POINTS)

/*
 * side  ZEUS_KIN_LEFT or ZEUS_KIN_RIGHT
 * q     ZEUS_KIN_NQ joint angles, ZEUS_KIN_Q_* order                 (rad)
 * p     p[ZEUS_KIN_SOLE]: position in the IMU frame                  (m)
 * J     J[point][r * ZEUS_KIN_NQ + i] = d p[point][r] / d q[i]       (m/rad)
 *       - the exact derivative, not a finite difference. May be NULL.
 *
 * Returns 0, or -1 for a side that is neither (p and J are then untouched).
 */
int zeus_kin_foot(zeus_kin_side_t side,
                  const float q[ZEUS_KIN_NQ],
                  float p[ZEUS_KIN_POINTS][3],
                  float J[ZEUS_KIN_POINTS][3 * ZEUS_KIN_NQ]);

/* First 16 hex digits of the URDF's sha256: which model this firmware has. */
extern const char zeus_kin_model_sha[];

#endif /* ZEUS_KINEMATICS_H */
