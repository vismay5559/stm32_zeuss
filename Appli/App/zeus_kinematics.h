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
 * Each leg has two contact points, toe and heel, one per contact switch.
 * zeus_kin_foot() returns both: they share every joint, so the second costs a
 * few multiplications.
 *
 * q, per leg, in ZEUS_KIN_Q_* order (rad):
 *
 *   - hip pitch and knee are series-elastic. The drive reports the motor side
 *     (HIP_PITCH, KNEE_PITCH), the AS5048A reports the spring's deflection
 *     (the *_SPRING entries), and the leg's real angle is their sum. Passing 0
 *     for a spring treats it as rigid.
 *   - the waist joints move the torso, and the IMU bolted to it, relative to
 *     both legs, so they are part of each leg's chain.
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
    ZEUS_KIN_Q_WAIST_PITCH       = 6,   /* the same value for both legs  */
    ZEUS_KIN_Q_WAIST_ROLL        = 7,   /* the same value for both legs  */
    ZEUS_KIN_NQ                  = 8
};

typedef enum
{
    ZEUS_KIN_LEFT  = 0,
    ZEUS_KIN_RIGHT = 1
} zeus_kin_side_t;

#define ZEUS_KIN_TOE        0
#define ZEUS_KIN_HEEL       1
#define ZEUS_KIN_POINTS     2

/* The NEXUS_CONTACT_* index of a point: L_TOE 0, L_HEEL 1, R_TOE 2, R_HEEL 3. */
#define ZEUS_KIN_CONTACT(side, point)   ((int)(side) * ZEUS_KIN_POINTS + (int)(point))

/*
 * side  ZEUS_KIN_LEFT or ZEUS_KIN_RIGHT
 * q     ZEUS_KIN_NQ joint angles, ZEUS_KIN_Q_* order                 (rad)
 * p     p[ZEUS_KIN_TOE], p[ZEUS_KIN_HEEL]: position in the IMU frame  (m)
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
