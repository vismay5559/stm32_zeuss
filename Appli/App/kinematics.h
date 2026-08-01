#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "lie_group.h"

/*
 * Leg forward kinematics for the contact-aided InEKF.
 *
 * Port of zeus_sensor_fusion/kinematics.py. Gives the position of a foot
 * contact point in the body/IMU frame, plus the Jacobian of that position with
 * respect to the four leg joint angles - which is what the filter needs to
 * turn encoder noise into a measurement covariance.
 *
 * Chain, per leg:
 *   body -> hip offset -> hip_pitch(Y) -> hip_roll(X) -> thigh(-Z)
 *        -> knee_pitch(Y) -> shank(-Z) -> ankle_pitch(Y) -> foot(-Z)
 *
 * Z-up, right-handed. Pitch joints rotate about +Y, hip roll about +X.
 */

#define KIN_LEG_JOINTS   4      /* hip pitch, hip roll, knee pitch, ankle pitch */

typedef struct
{
    inekf_real_t thigh_length;      /* hip to knee    (m) */
    inekf_real_t shank_length;      /* knee to ankle  (m) */
    inekf_real_t foot_height;       /* ankle to contact point (m) */
    inekf_real_t left_hip_offset[3];
    inekf_real_t right_hip_offset[3];
} kin_params_t;

/* Fills in the defaults from the Python KinematicsParams dataclass. */
void kin_defaults(kin_params_t *p);

/*
 * Foot contact position in the body frame, and its 3x4 Jacobian.
 *
 *   q       four joint angles, in chain order:
 *           [hip_pitch, hip_roll, knee_pitch, ankle_pitch]  (rad)
 *   p_out   contact position in body frame                  (m, 3)
 *   J_out   d(p)/d(q), row-major 3x4                        (m/rad)
 *
 * Pass the matching hip offset from kin_params_t for the leg in question.
 */
void kin_foot(const kin_params_t *params,
              const inekf_real_t *hip_offset,
              const inekf_real_t *q,
              inekf_real_t *p_out,
              inekf_real_t *J_out);

#endif /* KINEMATICS_H */
