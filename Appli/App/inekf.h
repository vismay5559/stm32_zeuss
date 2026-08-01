#ifndef INEKF_H
#define INEKF_H

#include "lie_group.h"
#include "kinematics.h"

/*
 * Contact-aided right-invariant EKF.
 *
 * Port of zeus_sensor_fusion/inekf.py. Estimates body orientation, velocity,
 * position and IMU biases by fusing the IMU with leg forward kinematics while
 * feet are in contact with the ground.
 *
 * State, following Hartley et al. 2019:
 *   X     in SE_{N+2}(3)   -  R, v, p and one world position per active contact
 *   theta in R^6           -  IMU biases [b_g; b_a]
 *   P                      -  right-invariant error covariance, 3(N+3)+6
 *
 * Rather than the dense (5+N)x(5+N) matrix the Python keeps, the components of
 * X are stored separately: the bottom rows of that matrix are always identity,
 * so carrying them buys nothing and costs a lot of multiply-by-zero.
 *
 * Contacts are a fixed pair of slots (left, right) with an active flag rather
 * than a resizing matrix. A slot that is inactive has its rows and columns in
 * P zeroed, which makes it inert in every product.
 */

#define INEKF_CONTACT_LEFT    0
#define INEKF_CONTACT_RIGHT   1

/* Error-state indices, in the fixed layout described in lie_group.h. */
#define INEKF_IDX_PHI     0
#define INEKF_IDX_V       3
#define INEKF_IDX_P       6
#define INEKF_IDX_D(k)    (9 + 3 * (k))
#define INEKF_IDX_BG      (9 + 3 * INEKF_MAX_CONTACTS)
#define INEKF_IDX_BA      (INEKF_IDX_BG + 3)

typedef struct
{
    /* Process noise, standard deviations. */
    inekf_real_t noise_gyro;          /* rad/s     */
    inekf_real_t noise_accel;         /* m/s^2     */
    inekf_real_t noise_gyro_bias;     /* rad/s^2   */
    inekf_real_t noise_accel_bias;    /* m/s^3     */
    inekf_real_t noise_contact_vel;   /* m/s, foot-slip model */
    inekf_real_t noise_encoder;       /* rad       */

    /* Initial covariance, standard deviations. */
    inekf_real_t init_orientation;
    inekf_real_t init_velocity;
    inekf_real_t init_position;
    inekf_real_t init_contact;
    inekf_real_t init_gyro_bias;
    inekf_real_t init_accel_bias;

    inekf_real_t gravity[3];          /* world frame, Z-up -> {0,0,-9.81} */
} inekf_params_t;

typedef struct
{
    inekf_params_t params;

    /* --- state --- */
    inekf_real_t R[9];                                  /* body -> world      */
    inekf_real_t v[3];                                  /* world velocity     */
    inekf_real_t p[3];                                  /* world position     */
    inekf_real_t d[INEKF_MAX_CONTACTS][3];              /* world contact pos  */
    uint8_t      active[INEKF_MAX_CONTACTS];

    inekf_real_t bg[3];
    inekf_real_t ba[3];

    inekf_real_t P[INEKF_STRIDE * INEKF_STRIDE];

    /* Scratch, kept here rather than on the stack: these are 1.7 kB each and
       the 1 kHz loop runs on the main stack. */
    inekf_real_t Phi[INEKF_STRIDE * INEKF_STRIDE];
    inekf_real_t tmpA[INEKF_STRIDE * INEKF_STRIDE];
    inekf_real_t tmpB[INEKF_STRIDE * INEKF_STRIDE];
} inekf_t;

/* Sensible defaults, matching the Python InEKFParams. */
void inekf_default_params(inekf_params_t *p);

void inekf_init(inekf_t *f, const inekf_params_t *params);
void inekf_reset(inekf_t *f);

/*
 * Propagate with one IMU sample.
 *   omega, accel  raw body-frame gyro (rad/s) and accelerometer (m/s^2)
 *   dt            seconds since the previous call
 */
void inekf_predict(inekf_t *f, const inekf_real_t *omega,
                   const inekf_real_t *accel, inekf_real_t dt);

/*
 * Contact management. B_p_BC is the contact position in the body frame from
 * forward kinematics; J_p is its 3x4 Jacobian w.r.t. that leg's joint angles.
 */
void inekf_add_contact(inekf_t *f, int slot,
                       const inekf_real_t *B_p_BC, const inekf_real_t *J_p);
void inekf_remove_contact(inekf_t *f, int slot);

/* Forward-kinematic measurement update for one active contact. */
void inekf_update_contact(inekf_t *f, int slot,
                          const inekf_real_t *B_p_BC, const inekf_real_t *J_p);

/* --- accessors --- */
inekf_real_t inekf_height(const inekf_t *f);                 /* world z, m   */
void inekf_velocity_world(const inekf_t *f, inekf_real_t *v3);
void inekf_velocity_body(const inekf_t *f, inekf_real_t *v3);
void inekf_quaternion(const inekf_t *f, inekf_real_t *q4);   /* w,x,y,z      */
int  inekf_num_contacts(const inekf_t *f);

#endif /* INEKF_H */
