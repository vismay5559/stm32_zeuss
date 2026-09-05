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
/*
 * Fill in sensible starting settings - chiefly how noisy each sensor is
 * assumed to be. Those numbers are how the estimator decides which sensor to
 * believe when two of them disagree, which they constantly do.
 */
void inekf_default_params(inekf_params_t *p);

/*
 * Set the estimator up. Call once before anything else here.
 *
 * It starts out knowing nothing: the robot is assumed upright and still, and
 * the estimator records that it is very unsure about that. Those doubts
 * shrink as real sensor readings arrive.
 */
void inekf_init(inekf_t *f, const inekf_params_t *params);
/*
 * Throw away the current estimate and start over, keeping the settings.
 *
 * For when the estimate has become nonsense - better to admit knowing
 * nothing than to keep building on a wrong answer.
 */
void inekf_reset(inekf_t *f);

/*
 * Propagate with one IMU sample.
 *   omega, accel  raw body-frame gyro (rad/s) and accelerometer (m/s^2)
 *   dt            seconds since the previous call
 */
/*
 * Move the estimate forward in time using the movement sensor. Call once per
 * heartbeat.
 *
 * This is dead reckoning: turning and acceleration are added up to work out
 * where the robot must have got to. It is quick and works anywhere, but small
 * errors accumulate, so an estimate fed only by this drifts away from the
 * truth. The estimator tracks how unsure it is getting, and that uncertainty
 * grows every time this is called without a correction.
 */
void inekf_predict(inekf_t *f, const inekf_real_t *omega,
                   const inekf_real_t *accel, inekf_real_t dt);

/*
 * Contact management. B_p_BC is the contact position in the body frame from
 * forward kinematics; J_p is its 3x4 Jacobian w.r.t. that leg's joint angles.
 */
/*
 * Tell the estimator a foot has just landed, and roughly where it is.
 *
 * From this moment the estimator treats that point on the ground as fixed.
 * That is the anchor everything else is corrected against, so this must only
 * be called when the foot really is planted.
 */
void inekf_add_contact(inekf_t *f, int slot,
                       const inekf_real_t *B_p_BC, const inekf_real_t *J_p);
/*
 * Tell the estimator a foot has lifted. It stops being an anchor, and the
 * estimator no longer has that particular fixed point to correct against.
 */
void inekf_remove_contact(inekf_t *f, int slot);

/* Forward-kinematic measurement update for one active contact. */
/*
 * Correct the estimate using a planted foot. Call once per heartbeat for each
 * foot on the ground.
 *
 * The logic is this: the leg's own joint sensors say where the foot should be
 * relative to the body. The foot has not moved, because it is on the ground.
 * So any disagreement between those two must be error in the estimate of
 * where the BODY is - and it can be corrected.
 *
 * This is what stops the drift that dead reckoning alone produces. A robot
 * with no foot on the ground has nothing to correct against, which is why an
 * estimate degrades while it is in the air.
 */
void inekf_update_contact(inekf_t *f, int slot,
                          const inekf_real_t *B_p_BC, const inekf_real_t *J_p);

/* --- accessors --- */
/* How high the robot's body is above the floor, in metres. */
inekf_real_t inekf_height(const inekf_t *f);
/*
 * How fast the robot is travelling, measured against the room: north, east
 * and up, in metres per second. This does not change meaning when the robot
 * turns on the spot.
 */
void inekf_velocity_world(const inekf_t *f, inekf_real_t *v3);
/*
 * How fast the robot is travelling from its own point of view: forwards,
 * sideways and up, in metres per second. Turn the robot around and "forwards"
 * turns with it. This is usually what a walking controller wants.
 */
void inekf_velocity_body(const inekf_t *f, inekf_real_t *v3);
/*
 * Which way the robot is facing and how it is tilted, as four numbers.
 *
 * Four numbers rather than the three you might expect (roll, pitch, yaw)
 * because three has a well-known flaw: at certain angles two of them stop
 * being distinguishable and the description falls apart. Four numbers avoid
 * that entirely, at the cost of not being readable by eye.
 */
void inekf_quaternion(const inekf_t *f, inekf_real_t *q4);
/*
 * How many feet the estimator is currently treating as planted. Zero means it
 * has nothing fixed to correct against and is drifting on dead reckoning
 * alone.
 */
int  inekf_num_contacts(const inekf_t *f);

#endif /* INEKF_H */
