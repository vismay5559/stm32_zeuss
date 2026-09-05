#ifndef FUSION_H
#define FUSION_H

#include "inekf.h"
#include "link_proto.h"
#include "imu_bno085.h"
#include "act_odrive.h"

/*
 * The bridge between the sensors and the state estimator.
 *
 * inekf.c knows nothing about this robot - it wants IMU samples, contact
 * events, and a foot position with its Jacobian. Everything robot-specific
 * lives here: which encoder is which joint, what units each sensor reports,
 * when a foot counts as planted, and when the estimate is trustworthy.
 *
 * Keeping that split means the filter can be tested against synthetic data
 * (see the host tests) without dragging in the whole robot.
 */

/*
 * Reset the estimator and everything derived from it. Call once at startup,
 * before the first fusion_tick(). Leaves the estimate INVALID until enough
 * ticks have run for it to converge.
 */
void fusion_init(void);

/*
 * Call once per 1 kHz tick, after the sensors have been serviced.
 *
 *   imu       latest sample; prediction runs only when BOTH the accelerometer
 *             and gyro sequence numbers have moved
 *   act       actuator telemetry, positions in turns - the source of every
 *             joint angle forward kinematics uses
 *   contacts  debounced contact bitmask from contact.c
 *   now_us    free-running microsecond counter, for the real dt
 *
 * The spring encoders are deliberately absent. They measure deflection, not
 * joint angle, and feeding them to forward kinematics is what this used to get
 * wrong; they reach the Pi as spring_angle for the torque calculation and play
 * no part in the estimate.
 */
void fusion_tick(const imu_sample_t *imu,
                 const act_telemetry_t *act,
                 uint8_t contacts,
                 uint32_t now_us);

/*
 * Copy the current estimate into the outgoing state packet: position,
 * velocity, orientation, the IMU bias estimates, and the status byte.
 *
 * Call after fusion_tick() in the same tick, so the packet carries this
 * tick's estimate rather than the previous one's.
 */
void fusion_fill_state(nexus_state_t *st);

/*
 * How much the estimate can be trusted, as one of NEXUS_FUSION_INVALID,
 * NEXUS_FUSION_CONVERGING or NEXUS_FUSION_OK.
 *
 * INVALID means it has diverged or has not started; CONVERGING means it is
 * running but has not settled - which is also where it stays for good while
 * robot_config.h is marked UNCALIBRATED, since an uncalibrated robot cannot
 * honestly report OK. safety.c will not arm on anything but OK.
 */
uint8_t fusion_status(void);

/*
 * How many consecutive ticks the estimate has been converged, for
 * diagnostics. Resets to zero whenever it stops being converged, so a value
 * that keeps returning to zero says the filter is struggling rather than
 * simply starting up.
 */
uint32_t fusion_converged_ticks(void);

#endif /* FUSION_H */
