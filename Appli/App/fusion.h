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

/* Copy the estimate into the outgoing packet. */
void fusion_fill_state(nexus_state_t *st);

/* NEXUS_FUSION_INVALID / CONVERGING / OK */
uint8_t fusion_status(void);

/* For diagnostics: how long the estimate has been converged, in ticks. */
uint32_t fusion_converged_ticks(void);

#endif /* FUSION_H */
