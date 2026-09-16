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
 *   imu           latest sample; prediction runs only when BOTH the
 *                 accelerometer and gyro sequence numbers have moved
 *   act           actuator telemetry, positions in turns: the motor side of
 *                 every leg joint, and both waist joints
 *   spring_rad    spring deflections in NEXUS_ENC_* order, from
 *                 robot_spring_deflection(): ADDED to the motor side of hip
 *                 pitch and knee, since link = motor + deflection
 *   spring_valid  enc_valid bitmask; a spring that is not valid, or reads
 *                 beyond its travel, is treated as unknown rather than zero
 *   contacts      debounced bitmask from contact.c - each of the four switches
 *                 (NEXUS_CONTACT_*_BIT) is its own contact point
 *   now_us        free-running microsecond counter, for the real dt
 */
void fusion_tick(const imu_sample_t *imu,
                 const act_telemetry_t *act,
                 const float spring_rad[NEXUS_NUM_ENCODERS],
                 uint8_t spring_valid,
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

/* How many contact points (of the four) the filter is using right now. */
uint8_t fusion_num_contacts(void);

#endif /* FUSION_H */
