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
 *   imu       latest sample; the prediction step only runs when imu->seq moves
 *   enc_rad   NEXUS_NUM_ENCODERS after-spring angles, radians
 *   enc_valid bit per encoder, from enc_get()
 *   act       actuator telemetry, positions in turns
 *   contacts  debounced contact bitmask from contact.c
 *   now_us    free-running microsecond counter, for the real dt
 */
void fusion_tick(const imu_sample_t *imu,
                 const float *enc_rad, uint8_t enc_valid,
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
