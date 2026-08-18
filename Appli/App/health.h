#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>
#include "imu_bno085.h"

/*
 * Subsystem health tracking.
 *
 * The board has three LEDs but six things worth watching, so instead of one
 * LED per subsystem this keeps a bitmask and reports the first fault as a
 * blink code on the red LED. The same bitmask is what the Pi should eventually
 * receive, so the bench diagnosis and the logged diagnosis are the same value.
 *
 * "Healthy" here means DATA IS FLOWING, not "the peripheral initialised".
 * A peripheral that init'd fine but has nothing plugged into it is a fault -
 * which is exactly what you want to see while wiring things up.
 */

#define HEALTH_IMU     (1u << 0)   /* blink 1: no IMU samples arriving        */
#define HEALTH_ENC     (1u << 1)   /* blink 2: encoder reads invalid          */
#define HEALTH_CAN1    (1u << 2)   /* blink 3: nothing heard on CAN bus 1     */
#define HEALTH_CAN2    (1u << 3)   /* blink 4: nothing heard on CAN bus 2     */
#define HEALTH_LINK    (1u << 4)   /* blink 5: no commands from the Pi        */
#define HEALTH_TIMING  (1u << 5)   /* blink 6: the 1 kHz loop missed a tick   */

#define HEALTH_COUNT   6u

/*
 * Which subsystems are actually wired up yet. Only these can raise a fault,
 * so during bring-up the board stays quiet about the things you have not
 * connected. Start at HEALTH_TIMING only and add each subsystem as you wire
 * it - the LED then tells you the moment that subsystem starts working.
 */
/*
 * Set this to the subsystems that are physically connected right now.
 * Add a flag the moment you plug that hardware in - the red LED will blink
 * its code until data actually flows, then go dark. That transition is the
 * clearest possible "it works" signal during wiring.
 *
 *   nothing wired      HEALTH_TIMING
 *   + IMU              HEALTH_TIMING | HEALTH_IMU
 *   + encoders         ... | HEALTH_ENC
 *   + one CAN bus      ... | HEALTH_CAN1
 *   + both CAN buses   ... | HEALTH_CAN2
 *   + Pi connected     ... | HEALTH_LINK
 */
#define HEALTH_EXPECTED_NOW  (HEALTH_TIMING | HEALTH_IMU)

/* Everything the finished robot has plugged in. */
#define HEALTH_EXPECTED_ROBOT  (HEALTH_TIMING | HEALTH_IMU | HEALTH_ENC | \
                                HEALTH_CAN1   | HEALTH_CAN2 | HEALTH_LINK)

/*
 * What app_init() actually watches.
 *
 * This defaults to the full set, because safety.c will not arm the actuators
 * until everything being watched is healthy - and a failsafe that is not
 * watching the Pi link is not a failsafe. A partially wired bench therefore
 * sits in BOOT and refuses to arm, which is the correct answer to "half the
 * robot is missing", not an obstacle to work around.
 *
 * During bring-up, narrow it at configure time rather than by editing here:
 *
 *     HEALTH_EXPECTED_MASK='(HEALTH_TIMING|HEALTH_IMU)' cmake --preset Debug
 *
 * (An environment variable, not -D: the top-level project configures Appli/
 * through ExternalProject_Add and does not forward -D arguments. See
 * Appli/CMakeLists.txt.)
 */
#ifndef HEALTH_EXPECTED_MASK
#define HEALTH_EXPECTED_MASK  HEALTH_EXPECTED_ROBOT
#endif

void     health_init(uint32_t expected_mask);
void     health_set_expected(uint32_t mask);
uint32_t health_expected(void);

/*
 * Call once per 1 kHz tick, after the subsystems have been serviced.
 *
 * Takes the sensor readings the caller has ALREADY fetched this tick rather
 * than fetching its own. It used to call imu_get() and enc_get() again for
 * data app.c had read microseconds earlier - two more critical sections and a
 * struct copy every millisecond, and worse, a second sample that could differ
 * from the one that went into the packet.
 */
void     health_tick(const imu_sample_t *imu, uint8_t enc_valid);

/*
 * Acknowledge the latched timing fault.
 *
 * Every other check clears itself the moment data flows again. HEALTH_TIMING
 * deliberately does not - a missed deadline matters after the tick that missed
 * it - so something has to be able to say "I have seen that, carry on", or the
 * first overrun of a session locks the robot out permanently.
 *
 * Called by safety.c on the explicit re-arm handshake, so acknowledging is
 * always a deliberate act by whoever is flying the robot.
 */
void     health_clear_latched(void);

/* Currently-faulted subsystems, already masked by "expected". 0 = all good. */
uint32_t health_faults(void);

/* 0 = no fault, otherwise 1..HEALTH_COUNT = blink count of the first fault. */
uint8_t  health_blink_code(void);

#endif /* HEALTH_H */
