#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>

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

void     health_init(uint32_t expected_mask);
void     health_set_expected(uint32_t mask);
uint32_t health_expected(void);

/* Call once per 1 kHz tick, after the subsystems have been serviced. */
void     health_tick(void);

/* Currently-faulted subsystems, already masked by "expected". 0 = all good. */
uint32_t health_faults(void);

/* 0 = no fault, otherwise 1..HEALTH_COUNT = blink count of the first fault. */
uint8_t  health_blink_code(void);

#endif /* HEALTH_H */
