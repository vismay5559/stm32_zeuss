#ifndef ENC_AS5047P_H
#define ENC_AS5047P_H

#include <stdint.h>
#include "link_proto.h"

/*
 * THE SPRING SENSORS
 *
 * The hip pitch and knee of each leg have a spring in them, so the leg gives a
 * little instead of being rigid. An AS5047P on each spring measures how far it
 * is wound up. From that you can work out how hard the leg is pushing, because
 * a spring pushed twice as far pushes twice as hard.
 *
 * IMPORTANT: they measure the WIND-UP, not the angle of the joint. The joint
 * angles come from the motors. Reading these as joint angles is a mistake
 * that has been made in this codebase before, and it sent the estimator's
 * idea of where the foot was badly wrong.
 *
 * ---------------------------------------------------------------------------
 * WIRING: two daisy chains, one per leg, sharing SPI1
 *
 *   STM32 pin           Nucleo header    goes to
 *   PB3  SPI1_SCK       -                CLK  of all four sensors
 *   PD7  SPI1_MOSI      -                MOSI of both HIP sensors
 *   PB4  SPI1_MISO      -                MISO of both KNEE sensors
 *   PF1  enc_cs_left    -                CSn  of both LEFT sensors
 *   PD14 enc_cs_right   Arduino D10      CSn  of both RIGHT sensors
 *
 *   and within each leg:  HIP MISO -> KNEE MOSI
 *
 *        MOSI ──► [hip] ──► [knee] ──► MISO          (per leg, own CSn)
 *
 * Plus 3.3 V and GND to every sensor. With its CSn high a sensor lets go of
 * MISO, so the two knees can share the MISO line.
 *
 * Why that order matters: a daisy chain is one long shift register. In a
 * two-word transfer the FIRST word back is from the sensor nearest MISO (the
 * knee) and the SECOND from the one nearest MOSI (the hip). Swap hip and knee
 * in the wiring and the two springs swap in the data, silently - so after
 * wiring, press one spring by hand and check which spring_angle moves.
 * s_slot[] in enc_as5047p.c is the one place to change if the harness differs.
 *
 * Resulting order, NEXUS_ENC_* in link_proto.h:
 *   0 left hip pitch   1 left knee pitch   2 right hip pitch   3 right knee pitch
 * ---------------------------------------------------------------------------
 *
 * Each tick reads the left chain, then the right chain straight after it, both
 * in the background: the driver is told when each is done.
 */

/*
 * Get the sensors ready. Call once when the robot starts up, before anything
 * else here.
 */
void enc_init(void);

/*
 * Ask all the sensors for a fresh reading. Call once per tick, at the end of
 * the tick.
 *
 * This only asks. The answers arrive a moment later, on their own. Asking at
 * the end of the tick gives the reply the whole gap before the next tick to
 * come back, which means readings are always the same age rather than
 * sometimes fresh and sometimes not.
 *
 * If the previous request never came back, this leaves it alone for a while
 * and then gives up on it, counting that in enc_stalls().
 */
void enc_start_read(void);

/*
 * The hardware calls this by itself when a chain's readings have arrived.
 * Nothing else should call it.
 *
 * Each sensor sends a check digit along with its reading. A reading is only
 * kept if that digit adds up and the sensor is not reporting a fault, so a
 * garbled reading is thrown away rather than believed. A reading that fails
 * keeps its previous value and is marked untrustworthy.
 */
void enc_on_dma_complete(void);

/*
 * Hand back the newest readings, in NEXUS_ENC_* order.
 *
 * `angle` gets one number per sensor: a whole turn of the shaft counts from
 * 0 up to 16383 and then wraps back to 0.
 *
 * `valid_mask` says which of those numbers to trust - one yes/no per sensor.
 * ALWAYS check it. A "no" means that sensor's number is left over from an
 * earlier reading, and using it as if it were current is the failure this
 * mask exists to prevent.
 */
void enc_get(uint16_t angle[NEXUS_NUM_ENCODERS], uint8_t *valid_mask);

/*
 * How many requests were given up on because no answer ever came back.
 *
 * Anything other than zero means the wiring or the connection is unreliable -
 * a worse problem than one bad reading, because it means the robot is
 * regularly flying blind on these sensors.
 */
uint32_t enc_stalls(void);

/* How many communication errors happened and were recovered from. */
uint32_t enc_errors(void);

#endif /* ENC_AS5047P_H */
