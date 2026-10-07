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
 * WIRING: four sensors on one SPI bus, one chip select each
 *
 *   STM32 pin            Nucleo header       goes to
 *   PB3  SPI1_SCK        CN7 pin 15 (D23)    SCK  of all four sensors
 *   PD7  SPI1_MOSI       CN11 pin 45         MOSI of all four sensors
 *   PB4  SPI1_MISO       CN7 pin 19 (D25)    MISO of all four sensors
 *   PF1  enc_cs_l_hip    CN9 pin 19 (D69)    CSn of the LEFT  HIP  sensor
 *   PD15 enc_cs_l_knee   CN7 pin 18 (D9)     CSn of the LEFT  KNEE sensor
 *   PD14 enc_cs_r_hip    CN7 pin 16 (D10)    CSn of the RIGHT HIP  sensor
 *   PF5  enc_cs_r_knee   CN7 pin 20 (D8)     CSn of the RIGHT KNEE sensor
 *
 * Plus 3.3 V and GND to every sensor. SCK, MOSI and MISO are COMMON - all four
 * sensors sit on the same three wires. Only the chip selects are separate, and
 * exactly one of them is low at a time, so only the selected sensor drives
 * MISO. The other three let go of it.
 *
 * This replaced two daisy chains of two. In a chain the sensors are one long
 * shift register, so the ORDER of the words coming back carried meaning: the
 * first word was from whichever sensor sat nearest MISO. Wire hip and knee the
 * other way round and the two springs swapped in the data, silently. With a
 * select per sensor that cannot happen - a reply can only have come from the
 * one sensor that was selected.
 *
 * If a spring is reported as another one now, the cause is a CS wire on the
 * wrong pin. s_cs[] in enc_as5047p.c is the one place to change.
 *
 * Resulting order, NEXUS_ENC_* in link_proto.h:
 *   0 left hip pitch   1 left knee pitch   2 right hip pitch   3 right knee pitch
 * ---------------------------------------------------------------------------
 *
 * Each tick reads the four sensors one after another, all in the background:
 * the driver is told when each one is done and starts the next. Four short
 * transfers instead of two longer ones costs a little more SPI overhead and
 * buys back the silent hip/knee swap.
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
