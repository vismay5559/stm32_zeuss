#ifndef ENC_AS5048A_H
#define ENC_AS5048A_H

#include <stdint.h>
#include "link_proto.h"

/*
 * THE SPRING SENSORS
 *
 * Each leg joint has a spring in it, so the leg gives a little instead of
 * being rigid. These sensors measure how far each spring is squashed. From
 * that you can work out how hard the leg is pushing, because a spring pushed
 * twice as far pushes twice as hard.
 *
 * IMPORTANT: they measure the SQUASH, not the angle of the joint. The joint
 * angles come from the motors instead. Reading these as joint angles is a
 * mistake that has been made in this codebase before, and it sent the
 * estimator's idea of where the foot was badly wrong.
 *
 * All the sensors are wired in a chain, so one request reads every one of
 * them at once. See README.md in this folder for tick, DMA and mask.
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
 * This only asks. The answer arrives a moment later, on its own. Asking at the
 * end of the tick gives the reply the whole gap before the next tick to come
 * back, which means readings are always the same age rather than sometimes
 * fresh and sometimes not.
 *
 * If the previous request never came back, this leaves it alone for a while
 * and then gives up on it, counting that in enc_stalls().
 */
void enc_start_read(void);

/*
 * The hardware calls this by itself when the readings have arrived. Nothing
 * else should call it.
 *
 * Each sensor sends a check digit along with its reading. A reading is only
 * kept if that digit adds up and the sensor is not reporting a fault, so a
 * garbled reading is thrown away rather than believed. A reading that fails
 * keeps its previous value and is marked untrustworthy.
 */
void enc_on_dma_complete(void);

/*
 * Hand back the newest readings.
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

#endif /* ENC_AS5048A_H */
