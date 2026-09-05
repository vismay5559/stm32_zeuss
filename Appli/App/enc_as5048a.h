#ifndef ENC_AS5048A_H
#define ENC_AS5048A_H

#include <stdint.h>
#include "link_proto.h"

/*
 * AS5048A magnetic encoders, read over one SPI bus by DMA.
 *
 * All NEXUS_NUM_ENCODERS devices are daisy-chained, so a single transfer
 * clocks a reading out of every one of them. The transfer is started at the
 * end of a tick and lands during the gap before the next, which gives it the
 * whole inter-tick period and makes the sample age a constant 1 ms rather
 * than something that varies with how long the tick ran.
 *
 * These encoders sit AFTER the series springs. What they measure is spring
 * deflection, not joint angle - the joint angles come from the drives. Reading
 * them as joint angles is a mistake this codebase has made before.
 */

/*
 * Prepare the driver. Call once at startup, before any other function here.
 * Fills the transmit buffer with the read-angle command, clears the stored
 * readings and counters, and idles chip-select high.
 */
void enc_init(void);

/*
 * Begin one SPI transfer for every encoder. Call once per tick, at the end of
 * the tick.
 *
 * If the previous transfer has not finished it is left alone, and after
 * ENC_XFER_TIMEOUT_TICKS of that it is abandoned and counted in enc_stalls().
 * Returns immediately either way - the reading arrives later, in the DMA
 * completion interrupt.
 */
void enc_start_read(void);

/*
 * Called from the SPI DMA completion interrupt. Not for application code.
 *
 * Releases chip-select and decodes each 16-bit word: a reading is kept only if
 * its even-parity bit checks out and the device's error flag is clear, so a
 * garbled word is dropped rather than believed. Readings that fail are left at
 * their previous value and their bit is cleared in the valid mask.
 */
void enc_on_dma_complete(void);

/*
 * Copy out the most recent readings.
 *
 * `angle` receives one raw 14-bit count per encoder (0..16383 over a full
 * turn). `valid_mask` receives one bit per encoder, set when that reading
 * passed its parity and error checks in the last completed transfer - always
 * check it, because a cleared bit means the matching angle is stale.
 *
 * Interrupts are briefly disabled so the angles and the mask describe the same
 * sample; the DMA interrupt rewrites both.
 */
void enc_get(uint16_t angle[NEXUS_NUM_ENCODERS], uint8_t *valid_mask);

/* Transfers abandoned because the DMA never completed. Non-zero means the SPI
   link is unreliable, not that a reading was merely bad. */
uint32_t enc_stalls(void);

/* SPI errors recovered from. */
uint32_t enc_errors(void);

#endif /* ENC_AS5048A_H */
