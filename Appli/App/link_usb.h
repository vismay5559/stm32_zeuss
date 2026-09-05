#ifndef LINK_USB_H
#define LINK_USB_H

#include <stdint.h>
#include "link_proto.h"

/*
 * The USB link to the Pi: one state packet out per tick, commands in.
 *
 * Bytes arrive in an interrupt and are assembled into a command there;
 * app_run() collects the finished command later, outside interrupt context.
 * Both directions are CRC-checked, so a corrupted frame is dropped rather
 * than acted on.
 */

/* Prepare the link. Call once at startup, before anything else here. */
void    link_usb_init(void);

/*
 * Send one state packet to the Pi. Call once per tick.
 *
 * Fills in the header and CRC, then hands the packet to USB. Returns the USB
 * status - USBD_OK when queued, USBD_BUSY when the previous packet is still
 * going out, in which case this one is dropped and counted in
 * link_usb_tx_dropped().
 *
 * The busy check happens BEFORE the packet is copied: the buffer is still
 * being read by USB DMA, so copying first would splice two ticks together.
 */
uint8_t link_usb_send_state(nexus_state_t *st);

/*
 * Called from the USB receive interrupt with whatever bytes arrived. Not for
 * application code.
 *
 * Feeds them through the frame parser one at a time. A complete, CRC-valid
 * command is stored for link_usb_take_command() to collect.
 */
void    link_usb_on_rx(uint8_t *buf, uint32_t len);

/*
 * Collect the most recent command, if one has arrived since the last call.
 *
 * Returns 1 and fills `out` when a command is waiting, 0 when none is. Only
 * complete, CRC-valid frames are ever handed back.
 *
 * Interrupts are briefly disabled while copying: the receive interrupt writes
 * the same buffer, and a command arriving mid-copy would blend two frames into
 * a target that was never sent.
 *
 * A valid CRC means the bytes survived the wire. It does not mean the command
 * is safe to act on - that is safety.c's decision, not this one's.
 */
uint8_t link_usb_take_command(nexus_cmd_t *out);

/* Total valid commands received from the Pi. Stops changing if the link dies. */
uint32_t link_usb_cmd_count(void);

/* State packets never sent because the USB endpoint was still busy. Matches
   the sequence gaps the Pi sees in nexus_link's stats. */
uint32_t link_usb_tx_dropped(void);

#endif /* LINK_USB_H */
