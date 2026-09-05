#ifndef LINK_USB_H
#define LINK_USB_H

#include <stdint.h>
#include "link_proto.h"

/*
 * THE CABLE TO THE PI
 *
 * The board and the Pi talk over a USB cable. Once per tick the board sends a
 * full report of everything it knows; whenever the Pi has an instruction, it
 * sends one back.
 *
 * Both directions carry a check number, so a message damaged on the way is
 * thrown away rather than acted on. A message arriving intact still is not
 * permission to move - safety.c decides that.
 *
 * See README.md in this folder for tick, the Pi, CRC and interrupt.
 */

/* Get the link ready. Call once at startup, before anything else here. */
void    link_usb_init(void);

/*
 * Send one report to the Pi. Call once per tick.
 *
 * Fills in the header and check number, then hands it over to be sent.
 *
 * If the previous report is still going out, this one is dropped and counted
 * in link_usb_tx_dropped(). That check happens BEFORE the report is copied:
 * the previous one is still being read out of the same space, so copying
 * first would splice two ticks together and send the Pi a report that is half
 * one moment and half another.
 */
uint8_t link_usb_send_state(nexus_state_t *st);

/*
 * The hardware calls this by itself when bytes arrive from the Pi. Nothing
 * else should call it.
 *
 * Feeds them through the message reader one at a time. A complete, undamaged
 * instruction is set aside for link_usb_take_command() to pick up.
 */
void    link_usb_on_rx(uint8_t *buf, uint32_t len);

/*
 * Pick up the latest instruction from the Pi, if one has arrived.
 *
 * Returns 1 and fills in `out` when there is one, 0 when there is not. Only
 * complete, undamaged instructions are ever handed over.
 *
 * Interrupts are held off for the moment it takes to copy, because a new
 * instruction landing halfway through would blend two of them together and
 * produce a target position the Pi never actually asked for.
 */
uint8_t link_usb_take_command(nexus_cmd_t *out);

/*
 * How many good instructions have arrived from the Pi in total.
 *
 * If this stops climbing, the Pi has gone quiet - which the safety system
 * treats as reason to stop moving.
 */
uint32_t link_usb_cmd_count(void);

/*
 * How many reports were never sent because the cable was still busy with the
 * previous one.
 *
 * The Pi sees these as gaps in the numbering, so the two sides should agree
 * on this figure. A few is normal; a rising count means the board is
 * producing reports faster than the cable is carrying them.
 */
uint32_t link_usb_tx_dropped(void);

#endif /* LINK_USB_H */
