#ifndef CONTACT_H
#define CONTACT_H

#include <stdint.h>

/*
 * The four foot switches: left toe, left heel, right toe, right heel.
 *
 * The raw GPIOs chatter as a foot lands, so nothing here reports a switch
 * until it has held its new position for a set number of ticks. Making
 * contact and breaking it use different thresholds, so a foot can be trusted
 * quickly when it lands without flickering off on a small bounce.
 */

/*
 * Set up the switch table and clear all state. Call once at startup, before
 * contact_poll().
 *
 * Each channel is bound to its GPIO and to its bit in the reported mask. The
 * bit constants (NEXUS_CONTACT_*_BIT) are not the same as the array indices
 * (NEXUS_CONTACT_*) - mixing them up made the left toe invisible and the
 * right heel set two bits at once, so they are spelled out here deliberately.
 */
void    contact_init(void);

/*
 * Read all four switches and update the debounce. Call once per tick.
 *
 * A switch reads as closed when its pin is low. A change is only accepted
 * after it has persisted for CONTACT_MAKE_TICKS (landing) or
 * CONTACT_BREAK_TICKS (lifting); anything shorter is treated as bounce and
 * discarded. Also updates the per-foot bits and the stable-tick counters.
 */
void    contact_poll(void);

/* Debounced per-switch bits, in NEXUS_CONTACT_*_BIT positions:
   bit0 L_TOE, bit1 L_HEEL, bit2 R_TOE, bit3 R_HEEL. */
uint8_t contact_switches(void);

/* Derived per-foot contact, in NEXUS_CONTACT_*_FOOT positions:
   bit4 = left foot down, bit5 = right foot down. Either switch on a foot
   counts as that foot being loaded. */
uint8_t contact_feet(void);

/* Ticks each foot has held its current contact state, saturating. Lets the
   estimator ignore a foot that has only just landed and may still be bouncing. */
uint16_t contact_stable_ticks(uint8_t foot);

#endif /* CONTACT_H */
