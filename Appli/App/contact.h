#ifndef CONTACT_H
#define CONTACT_H

#include <stdint.h>

void    contact_init(void);
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
