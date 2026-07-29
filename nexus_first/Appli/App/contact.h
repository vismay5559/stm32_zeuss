#ifndef CONTACT_H
#define CONTACT_H

#include <stdint.h>

void    contact_init(void);
void    contact_poll(void);

/* Debounced per-switch bits: L_TOE, L_HEEL, R_TOE, R_HEEL. */
uint8_t contact_switches(void);

/* Derived per-foot contact: bit0 = left foot down, bit1 = right foot down. */
uint8_t contact_feet(void);

/* Ticks each foot has held its current contact state, saturating. Lets the
   estimator ignore a foot that has only just landed and may still be bouncing. */
uint16_t contact_stable_ticks(uint8_t foot);

#endif /* CONTACT_H */
