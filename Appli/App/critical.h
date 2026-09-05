#ifndef CRITICAL_H
#define CRITICAL_H

#include "main.h"

/*
 * A "critical section" briefly blocks every interrupt so that a multi-byte
 * copy cannot be torn in half by an ISR that writes the same variables.
 *
 * Without this, a struct copy like  *out = s_telem;  can be interrupted
 * mid-way: the FDCAN ISR overwrites s_telem, and the caller ends up with
 * joint 1 from tick N and joint 2 from tick N+1 - a state that never
 * actually existed.
 *
 * PRIMASK is saved and restored rather than blindly re-enabling, so these
 * are safe to nest and safe to call from inside an ISR.
 *
 * Cost: the copies guarded here are at most ~180 bytes, roughly 100 ns on a
 * 600 MHz M7. That is far shorter than one CAN bit time, so nothing is at
 * risk of being missed.
 */
/*
 * Block interrupts and return the previous PRIMASK, which must be handed back
 * to critical_exit(). Safe to nest and safe to call from inside an ISR,
 * because it restores what it found rather than blindly re-enabling.
 */
static inline uint32_t critical_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/*
 * Restore the interrupt state that critical_enter() returned. Pass the value
 * from the matching critical_enter() - not 0 - or a nested section will
 * re-enable interrupts that an outer one meant to keep blocked.
 */
static inline void critical_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

#endif /* CRITICAL_H */
