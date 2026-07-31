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
static inline uint32_t critical_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void critical_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

#endif /* CRITICAL_H */
