#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

/*
 * THE CONSOLE, WITHOUT STOPPING THE ROBOT TO TALK.
 *
 * printf goes to the ST-LINK virtual COM port. The BSP implements that by
 * handing each character to HAL_UART_Transmit and waiting for it to leave -
 * 87 us per byte at 115200. A 350-byte status line therefore parks the caller
 * for 30 ms.
 *
 * In a 1 kHz control loop that is not a delay, it is 30 lost ticks. Measured
 * on the bench before this existed: the leg test ran at 971 Hz instead of
 * 1000, losing 28 ticks every second, with one 39 ms hole per second exactly
 * where the once-a-second status block was printed. The loop could not even
 * see it happening, because a missed tick sets a flag that was already set.
 *
 * So printing is made asynchronous. Characters go into a buffer and the caller
 * returns immediately; the UART interrupt drains it in the background. The
 * loop never waits for the console again, whatever it prints.
 *
 * The trade is explicit: if a burst outruns the wire the overflow is DROPPED,
 * not waited for. That is the right way round for a control loop - a truncated
 * log line costs nothing, a missed tick can cost a step - but it must never
 * happen silently, so the dropped bytes are counted and console_dropped() is
 * reported by `zeus check`. A non-zero count means the buffer is too small or
 * something is printing far too much.
 */

/* Enable the UART interrupt that drains the buffer. Call once, after
   BSP_COM_Init has brought the port up. */
void     console_init(void);

/* Characters lost to a full buffer since boot. Should stay 0. */
uint32_t console_dropped(void);

/*
 * Block until everything queued has gone out.
 *
 * Only for the paths that are about to stop the world anyway - Error_Handler,
 * a fault report, a deliberate halt - where losing the last words would lose
 * the reason. Never call this from the control loop; waiting is the entire
 * thing this file exists to avoid.
 */
void     console_flush(void);

#endif /* CONSOLE_H */
