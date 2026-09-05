#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

/*
 * Independent watchdog (IWDG).
 *
 * The point of this is narrow and worth stating plainly: if the 1 kHz loop
 * stops completing cycles, something must reset the board. Without it, a hang
 * anywhere - a stuck DMA wait, a hard fault, an infinite loop in a driver -
 * leaves the last SET_INPUT_POS standing and ten actuators holding position
 * on a machine nobody is controlling any more.
 *
 * WHY REGISTER-LEVEL AND NOT THE HAL
 *
 * stm32h7rsxx_hal_iwdg.c is not part of the vendored driver set in Drivers/,
 * so enabling HAL_IWDG_MODULE_ENABLED would compile and then fail to link.
 * The IWDG is five registers and no interrupts; driving it directly avoids
 * both that and any dependency on CubeMX regenerating a peripheral init.
 *
 * WHERE IT MUST BE REFRESHED
 *
 * Only from the path that has just finished a whole tick - never from the idle
 * loop. A watchdog fed by the idle loop happily keeps petting the dog while
 * the control loop is stalled, which is the exact failure it exists to catch.
 */

/*
 * Start the watchdog. Cannot be stopped again short of a reset, so call this
 * AFTER the slow one-time init (the IMU's reset sequence alone blocks for the
 * better part of a second) and immediately before the tick timer starts.
 *
 * Returns 1 on success, 0 if the IWDG never accepted the configuration -
 * which means its LSI clock is not running, and the caller should treat the
 * board as unsafe to arm.
 */
uint8_t wdg_start(void);

/*
 * Reload the counter so the board is not reset. Call once per COMPLETED
 * control cycle, from the tick path only.
 *
 * Never call this from the idle loop: a watchdog fed there keeps the board
 * alive while the control loop is stalled, which is the one failure it exists
 * to catch.
 */
void wdg_refresh(void);

/*
 * The configured timeout in milliseconds - how long the loop may go without a
 * wdg_refresh() before the board resets. For logging; the value is nominal,
 * since the LSI that clocks the IWDG is not trimmed.
 */
uint32_t wdg_timeout_ms(void);

/*
 * Did the last reset come from the watchdog?
 *
 * Latched from RCC_RSR during wdg_init_reset_cause(), which must run before
 * anything clears the flags. A robot that silently reboots mid-run and comes
 * back looking healthy is the worst possible outcome; this makes it loud.
 */
/*
 * Latch the reset cause from RCC_RSR and clear the flags. Call once, as early
 * in startup as possible - anything that clears RCC_RSR first destroys the
 * evidence.
 */
void    wdg_init_reset_cause(void);

/*
 * Returns 1 if the last reset was the watchdog firing, 0 otherwise. Only
 * meaningful after wdg_init_reset_cause() has run.
 */
uint8_t wdg_reset_was_watchdog(void);

#endif /* WATCHDOG_H */
