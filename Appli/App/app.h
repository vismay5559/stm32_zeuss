#ifndef APP_H
#define APP_H

#include <stdint.h>

/*
 * The application layer: everything the board does after the peripherals are
 * up. main() calls app_init() once and then app_run(), which never returns.
 *
 * Two clocks drive it. The 1 kHz timer interrupt calls app_on_tick(), which
 * only counts; the real per-tick work happens in app_run()'s loop, so a long
 * job cannot run inside an interrupt and delay the next one. Between ticks the
 * loop keeps servicing the IMU, the CAN transmit queue and the USB link.
 */

/*
 * Set the application up. Call once, from main(), after the peripherals are
 * initialised and before app_run().
 *
 * Clears the yellow LED that Boot left on (so a yellow board points at the
 * Boot-to-Appli handover rather than at anything here), reads and reports the
 * reset cause before anything can clear it - a board silently rebooting itself
 * says so on the next line - and zeroes the state the loop keeps.
 */
void app_init(void);

/*
 * Run the robot. Never returns.
 *
 * The loop does two kinds of work. As fast as it can go: service the IMU, pump
 * queued CAN frames to the hardware FIFO, and take any command that arrived
 * over USB. Once per 1 kHz tick: read the sensors, run the estimator, and send
 * the state packet.
 *
 * Every command passes safety.c before it can reach an actuator. A valid CRC
 * proves the bytes survived the wire; it says nothing about whether the board
 * is allowed to be moving or whether the numbers are sane.
 */
void app_run(void);

/*
 * Called from the 1 kHz timer interrupt. Counts the tick and returns
 * immediately - app_run() does the work outside interrupt context.
 */
void app_on_tick(void);

/* Number of 1 kHz ticks missed because a cycle ran long. Should stay at 0. */
uint32_t app_overruns(void);


#endif /* APP_H */
