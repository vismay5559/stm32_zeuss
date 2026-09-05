#ifndef APP_H
#define APP_H

#include <stdint.h>

/*
 * THE MAIN PROGRAM
 *
 * This is the part that actually runs the robot. Everything else in this
 * folder is a piece it uses.
 *
 * The startup code calls app_init() once to get set up, then app_run(), which
 * never finishes - it keeps going until the power is cut.
 *
 * Two rhythms run at the same time. A timer wakes the board 1000 times a
 * second; each wake-up is a "tick", and once per tick the robot reads its
 * sensors, works out where it is, and reports to the Pi. In between ticks the
 * program keeps busy with quicker jobs: listening to the movement sensor,
 * pushing messages out to the motors, and picking up anything the Pi sent.
 *
 * See README.md in this folder for tick, the Pi, and armed.
 */

/*
 * Get everything ready. Call once at startup, before app_run().
 *
 * Turns off the yellow light the startup code left on, so a board still
 * showing yellow tells you it never got this far. Records why the board last
 * restarted, before anything can erase that - a robot quietly restarting
 * itself mid-run and coming back looking fine is the worst thing that can
 * happen silently. Then clears everything back to a known starting point.
 */
void app_init(void);

/*
 * Run the robot. Never returns.
 *
 * Round and round: keep the sensors and motors serviced, and once per tick do
 * the full cycle of read, work out position, decide, report.
 *
 * Every command from the Pi is checked by safety.c before it can reach a
 * motor. The command arriving undamaged is not the same as the command being
 * safe to obey - the robot might not be healthy, or might not be switched on
 * for movement at all.
 */
void app_run(void);

/*
 * The timer calls this 1000 times a second. Nothing else should call it.
 *
 * It only makes a note that a tick is due and returns straight away. The real
 * work happens in app_run(). Doing it this way means a slow job can never
 * delay the next heartbeat.
 */
void app_on_tick(void);

/*
 * How many heartbeats were missed because a cycle took too long.
 *
 * Should stay at zero. Anything else means the robot is not keeping up with
 * itself, and its sense of timing - which the position estimate depends on -
 * is no longer reliable.
 */
uint32_t app_overruns(void);


#endif /* APP_H */
