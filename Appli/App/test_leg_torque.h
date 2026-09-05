#ifndef TEST_LEG_TORQUE_H
#define TEST_LEG_TORQUE_H

/*
 * Single-leg test with the position and velocity loops on the STM32.
 *
 * The sibling of test_leg_can.c. Same bus, same drives, same gait - the
 * difference is where the control loop lives:
 *
 *   test_leg_can.c     Set_Input_Pos     drive runs pos -> vel -> current
 *   test_leg_torque.c  Set_Input_Torque  drive runs current only; we run
 *                                        the PD law at 1 kHz
 *
 * Pick one with NEXUS_MODE in nexus_mode.h. They cannot both run: they would
 * fight for the same drives and the same bus.
 *
 * Read the comment block at the top of test_leg_torque.c before arming. In
 * torque mode a joint does not hold position on its own, and it will sag under
 * gravity unless a feedforward term supplies it.
 */

/*
 * A BENCH TEST PROGRAM, NOT PART OF THE ROBOT
 *
 * This is a standalone program for trying out one leg's motors in force mode rather than position mode - checking the leg pushes with the strength asked for, instead of moving to a place.
 * It replaces the normal robot program: you build it, run it, watch the
 * output, and it never hands control back.
 *
 * It exists so a problem can be narrowed down to one part, away from
 * everything else that might also be wrong.
 */

/* Get the test ready. Call once, before legtorque_run(). */
void legtorque_init(void);
/*
 * The timer calls this 1000 times a second while the test runs. Nothing else
 * should call it.
 */
void legtorque_on_tick(void);
/*
 * The hardware calls this by itself whenever the motors send a reply.
 * Nothing else should call it.
 */
void legtorque_on_rx(void);
/*
 * Run the test. Never returns - it keeps going until the power is cut or the
 * board is reset.
 */
void legtorque_run(void);

#endif /* TEST_LEG_TORQUE_H */
