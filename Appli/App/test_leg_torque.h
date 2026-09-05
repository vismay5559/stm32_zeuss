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

void legtorque_init(void);
void legtorque_on_tick(void);   /* from the TIM6 1 kHz interrupt */
void legtorque_on_rx(void);     /* from the FDCAN1 rx interrupt  */
void legtorque_run(void);       /* never returns */

#endif /* TEST_LEG_TORQUE_H */
