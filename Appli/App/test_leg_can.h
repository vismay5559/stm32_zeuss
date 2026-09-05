#ifndef TEST_LEG_CAN_H
#define TEST_LEG_CAN_H

/*
 * Single-leg CAN-FD bring-up test.
 *
 * Exercises FDCAN1 and nothing else - no IMU, no encoders, no USB, no host
 * link. Four ODrive S1 axes (hip roll, hip pitch, knee, ankle) on one bus.
 *
 * Per joint: 1 TX (Set_Input_Pos) and 2 RX (Get_Encoder_Estimates,
 * Get_Torques), plus the ODrive heartbeat.
 *
 * Everything is reported on the ST-LINK serial console at 115200 8N1, so you
 * can see exactly which nodes answer and what they say.
 */

/*
 * A BENCH TEST PROGRAM, NOT PART OF THE ROBOT
 *
 * This is a standalone program for trying out one leg's motors on their own - checking they can be found, switched on, moved to a position, and switched off again.
 * It replaces the normal robot program: you build it, run it, watch the
 * output, and it never hands control back.
 *
 * It exists so a problem can be narrowed down to one part, away from
 * everything else that might also be wrong.
 */

/* Get the test ready. Call once, before legtest_run(). */
void legtest_init(void);
/*
 * The timer calls this 1000 times a second while the test runs. Nothing else
 * should call it.
 */
void legtest_on_tick(void);
/*
 * The hardware calls this by itself whenever the motors send a reply.
 * Nothing else should call it.
 */
void legtest_on_rx(void);
/*
 * Run the test. Never returns - it keeps going until the power is cut or the
 * board is reset.
 */
void legtest_run(void);

#endif /* TEST_LEG_CAN_H */
