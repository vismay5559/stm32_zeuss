#ifndef TEST_IMU_H
#define TEST_IMU_H

/*
 * BNO085 IMU bring-up test.
 *
 * Exercises USART1 and nothing else - no CAN, no encoders, no USB. Prints the
 * quaternion, linear acceleration and angular velocity on the ST-LINK serial
 * console at 115200 8N1, along with the actual measured sample rate.
 */

/*
 * A BENCH TEST PROGRAM, NOT PART OF THE ROBOT
 *
 * This is a standalone program for trying out the movement sensor on its own - checking it starts up, keeps sending samples, and reports sensible numbers when the board is tilted by hand.
 * It replaces the normal robot program: you build it, run it, watch the
 * output, and it never hands control back.
 *
 * It exists so a problem can be narrowed down to one part, away from
 * everything else that might also be wrong.
 */

/* Get the test ready. Call once, before imutest_run(). */
void imutest_init(void);
/*
 * The timer calls this 1000 times a second while the test runs. Nothing else
 * should call it.
 */
void imutest_on_tick(void);
/*
 * Run the test. Never returns - it keeps going until the power is cut or the
 * board is reset.
 */
void imutest_run(void);

#endif /* TEST_IMU_H */
