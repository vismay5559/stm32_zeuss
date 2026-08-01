#ifndef TEST_IMU_H
#define TEST_IMU_H

/*
 * BNO085 IMU bring-up test.
 *
 * Exercises USART1 and nothing else - no CAN, no encoders, no USB. Prints the
 * quaternion, linear acceleration and angular velocity on the ST-LINK serial
 * console at 115200 8N1, along with the actual measured sample rate.
 */

void imutest_init(void);
void imutest_on_tick(void);   /* from the TIM6 1 kHz interrupt */
void imutest_run(void);       /* never returns */

#endif /* TEST_IMU_H */
