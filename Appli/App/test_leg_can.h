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

void legtest_init(void);
void legtest_on_tick(void);   /* from the TIM6 1 kHz interrupt */
void legtest_on_rx(void);     /* from the FDCAN1 rx interrupt  */
void legtest_run(void);   /* never returns */

#endif /* TEST_LEG_CAN_H */
