#ifndef TEST_SENSORS_H
#define TEST_SENSORS_H
#include "main.h"

/*
 * Every sensor at once: both CAN buses (receive only), the four spring
 * encoders and the IMU. One status block per second on the ST-LINK console.
 * Never commands or arms a drive.
 */
void sensors_init(void);
void sensors_run(void);
void sensors_on_can_rx(FDCAN_HandleTypeDef *hfdcan);

#endif
