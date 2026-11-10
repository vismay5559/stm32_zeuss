#ifndef TEST_CAN_SCAN_H
#define TEST_CAN_SCAN_H
#include "main.h"

/* Read both CAN buses while leaving all axes untouched. */
void canscan_init(void);
void canscan_run(void);
void canscan_on_rx(FDCAN_HandleTypeDef *hfdcan);

#endif
