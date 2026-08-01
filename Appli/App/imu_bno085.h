#ifndef IMU_BNO085_H
#define IMU_BNO085_H

#include <stdint.h>

typedef struct
{
    float    quat[4];    /* w, x, y, z */
    float    accel[3];   /* m/s^2, gravity removed */
    float    gyro[3];    /* rad/s */
    uint32_t seq;
} imu_sample_t;

void imu_init(void);
void imu_service(void);
void imu_get(imu_sample_t *out);
void imu_on_rx_event(uint16_t size);
uint32_t imu_rx_events(void);

/* Diagnostics: raw bytes off the UART, and valid SHTP frames parsed out of
   them. Together they separate a wiring fault from a protocol fault. */
uint32_t imu_rx_bytes(void);
uint32_t imu_frames(void);

#endif /* IMU_BNO085_H */
