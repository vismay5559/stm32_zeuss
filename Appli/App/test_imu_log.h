#ifndef TEST_IMU_LOG_H
#define TEST_IMU_LOG_H

/*
 * Binary IMU stream for an Allan-variance run: accelerometer and uncalibrated
 * gyro, averaged to 200 Hz, on the ST-LINK console. Decode and analyse with
 * tools/imu_allan/. Touches nothing but the IMU.
 */
void imulog_init(void);
void imulog_run(void);

#endif
