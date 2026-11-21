#include "test_imu_log.h"
#include "console.h"
#include "critical.h"
#include "imu_bno085.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/*
 * Stream the IMU's raw accelerometer and UNCALIBRATED gyro to the ST-LINK
 * console, in binary, for an Allan-variance run.
 *
 * WHY BINARY, WHY 200 Hz. The console is 115200 baud, about 11.5 kB/s. Six
 * numbers as text at 400 Hz is ~25 kB/s and would be dropped by console.c, so
 * each frame is the MEAN of the samples that arrived in a 5 ms window - two at
 * 400 Hz - packed as floats. 34 bytes x 200 Hz = 6.8 kB/s, well inside the
 * wire. Averaging two samples changes nothing an Allan plot reads: its
 * shortest useful tau is already longer than 5 ms, and the noise densities are
 * read at tau = 1 s.
 *
 * WHY UNCALIBRATED. The calibrated gyro has the sensor's own, continuously
 * revised bias removed, which flattens exactly the bias drift this run exists
 * to measure - and is the report the estimator should stop using for the same
 * reason (see SH2_GYRO_UNCAL in imu_bno085.c).
 *
 * Frames, little-endian, decoded by tools/imu_allan/capture.py:
 *
 *   A5 5A  type  len  payload[len]  sum8(type, len, payload)
 *
 *   type 1, len 28   u16 frame counter (gaps = lost frames)
 *                    u8 accel samples averaged, u8 gyro samples averaged
 *                    f32 accel[3] m/s^2 (specific force, includes gravity)
 *                    f32 gyro[3] rad/s, uncalibrated
 *   type 2, len 20   once a second:
 *                    f32 the sensor's own gyro bias estimate[3], rad/s
 *                    u32 console bytes dropped so far
 *                    u32 milliseconds since boot
 *
 * Text the driver prints at startup is interleaved; the decoder resyncs on the
 * header and the checksum, so it costs nothing.
 *
 * Nothing else runs: no CAN, no encoders, no USB. Keep the robot STILL - on
 * the floor or a bench, not on a stand that sways - for the whole hour.
 */

extern TIM_HandleTypeDef htim2;

#define FRAME_MS   5u

/* Straight into console.c's ring, past newlib's stdout buffer, so a frame is
   never held back waiting for a newline that binary data may not contain. */
extern int __io_putchar(int ch);

static void put_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t sum = (uint8_t)(type + len);

    (void)__io_putchar(0xA5);
    (void)__io_putchar(0x5A);
    (void)__io_putchar(type);
    (void)__io_putchar(len);
    for (uint8_t i = 0; i < len; i++)
    {
        (void)__io_putchar(payload[i]);
        sum = (uint8_t)(sum + payload[i]);
    }
    (void)__io_putchar(sum);
}

void imulog_init(void)
{
    printf("\r\n=========== IMU LOG: binary, 200 Hz, uncalibrated gyro ===========\r\n");
    printf("Capture with tools/imu_allan/capture.py. Keep the robot still.\r\n");

    imu_use_uncal_gyro(1u);
    imu_skip_quaternion(1u);
    imu_init();
    HAL_TIM_Base_Start(&htim2);
}

void imulog_run(void)
{
    float    acc_sum[3] = { 0 }, gyr_sum[3] = { 0 };
    uint8_t  n_acc = 0u, n_gyr = 0u;
    uint32_t prev_acc_seq = 0u, prev_gyr_seq = 0u;
    uint16_t frame = 0u;

    uint32_t last_frame = HAL_GetTick();
    uint32_t last_slow  = last_frame;
    uint32_t last_retry = last_frame;

    for (;;)
    {
        imu_service();

        imu_sample_t s;
        imu_get(&s);

        /* Accumulate each NEW sample once; the copy above may be the same one
           many loops in a row. */
        if (s.accel_seq != prev_acc_seq)
        {
            prev_acc_seq = s.accel_seq;
            for (int k = 0; k < 3; k++) { acc_sum[k] += s.accel[k]; }
            n_acc++;
        }
        if (s.gyro_seq != prev_gyr_seq)
        {
            prev_gyr_seq = s.gyro_seq;
            for (int k = 0; k < 3; k++) { gyr_sum[k] += s.gyro[k]; }
            n_gyr++;
        }

        uint32_t now = HAL_GetTick();

        /* Same start-up handshake as every other IMU mode: the sensor ignores
           requests sent before it has booted, so ask until samples arrive. */
        if (s.seq == 0u && (now - last_retry) >= 500u)
        {
            last_retry = now;
            if (!imu_saw_advertisement()) { imu_soft_reset(); }
            else                          { imu_request_reports(); }
        }

        if ((now - last_frame) >= FRAME_MS)
        {
            last_frame += FRAME_MS;

            /* An empty window sends nothing rather than a fake zero. The gap
               in the counter is how the decoder sees it. */
            if (n_acc != 0u && n_gyr != 0u)
            {
                uint8_t p[28];
                float   a[3], g[3];

                for (int k = 0; k < 3; k++)
                {
                    a[k] = acc_sum[k] / (float)n_acc;
                    g[k] = gyr_sum[k] / (float)n_gyr;
                }
                memcpy(&p[0], &frame, 2);
                p[2] = n_acc;
                p[3] = n_gyr;
                memcpy(&p[4],  a, 12);
                memcpy(&p[16], g, 12);
                put_frame(1u, p, sizeof(p));

                memset(acc_sum, 0, sizeof(acc_sum));
                memset(gyr_sum, 0, sizeof(gyr_sum));
                n_acc = n_gyr = 0u;
            }
            frame++;
        }

        if ((now - last_slow) >= 1000u)
        {
            uint8_t  p[20];
            float    b[3];
            uint32_t dropped = console_dropped();

            last_slow = now;
            imu_bno_gyro_bias(b);
            memcpy(&p[0],  b, 12);
            memcpy(&p[12], &dropped, 4);
            memcpy(&p[16], &now, 4);
            put_frame(2u, p, sizeof(p));
            BSP_LED_Toggle(LED_GREEN);
        }
    }
}
