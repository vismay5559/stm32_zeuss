#include "test_imu.h"
#include "imu_bno085.h"
#include "critical.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

/* ===================================================================== */
/*  CONFIGURATION                                                         */
/* ===================================================================== */

/*
 * How often to print a data line. The IMU produces 400 samples/s but a
 * 115200 baud console can carry roughly 12 lines/s, so printing every sample
 * is impossible - and pointless, since you cannot read it either. Data lines
 * are throttled; the RATE is measured separately over every sample, so the
 * throttling does not hide a rate problem.
 */
#define PRINT_EVERY_MS      200u    /* 5 data lines per second */
#define STATS_EVERY_MS     1000u

/* The rate we asked the BNO085 for, from imu_bno085.c. */
#define EXPECTED_HZ         400u

/* ===================================================================== */

static volatile uint32_t s_tick_pending;
static uint32_t s_tick;

void imutest_on_tick(void)
{
    s_tick_pending++;
}

void imutest_init(void)
{
    s_tick_pending = 0;
    s_tick         = 0;

    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Init(LED_RED);
    BSP_LED_Off(LED_YELLOW);
    BSP_LED_Off(LED_RED);

    printf("\r\n============= BNO085 IMU TEST =============\r\n");
    printf("uart     : USART1 @ 3 Mbaud (PA9 tx / PA10 rx)\r\n");
    printf("protocol : SHTP over UART\r\n");
    printf("requested: rotation vector + linear accel + gyro @ %u Hz\r\n",
           (unsigned)EXPECTED_HZ);
    printf("\r\nWiring: STM32 PA9 -> IMU RX, STM32 PA10 -> IMU TX,\r\n");
    printf("        common ground, and the BNO085 must be strapped for\r\n");
    printf("        UART-SHTP mode (not UART-RVC, not I2C, not SPI).\r\n");
    printf("==========================================\r\n\r\n");

    /* Sends the SET_FEATURE commands that ask for the three reports. */
    imu_init();

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);
}

static void print_data(void)
{
    imu_sample_t s;
    imu_get(&s);

    /*
     * A unit quaternion always satisfies w^2+x^2+y^2+z^2 = 1. Printing the
     * norm is a free sanity check: if it is not ~1.000 the bytes are being
     * misinterpreted (wrong Q-point, misaligned report, wrong report id)
     * even though the numbers look superficially plausible.
     */
    float norm = sqrtf(s.quat[0]*s.quat[0] + s.quat[1]*s.quat[1] +
                       s.quat[2]*s.quat[2] + s.quat[3]*s.quat[3]);

    printf("q[w,x,y,z]=%+.4f %+.4f %+.4f %+.4f |q|=%.3f  "
           "a[m/s2]=%+7.3f %+7.3f %+7.3f  "
           "w[rad/s]=%+7.3f %+7.3f %+7.3f\r\n",
           (double)s.quat[0], (double)s.quat[1],
           (double)s.quat[2], (double)s.quat[3], (double)norm,
           (double)s.accel[0], (double)s.accel[1], (double)s.accel[2],
           (double)s.gyro[0],  (double)s.gyro[1],  (double)s.gyro[2]);
}

static void print_stats(void)
{
    static uint32_t prev_seq, prev_bytes, prev_frames;

    imu_sample_t s;
    imu_get(&s);

    uint32_t bytes  = imu_rx_bytes();
    uint32_t frames = imu_frames();

    uint32_t d_seq   = s.seq  - prev_seq;
    uint32_t d_bytes = bytes  - prev_bytes;
    uint32_t d_frame = frames - prev_frames;

    prev_seq    = s.seq;
    prev_bytes  = bytes;
    prev_frames = frames;

    printf("  >> %lu samples/s (want %u), %lu frames/s, %lu bytes/s",
           (unsigned long)d_seq, (unsigned)EXPECTED_HZ,
           (unsigned long)d_frame, (unsigned long)d_bytes);

    /*
     * Three failures look identical from outside ("no IMU data"), so name
     * which one this is rather than leaving it to guesswork.
     */
    if (d_bytes == 0u)
    {
        printf("   <-- NOTHING ON THE WIRE: check TX/RX not swapped, power, ground");
        BSP_LED_On(LED_RED);
    }
    else if (d_frame == 0u)
    {
        printf("   <-- bytes but no valid frames: wrong baud, or IMU is in "
               "UART-RVC mode instead of SHTP");
        BSP_LED_On(LED_RED);
    }
    else if (d_seq == 0u)
    {
        printf("   <-- framing OK but no reports: SET_FEATURE did not take");
        BSP_LED_On(LED_RED);
    }
    else if (d_seq < (EXPECTED_HZ - (EXPECTED_HZ / 5u)))
    {
        printf("   <-- BELOW requested rate");
        BSP_LED_Off(LED_RED);
    }
    else
    {
        printf("   OK");
        BSP_LED_Off(LED_RED);
    }

    printf("\r\n\r\n");
}

void imutest_run(void)
{
    uint32_t print_tick = 0;
    uint32_t stats_tick = 0;
    uint32_t beat       = 0;

    for (;;)
    {
        /* Drain the UART ring as fast as possible - this is what actually
           parses incoming bytes into samples, and it is not tied to the tick. */
        imu_service();

        if (s_tick_pending == 0u)
        {
            continue;
        }

        uint32_t pm = critical_enter();
        s_tick_pending = 0;
        critical_exit(pm);

        s_tick++;

        if (++beat >= 500u)
        {
            beat = 0;
            BSP_LED_Toggle(LED_GREEN);
        }

        if (++print_tick >= PRINT_EVERY_MS)
        {
            print_tick = 0;
            print_data();
        }

        if (++stats_tick >= STATS_EVERY_MS)
        {
            stats_tick = 0;
            print_stats();

            /* printf blocks for several ms; drop the backlog it created so it
               does not look like the loop fell behind on its own. */
            pm = critical_enter();
            s_tick_pending = 0;
            critical_exit(pm);
        }
    }
}
