#include "test_imu.h"
#include "imu_bno085.h"
#include "critical.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;

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

/*
 * Loopback: set to 1, rebuild, and jumper PA9 directly to PA10 with the IMU
 * DISCONNECTED.
 *
 * This is the one test that separates "the STM32 is not listening" from "the
 * sensor is not talking" - two faults that produce identical output. The STM32
 * transmits a pattern to itself, so bytes coming back prove the UART, the DMA,
 * the pin mux and the clocks all work, and point the finger squarely at the
 * sensor or the wiring to it. No bytes means the fault is on this side.
 */
#define IMUTEST_LOOPBACK    0

/*
 * Override the UART baud rate. 0 keeps the 3 Mbaud that SHTP uses.
 *
 * Framing errors mean the bytes arriving do not match the rate we are reading
 * them at, and there are exactly two reasons for that: the sensor is talking a
 * different rate, or the link cannot carry 3 Mbaud cleanly. Setting this to
 * 115200 separates them - that is UART-RVC's rate, and slow enough that any
 * wiring will carry it.
 *
 *   clean bytes at 115200 -> sensor is in RVC mode; the straps did not take
 *   errors at 115200 too  -> wiring or signal integrity, not the mode
 */
#define IMUTEST_BAUD        0

/*
 * UART-RVC fallback. Set to 1, rewire PS0 to 3V3 and PS1 to GND, power-cycle.
 *
 * RVC has no handshake at all: the sensor powers up and transmits 19-byte
 * frames at 115200 forever, ignoring the host completely. Nothing we send can
 * be wrong because we send nothing. So if this works, the part is healthy and
 * the fault is confined to the SHTP conversation; if this fails too, the
 * sensor or the link is bad and no protocol work will help.
 *
 * Not a solution for the robot - 100 Hz, Euler angles, no gyro - but it
 * answers "is this sensor alive" definitively, which SHTP currently cannot.
 */
#define IMUTEST_RVC         0

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
    printf("\r\nWiring (Adafruit BNO08x breakout):\r\n");
    printf("  STM32 PA9  -> SCL pin  (sensor UART RX)\r\n");
    printf("  STM32 PA10 <- SDA pin  (sensor UART TX)\r\n");
    printf("  Vin -> 3V3, GND -> GND\r\n");
    printf("\r\nMode straps - both pins are pulled LOW on the breakout:\r\n");
    printf("  PS1  PS0   mode\r\n");
    printf("  low  low   I2C\r\n");
    printf("  low  HIGH  UART-RVC   (115200, 100 Hz, no gyro/quaternion)\r\n");
    printf("  HIGH low   UART-SHTP  <-- this one: PS1 to 3V3, PS0 alone\r\n");
    printf("  HIGH HIGH  SPI\r\n");
#if IMUTEST_LOOPBACK
    printf("\r\n*** LOOPBACK MODE ***\r\n");
    printf("  Two ways to wire this:\r\n");
    printf("   (a) IMU disconnected, PA9 jumpered straight to PA10.\r\n");
    printf("       Tests the STM32 alone - UART, DMA, pin mux, clocks.\r\n");
    printf("   (b) IMU connected, PA9 -> SCL as usual, and PA10 moved from\r\n");
    printf("       SDA to SCL as well. Reads our own transmission back off\r\n");
    printf("       the wire the sensor actually listens on, so corruption\r\n");
    printf("       there is corruption the sensor sees too.\r\n");
    printf("  Frames and samples stay 0 - the pattern is not SHTP.\r\n");
    printf("  Watch 'corrupt', not bytes: the right number of WRONG bytes\r\n");
    printf("  looks identical to success if you only count them.\r\n");
#endif
    printf("==========================================\r\n\r\n");

#if IMUTEST_RVC || IMUTEST_BAUD
#if IMUTEST_RVC
    /* RVC is 115200 by definition; no point making the user set both. */
    printf("*** UART-RVC MODE: strap PS0 HIGH, PS1 LOW, power-cycle ***\r\n\r\n");
    huart1.Init.BaudRate = 115200;
#else
    printf("*** BAUD OVERRIDE: %u (SHTP normally needs 3000000) ***\r\n\r\n",
           (unsigned)IMUTEST_BAUD);
    huart1.Init.BaudRate = IMUTEST_BAUD;
#endif
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        printf("!! HAL_UART_Init failed at %u baud\r\n", (unsigned)IMUTEST_BAUD);
    }
#endif

#if IMUTEST_RVC
    /*
     * Deliberately skip imu_init(). It arms the RX DMA, which would then fight
     * the polled HAL_UART_Receive() in rvc_run() and return BUSY forever - RVC
     * would fail for a reason that has nothing to do with the sensor, which is
     * exactly the confusion this test exists to avoid. It also sends SHTP
     * commands, and the whole point of RVC is that we send nothing at all.
     */
    printf("RVC: not sending anything - the sensor talks on its own\r\n\r\n");
#else
    /* Sends the SET_FEATURE commands that ask for the three reports. */
    imu_init();
#endif

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
    static uint32_t prev_seq, prev_bytes, prev_frames, prev_errs;

    imu_sample_t s;
    imu_get(&s);

    uint32_t bytes  = imu_rx_bytes();
    uint32_t frames = imu_frames();
    uint32_t errs   = imu_errors();

    uint32_t d_seq   = s.seq  - prev_seq;
    uint32_t d_bytes = bytes  - prev_bytes;
    uint32_t d_frame = frames - prev_frames;
    uint32_t d_errs  = errs   - prev_errs;

    prev_seq    = s.seq;
    prev_bytes  = bytes;
    prev_frames = frames;
    prev_errs   = errs;

    /*
     * Totals as well as rates. A sensor that talks once at boot and then goes
     * quiet - which is exactly what SHTP does, it sends an advertisement and
     * waits - shows up as zero on every rate while the totals say several
     * hundred bytes arrived. Printing only rates hides that completely.
     */
    printf("  >> %lu samples/s (want %u), %lu frames/s, %lu bytes/s, %lu err/s\r\n",
           (unsigned long)d_seq, (unsigned)EXPECTED_HZ,
           (unsigned long)d_frame, (unsigned long)d_bytes,
           (unsigned long)d_errs);
    printf("     totals: %lu bytes, %lu frames, %lu samples, %lu errors\r\n",
           (unsigned long)bytes, (unsigned long)frames,
           (unsigned long)s.seq, (unsigned long)errs);

#if IMUTEST_LOOPBACK
    {
        uint32_t total = 0;
        uint32_t bad   = imu_loopback_check(&total);

        printf("     loopback: %lu of %lu bytes corrupt -> %s\r\n",
               (unsigned long)bad, (unsigned long)total,
               (total == 0u) ? "nothing came back"
                             : ((bad == 0u) ? "LINK CLEAN"
                                            : "LINK CORRUPTS DATA"));
    }
#endif
    printf("     frames by channel: 0/cmd=%u 1/exec=%u 2/control=%u 3/reports=%u"
           "  last ctrl resp=0x%02X",
           (unsigned)imu_channel_frames(0), (unsigned)imu_channel_frames(1),
           (unsigned)imu_channel_frames(2), (unsigned)imu_channel_frames(3),
           (unsigned)imu_last_control_response());
    printf("\r\n     handshake: advertisement=%s reset-notice=%s",
           imu_saw_advertisement() ? "SEEN" : "not seen",
           imu_saw_reset() ? "seen" : "not seen");

    {
        static uint32_t p_rv, p_ac, p_gy;
        uint32_t rv, ac, gy;

        imu_report_counts(&rv, &ac, &gy);
        printf("\r\n     per-report Hz: rotation=%lu accel=%lu gyro=%lu",
               (unsigned long)(rv - p_rv), (unsigned long)(ac - p_ac),
               (unsigned long)(gy - p_gy));
        p_rv = rv; p_ac = ac; p_gy = gy;
    }

    {
        uint8_t  c[32];
        uint16_t cn = imu_control_payload(c, sizeof(c));

        if (cn > 0u)
        {
            printf("\r\n     control reply:");
            for (uint16_t i = 0; i < cn; i++) { printf(" %02X", (unsigned)c[i]); }

            /* Get Feature Response: [0]=0xFC, [1]=report id, [5..8]=interval us.
               A zero interval means the sensor has the report switched off -
               the difference between "rejected" and "accepted but idle", which
               Set Feature alone can never tell us. */
            if ((c[0] == 0xFCu) && (cn >= 9u))
            {
                uint32_t iv = (uint32_t)c[5] | ((uint32_t)c[6] << 8) |
                              ((uint32_t)c[7] << 16) | ((uint32_t)c[8] << 24);
                printf("\r\n     -> report 0x%02X interval %lu us (%s)",
                       (unsigned)c[1], (unsigned long)iv,
                       (iv == 0u) ? "DISABLED" : "enabled");
            }
        }
    }

    /* Dump the raw bytes once. Everything else is derived; this is not. */
    static uint8_t s_dumped;
    if (!s_dumped && (bytes > 0u))
    {
        uint8_t  snap[64];
        uint16_t n = imu_snapshot(snap, sizeof(snap));

        s_dumped = 1;
        printf("\r\n     first %u bytes:", (unsigned)n);
        for (uint16_t i = 0; i < n; i++)
        {
            if ((i % 16u) == 0u) { printf("\r\n       "); }
            printf("%02X ", (unsigned)snap[i]);
        }
        /* And what we sent, so both halves of the conversation are visible. */
        uint16_t t = imu_tx_snapshot(snap, sizeof(snap));
        printf("\r\n     we transmit (%u bytes):", (unsigned)t);
        for (uint16_t i = 0; i < t; i++)
        {
            if ((i % 16u) == 0u) { printf("\r\n       "); }
            printf("%02X ", (unsigned)snap[i]);
        }

        n = imu_snapshot(snap, sizeof(snap));
        printf("\r\n     ");
        if ((n >= 2u) && (snap[0] == 0x7Eu))
        {
            printf("-> starts 7E: SHTP framing, baud is correct");
        }
        else if ((n >= 2u) && (snap[0] == 0xAAu) && (snap[1] == 0xAAu))
        {
            printf("-> starts AA AA: sensor is in UART-RVC mode, not SHTP");
        }
        else
        {
            printf("-> snapshot starts mid-frame. Harmless when frames/s is"
                   " healthy:\r\n        with -rst the sensor keeps streaming while the STM32\r\n        reboots, so capture begins mid-packet.");
        }
    }

    /*
     * Errors alongside bytes is a different diagnosis from either alone: the
     * link is alive but the bytes are being mangled, so nothing downstream
     * will ever parse no matter how long it runs.
     */
    if ((d_errs > 0u) && (d_bytes > 0u))
    {
        printf("\r\n     <-- bytes ARE arriving but corrupt. Wrong baud, or the\r\n");
        printf("         wiring cannot carry %lu baud - try IMUTEST_BAUD 115200",
               (unsigned long)huart1.Init.BaudRate);
    }

    /*
     * Three failures look identical from outside ("no IMU data"), so name
     * which one this is rather than leaving it to guesswork.
     */
    if (d_bytes == 0u)
    {
        printf("   <-- NOTHING ON THE WIRE\r\n");

        /*
         * Zero bytes has two completely different causes and the counters
         * above cannot tell them apart, because a driver that never armed its
         * receiver reports exactly the same zeros as a disconnected sensor.
         * These come straight from the hardware registers, so they answer the
         * prior question: is this side even listening?
         */
        imu_diag_t d;
        imu_diag(&d);

        printf("     arm=%lu isr=0x%08lX err=0x%lX rxstate=0x%lX ndtr=%lu evts=%lu\r\n",
               (unsigned long)d.arm_status, (unsigned long)d.uart_isr,
               (unsigned long)d.uart_error, (unsigned long)d.rx_state,
               (unsigned long)d.dma_ndtr,   (unsigned long)d.rx_events);

        if (d.arm_status != 0u)
        {
            printf("     -> RX DMA NEVER ARMED. Firmware fault, not wiring.\r\n");
        }
        else if (d.dma_ndtr == 1024u)
        {
            printf("     -> listening, but not one byte has ever arrived.\r\n");
            printf("        Sensor not transmitting, or PA10 not connected to SDA.\r\n");
        }
        if (d.uart_isr & 0x8u)  { printf("     -> ORE: bytes arrived and were dropped\r\n"); }
        if (d.uart_isr & 0x2u)  { printf("     -> FE: framing error, baud mismatch\r\n"); }
        if (d.uart_isr & 0x4u)  { printf("     -> NE: line noise\r\n"); }

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

#if IMUTEST_RVC
/*
 * BNO08x UART-RVC frame, 19 bytes:
 *   0,1  0xAA 0xAA header      9,10  x accel, mg
 *   2    index                11,12  y accel
 *   3,4  yaw,   0.01 deg      13,14  z accel
 *   5,6  pitch, 0.01 deg      15-17  reserved
 *   7,8  roll,  0.01 deg         18  checksum: sum of bytes 2..17
 */
static void rvc_run(void)
{
    uint8_t  f[19];
    uint16_t n = 0;
    uint32_t frames = 0, bad = 0, bytes = 0;
    uint32_t report = 0;

    printf("RVC mode: expecting AA AA framed 19-byte packets at 100 Hz\r\n\r\n");

    for (;;)
    {
        uint8_t b;
        HAL_StatusTypeDef st = HAL_UART_Receive(&huart1, &b, 1, 100);

        if (st != HAL_OK)
        {
            if (++report >= 10u)
            {
                report = 0;

                /*
                 * TIMEOUT and ERROR mean opposite things and must not be
                 * reported the same way. TIMEOUT is a genuinely silent line.
                 * ERROR means the peripheral rejected the read - a stale
                 * overrun flag, or a receive still owned by DMA - and no
                 * amount of rewiring fixes that.
                 */
                printf("  no data: HAL=%s isr=0x%08lX rxstate=0x%lX baud=%lu\r\n",
                       (st == HAL_TIMEOUT) ? "TIMEOUT (line is silent)"
                                           : ((st == HAL_BUSY) ? "BUSY (DMA still owns RX)"
                                                               : "ERROR"),
                       (unsigned long)huart1.Instance->ISR,
                       (unsigned long)huart1.RxState,
                       (unsigned long)huart1.Init.BaudRate);

                /* Clear sticky error flags so one glitch does not wedge every
                   subsequent read the way it did on the SHTP path. */
                __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                               UART_CLEAR_NEF | UART_CLEAR_OREF);
                huart1.ErrorCode = HAL_UART_ERROR_NONE;
            }
            continue;
        }

        bytes++;

        /* Resync on the header rather than trusting alignment. */
        if (n < 2u)
        {
            n = (b == 0xAAu) ? (uint16_t)(n + 1u) : 0u;
            f[0] = f[1] = 0xAAu;
            continue;
        }

        f[n++] = b;
        if (n < sizeof(f))
        {
            continue;
        }
        n = 0;

        uint8_t sum = 0;
        for (int i = 2; i < 18; i++)
        {
            sum = (uint8_t)(sum + f[i]);
        }

        if (sum != f[18])
        {
            bad++;
            continue;
        }

        frames++;
        if ((frames % 50u) != 0u)
        {
            continue;
        }

        int16_t yaw   = (int16_t)((uint16_t)f[3]  | ((uint16_t)f[4]  << 8));
        int16_t pitch = (int16_t)((uint16_t)f[5]  | ((uint16_t)f[6]  << 8));
        int16_t roll  = (int16_t)((uint16_t)f[7]  | ((uint16_t)f[8]  << 8));
        int16_t ax    = (int16_t)((uint16_t)f[9]  | ((uint16_t)f[10] << 8));
        int16_t ay    = (int16_t)((uint16_t)f[11] | ((uint16_t)f[12] << 8));
        int16_t az    = (int16_t)((uint16_t)f[13] | ((uint16_t)f[14] << 8));

        printf("yaw=%+7.2f pitch=%+7.2f roll=%+7.2f deg  "
               "a=%+6.3f %+6.3f %+6.3f g   frames=%lu bad=%lu bytes=%lu\r\n",
               (double)yaw * 0.01, (double)pitch * 0.01, (double)roll * 0.01,
               (double)ax * 0.001, (double)ay * 0.001, (double)az * 0.001,
               (unsigned long)frames, (unsigned long)bad, (unsigned long)bytes);

        BSP_LED_Toggle(LED_GREEN);
        BSP_LED_Off(LED_RED);
    }
}
#endif

void imutest_run(void)
{
#if IMUTEST_RVC
    rvc_run();
#endif

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

#if IMUTEST_LOOPBACK
        /* Transmit into our own RX pin 10x/s so bytes/s is a live number
           rather than a one-shot burst at startup. */
        if ((s_tick % 100u) == 0u)
        {
            imu_tx_test_pattern();
        }
#endif

        /*
         * Keep asking until reports actually arrive.
         *
         * Gating this on bytes was wrong: the BNO085 sends a short message at
         * boot and then waits, so 13 bytes land, the condition goes false, and
         * the requests stop forever - having been sent exactly once, 100 ms
         * after boot, which is before the sensor is ready to accept them.
         * Samples are what we want, so samples are what the retry watches.
         */
        if ((s_tick % 500u) == 0u)
        {
            imu_sample_t s;
            imu_get(&s);

            if (s.seq == 0u)
            {
                /*
                 * Follow the protocol's own order instead of firing commands
                 * blindly. The sensor sends an advertisement on channel 0 once
                 * it has finished booting; anything sent before that is
                 * discarded with no error, which is indistinguishable from a
                 * dead sensor. So: reset until the advertisement appears, and
                 * only then ask for reports.
                 */
                if (!imu_saw_advertisement())
                {
                    imu_soft_reset();
                }
                else
                {
                    imu_request_product_id();
                    imu_request_reports();
                }
            }
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
