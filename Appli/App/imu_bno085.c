#include "imu_bno085.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;

/*
 * BNO085 SHTP over UART. Transport framing is SLIP-like: 0x7E delimits a frame,
 * 0x7D escapes the next byte by XOR 0x20. Inside sits a 4-byte SHTP header
 * (length LSB, length MSB, channel, seq) whose length count includes itself.
 */
#define SHTP_FLAG            0x7Eu
#define SHTP_ESC             0x7Du
#define SHTP_ESC_XOR         0x20u
#define SHTP_PROTOCOL_ID     0x01u

#define SHTP_CH_EXECUTABLE   1u
#define SHTP_CH_CONTROL      2u
#define SHTP_CH_INPUT_REPORT 3u

/* Executable channel commands. */
#define SHTP_EXEC_RESET      1u

#define SH2_SET_FEATURE      0xFDu
#define SH2_GET_FEATURE_REQ  0xFEu
#define SH2_GET_FEATURE_RESP 0xFCu
#define SH2_PRODUCT_ID_REQ   0xF9u
#define SH2_PRODUCT_ID_RESP  0xF8u
#define SH2_BASE_TIMESTAMP   0xFBu
#define SH2_GYRO             0x02u
#define SH2_ROTATION_VECTOR  0x05u

/*
 * Report 0x01, "Calibrated Acceleration ... including gravity" (datasheet
 * 2.1.1) - NOT 0x04 linear acceleration, which has gravity removed.
 *
 * The filter propagates velocity as v += (R*a + g)*dt, which is Hartley
 * equation 50 and expects a to be specific force: what an accelerometer
 * physically reads. At rest that is +9.81 upward, gravity cancels it, and
 * velocity stays put. Feed it gravity-free data and a stationary robot reads
 * 0 + (-9.81), so the filter concludes it is in free fall and the velocity
 * estimate ramps downward forever.
 */
#define SH2_ACCELEROMETER    0x01u

#define IMU_REPORT_INTERVAL_US  2500u   /* 400 Hz - gyro and accelerometer */
#define IMU_QUAT_INTERVAL_US   10000u   /* 100 Hz - rotation vector          */

/*
 * Ask the sensor for its own quaternion at all.
 *
 * The estimator does not need it - it derives orientation from gyro and
 * contacts - and the Pi only sees it at 250 Hz. But the rotation vector is a
 * fusion product, so producing it costs the sensor far more than a raw report
 * and appears to be what starves the accelerometer: measured 378/378/216 with
 * all three at 400 Hz, then 105/421/158 after dropping the quaternion to
 * 100 Hz. Total throughput FELL, so this is not simple bandwidth.
 *
 * Set to 0 to drop it entirely and see what the accelerometer does with the
 * fusion switched off. The Pi still gets fused_quat from our own filter, which
 * is fused with leg kinematics and is the better estimate anyway.
 */
#define IMU_ENABLE_QUAT         1

/* Gap between consecutive SET_FEATURE frames, milliseconds. */
#define IMU_REQUEST_GAP_MS      5u

/*
 * Minimum spacing between BYTES sent to the sensor, microseconds.
 *
 * BNO08X datasheet 1.2.3.1: "Bytes sent from the host to the BNO08X must be
 * separated by at least 100us. Bytes sent from the BNO to the host have no
 * extra spacing."
 *
 * At 3 Mbaud a byte takes 3.3 us, so a normal back-to-back transmission is
 * thirty times too fast and the sensor mangles everything after the first
 * byte. That is exactly what we saw: its own transmissions arrive perfectly
 * while every frame we send comes back as an SHTP error, because the rule is
 * one-directional. A 24-byte frame now takes ~2.4 ms instead of 80 us, which
 * costs nothing - these are configuration frames, not the data path.
 */
#define IMU_TX_BYTE_GAP_US      120u

/*
 * Listen this long before transmitting anything at all, milliseconds.
 *
 * Datasheet 5.2.1: after power up or reset the BNO08X sends its SHTP
 * advertisement unprompted, followed by a reset message on channel 1 and an
 * initialization message on channel 2. None of that depends on the host.
 * Startup timing (6.5.3) puts internal initialisation at 90 ms.
 *
 * So a silent window at boot answers a question nothing else can: does the
 * sensor come up and talk on its own? If bytes arrive here, the part is
 * healthy and only our conversation is at fault. If nothing arrives in a full
 * second after a POWER CYCLE, the sensor is not running, and no protocol work
 * will change that.
 */
#define IMU_LISTEN_FIRST_MS     1000u

/*
 * Hardware reset line. Set to 1 and wire the breakout's RST pin to PA8.
 *
 * Pulling RST low is the only thing proven to recover this part from its wedged
 * power-up state - the software reset has to travel over the very link that is
 * broken, so it cannot be relied on to fix it. One wire makes recovery
 * unconditional, and PA8 is free and sits next to PA9/PA10 on the header.
 */
#define IMU_USE_RST_PIN   0
#define IMU_RST_PORT      GPIOA
#define IMU_RST_PIN       GPIO_PIN_8

#define IMU_RX_BUF   1024u
#define IMU_FRAME_MAX 512u

static uint8_t s_dma[IMU_RX_BUF] __attribute__((section("noncacheable_buffer"), aligned(32)));

static uint16_t s_rd;
static volatile uint32_t s_rx_events;

/*
 * Diagnostic counters. These separate three very different failures that all
 * look identical from the outside ("no IMU data"):
 *   bytes  == 0  -> nothing on the wire at all: TX/RX swapped, no power, no ground
 *   bytes  >  0 but frames == 0 -> bytes arriving but no valid SHTP framing:
 *                                  wrong baud rate, or the IMU is in UART-RVC
 *                                  mode rather than SHTP
 *   frames >  0 but samples == 0 -> framing fine, but no report we asked for:
 *                                  the SET_FEATURE commands did not take
 */
static volatile uint32_t s_bytes;
static volatile uint32_t s_frames;
static uint32_t          s_arm_status = 0xFFu;   /* until imu_init runs */
static volatile uint32_t s_errors;               /* UART errors recovered from */
static volatile uint32_t s_last_error;

/*
 * Frames seen per SHTP channel. Which channel answers says what the sensor
 * made of a request: 2 is a control response, meaning SET_FEATURE was heard;
 * 3 is actual sensor data. Traffic on neither means it is not listening.
 */
static volatile uint16_t s_chan_frames[8];
static volatile uint8_t  s_last_control;
/*
 * Per-report counters. "samples" counts FRAMES, and a frame carries whichever
 * report the sensor had ready, so it sums three different streams and reads
 * far higher than any one of them. Only these say what each report's real rate
 * is - which matters, because the datasheet (6.9) warns that sensors cannot
 * all run at their individual maximum simultaneously.
 */
static volatile uint32_t s_n_rv;      /* rotation vector  */
static volatile uint32_t s_n_accel;   /* linear accel     */
static volatile uint32_t s_n_gyro;    /* calibrated gyro  */

static volatile uint8_t  s_saw_advert;   /* sensor finished booting */
static volatile uint8_t  s_saw_reset;    /* executable channel reset notice */
static uint8_t           s_ctrl_payload[64];
static uint16_t          s_ctrl_len;

/* First bytes ever received, kept verbatim for inspection. */
#define IMU_SNAP_LEN 64u
static uint8_t  s_snap[IMU_SNAP_LEN];
static uint16_t s_snap_len;
static uint8_t  s_txsnap[IMU_SNAP_LEN];
static uint16_t s_txsnap_len;

static uint8_t  s_frame[IMU_FRAME_MAX];
static uint16_t s_flen;
static uint8_t  s_esc;
static uint8_t  s_state;
/*
 * SHTP sequence numbers are PER CHANNEL, not global. One shared counter means
 * the first message on channel 2 carries whatever number channel 1 left
 * behind, so the sensor sees a stream that starts mid-sequence and every frame
 * after it is out of order. Each channel counts independently.
 */
static uint8_t  s_tx_seq[8];

enum { ST_WAIT_START = 0, ST_WAIT_PROTO, ST_IN_FRAME };

static imu_sample_t s_sample;

/*
 * Microsecond delay from the cycle counter. TIM2 is not started until after
 * imu_init() runs, and HAL_Delay only resolves to 1 ms, so neither can pace
 * bytes here.
 */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000u) * us;

    while ((DWT->CYCCNT - start) < cycles)
    {
        __NOP();
    }
}

/*
 * Send to the sensor one byte at a time, spaced per the datasheet. Every
 * transmission to the BNO08X must go through here.
 */
static void uart_tx_spaced(const uint8_t *p, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)&p[i], 1u, 10u);
        delay_us(IMU_TX_BYTE_GAP_US);
    }
}

static float q_to_float(const uint8_t *p, int qpoint)
{
    int16_t raw = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    return (float)raw / (float)(1u << qpoint);
}

static void shtp_send(uint8_t channel, const uint8_t *payload, uint16_t plen)
{
    uint8_t  out[80];
    uint16_t n = 0;
    uint8_t  hdr[4];
    uint16_t total = (uint16_t)(plen + 4u);

    hdr[0] = (uint8_t)(total & 0xFFu);
    hdr[1] = (uint8_t)((total >> 8) & 0xFFu);
    hdr[2] = channel;
    hdr[3] = s_tx_seq[channel & 7u]++;

    out[n++] = SHTP_FLAG;
    out[n++] = SHTP_PROTOCOL_ID;

    for (uint16_t i = 0; i < 4u + plen; i++)
    {
        uint8_t b = (i < 4u) ? hdr[i] : payload[i - 4u];

        if ((b == SHTP_FLAG) || (b == SHTP_ESC))
        {
            out[n++] = SHTP_ESC;
            out[n++] = (uint8_t)(b ^ SHTP_ESC_XOR);
        }
        else
        {
            out[n++] = b;
        }
    }

    out[n++] = SHTP_FLAG;

    /* Keep the first frame we ever send. Everything about the receive path can
       be checked against the bytes on the wire; without this the transmit path
       is the one half still being taken on trust. */
    if (s_txsnap_len == 0u)
    {
        s_txsnap_len = (n < IMU_SNAP_LEN) ? n : IMU_SNAP_LEN;
        for (uint16_t i = 0; i < s_txsnap_len; i++)
        {
            s_txsnap[i] = out[i];
        }
    }

    uart_tx_spaced(out, n);
}

static void imu_enable_report(uint8_t report_id, uint32_t interval_us)
{
    uint8_t p[17];

    memset(p, 0, sizeof(p));
    p[0] = SH2_SET_FEATURE;
    p[1] = report_id;
    p[5] = (uint8_t)(interval_us & 0xFFu);
    p[6] = (uint8_t)((interval_us >> 8) & 0xFFu);
    p[7] = (uint8_t)((interval_us >> 16) & 0xFFu);
    p[8] = (uint8_t)((interval_us >> 24) & 0xFFu);

    shtp_send(SHTP_CH_CONTROL, p, sizeof(p));
}

/*
 * Ask what the sensor actually did with a Set Feature.
 *
 * Set Feature is fire-and-forget - the sensor is not required to acknowledge
 * it, so silence afterwards is ambiguous: rejected outright, accepted but
 * clamped to a rate we then failed to notice, or accepted and enabled with the
 * reports going somewhere we are not looking. The Get Feature Response carries
 * the interval the sensor is actually using, which distinguishes all three. An
 * interval of zero means the report is off.
 */
void imu_request_feature_status(uint8_t report_id)
{
    uint8_t p[2];

    p[0] = SH2_GET_FEATURE_REQ;
    p[1] = report_id;

    shtp_send(SHTP_CH_CONTROL, p, sizeof(p));
}

static void parse_reports(const uint8_t *p, uint16_t n)
{
    uint16_t i = 0;
    uint8_t  updated = 0;

    while (i < n)
    {
        uint8_t id = p[i];

        if (id == SH2_BASE_TIMESTAMP)
        {
            i += 5u;
        }
        else if ((id == SH2_ROTATION_VECTOR) && ((i + 14u) <= n))
        {
            s_sample.quat[1] = q_to_float(&p[i + 4], 14);
            s_sample.quat[2] = q_to_float(&p[i + 6], 14);
            s_sample.quat[3] = q_to_float(&p[i + 8], 14);
            s_sample.quat[0] = q_to_float(&p[i + 10], 14);
            i += 14u;
            s_n_rv++;
            updated = 1;
        }
        else if ((id == SH2_ACCELEROMETER) && ((i + 10u) <= n))
        {
            s_sample.accel[0] = q_to_float(&p[i + 4], 8);
            s_sample.accel[1] = q_to_float(&p[i + 6], 8);
            s_sample.accel[2] = q_to_float(&p[i + 8], 8);
            i += 10u;
            s_n_accel++;
            updated = 1;
        }
        else if ((id == SH2_GYRO) && ((i + 10u) <= n))
        {
            s_sample.gyro[0] = q_to_float(&p[i + 4], 9);
            s_sample.gyro[1] = q_to_float(&p[i + 6], 9);
            s_sample.gyro[2] = q_to_float(&p[i + 8], 9);
            i += 10u;
            s_n_gyro++;
            updated = 1;
        }
        else
        {
            break;
        }
    }

    if (updated)
    {
        s_sample.seq++;
    }
}

static void parse_shtp(const uint8_t *f, uint16_t n)
{
    if (n < 4u)
    {
        return;
    }

    s_frames++;
    s_chan_frames[f[2] & 7u]++;

    /* Remember what the sensor replied on the control channel. 0xF8 is a
       Product ID response, 0xFC a Get Feature response - either proves a
       command was understood rather than merely received. */
    /*
     * Channel 0 payload 0x00 is the advertisement the sensor sends once after
     * every reset; 0x01 is its error list. Channel 1 payload 0x01 is the
     * reset-complete notice. Seeing the advertisement is the only positive
     * proof the sensor has finished booting and is ready for commands - before
     * it, anything we send is discarded silently.
     */
    if ((f[2] == 0u) && (n > 4u) && (f[4] == 0x00u))
    {
        s_saw_advert = 1;
    }
    if ((f[2] == SHTP_CH_EXECUTABLE) && (n > 4u))
    {
        s_saw_reset = 1;
    }

    if ((f[2] == SHTP_CH_CONTROL) && (n > 4u))
    {
        s_last_control = f[4];

        /* Keep the whole reply, not just its ID. A Get Feature Response says
           which report it is about and the interval actually in force - the
           only direct evidence of what the sensor did with our request. */
        uint16_t c = (uint16_t)(n - 4u);
        s_ctrl_len = (c < IMU_SNAP_LEN) ? c : IMU_SNAP_LEN;
        for (uint16_t i = 0; i < s_ctrl_len; i++)
        {
            s_ctrl_payload[i] = f[4u + i];
        }
    }

    if (f[2] != SHTP_CH_INPUT_REPORT)
    {
        return;
    }

    parse_reports(&f[4], (uint16_t)(n - 4u));
}

static void feed(uint8_t b)
{
    switch (s_state)
    {
    case ST_WAIT_START:
        if (b == SHTP_FLAG)
        {
            s_state = ST_WAIT_PROTO;
        }
        break;

    case ST_WAIT_PROTO:
        if (b == SHTP_FLAG)
        {
            break;
        }
        if (b == SHTP_PROTOCOL_ID)
        {
            s_state = ST_IN_FRAME;
            s_flen  = 0;
            s_esc   = 0;
        }
        else
        {
            s_state = ST_WAIT_START;
        }
        break;

    case ST_IN_FRAME:
        if (b == SHTP_FLAG)
        {
            if (s_flen > 0u)
            {
                parse_shtp(s_frame, s_flen);
            }
            s_state = ST_WAIT_PROTO;
            s_flen  = 0;
            s_esc   = 0;
            break;
        }

        if (s_esc)
        {
            s_frame[s_flen++] = (uint8_t)(b ^ SHTP_ESC_XOR);
            s_esc = 0;
        }
        else if (b == SHTP_ESC)
        {
            s_esc = 1;
        }
        else
        {
            s_frame[s_flen++] = b;
        }

        if (s_flen >= IMU_FRAME_MAX)
        {
            s_state = ST_WAIT_START;
            s_flen  = 0;
            s_esc   = 0;
        }
        break;

    default:
        s_state = ST_WAIT_START;
        break;
    }
}

void imu_init(void)
{
    dwt_init();

    memset(&s_sample, 0, sizeof(s_sample));
    s_sample.quat[0] = 1.0f;

    s_rd    = 0;
    s_rx_events = 0;
    s_bytes     = 0;
    s_frames    = 0;
    s_flen  = 0;
    s_esc   = 0;
    s_state = ST_WAIT_START;
    memset((void *)s_tx_seq, 0, sizeof(s_tx_seq));

#if IMU_USE_RST_PIN
    {
        GPIO_InitTypeDef r = {0};

        printf("imu: driving RST on PA8\r\n");

        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* Idle high - RST is active low, so a low here holds the sensor in
           reset permanently. Set the level before switching to an output so
           the pin never glitches low on the way. */
        HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_SET);

        r.Pin   = IMU_RST_PIN;
        r.Mode  = GPIO_MODE_OUTPUT_PP;
        r.Pull  = GPIO_NOPULL;
        r.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(IMU_RST_PORT, &r);
    }
#endif

    /*
     * Drive PA9 as hard as the pin allows. CubeMX sets it to HIGH speed, which
     * is marginal for 3 Mbaud through the breakout's level shifter: a slow
     * rising edge is still climbing when the sensor samples the bit, so it
     * reads the wrong value and rejects the frame. Costs nothing to raise, and
     * the receive direction - which the sensor drives - already works, so the
     * asymmetry points here.
     */
    {
        GPIO_InitTypeDef g = {0};

        g.Pin       = GPIO_PIN_9;
        g.Mode      = GPIO_MODE_AF_PP;
        g.Pull      = GPIO_PULLUP;
        g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        g.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &g);
    }

    /* Keep the status. If arming fails there is no receiver at all, and every
       downstream counter reads zero for a reason that has nothing to do with
       the sensor - which is indistinguishable from a wiring fault unless the
       failure is recorded here. */
    s_arm_status = (uint32_t)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_dma, IMU_RX_BUF);

    /*
     * The BNO085 needs a moment after power-up before it accepts control
     * reports. 100 ms was optimistic - the part boots, runs self-test and
     * sends its advertisement first, and a request that lands early is simply
     * discarded with no error. The test re-sends anyway, so this only has to
     * be long enough for the common case.
     */
    HAL_Delay(100);

    /*
     * Reset first, unconditionally. A sensor already running is put back into
     * a known state; a sensor wedged from power-up is recovered. Doing it
     * every time costs half a second at boot and removes an entire class of
     * failure that is otherwise indistinguishable from bad wiring.
     */
    /* Resync before resetting: a parser stuck mid-frame will swallow the reset
       command itself, which is precisely the deadlock seen on hardware. */
    printf("imu: resync\r\n");
    imu_resync();
    HAL_Delay(50);

    printf("imu: reset\r\n");
    imu_soft_reset();
    HAL_Delay(500);        /* boot, self-test, and the advertisement */

    printf("imu: requesting reports\r\n");
    imu_request_reports();
    printf("imu: init done\r\n");
}

/*
 * Flush the sensor's frame parser with a burst of bare delimiters.
 *
 * This is the likeliest reason the part comes up wedged. PA9 is an undriven
 * input from the moment power arrives until MX_USART1_UART_Init() runs, and
 * the sensor - which boots in the same instant - sees that floating line as
 * data. It starts assembling a frame from noise and never receives a
 * terminator, so its parser sits mid-frame forever. Everything we send
 * afterwards is swallowed as continuation of that phantom frame, which is
 * exactly what we observe: valid commands, including the reset that would fix
 * it, all answered with errors.
 *
 * 0x7E is the frame delimiter. A run of them terminates whatever the parser
 * thinks it is in the middle of and leaves it at a frame boundary. They cost
 * nothing if the parser was already idle - back-to-back delimiters are empty
 * frames, which SHTP ignores.
 */
void imu_resync(void)
{
    uint8_t flags[64];

    for (uint16_t i = 0; i < sizeof(flags); i++)
    {
        flags[i] = SHTP_FLAG;
    }

    uart_tx_spaced(flags, sizeof(flags));

    /* Our own parser may equally be mid-frame on the sensor's boot noise. */
    s_flen  = 0;
    s_esc   = 0;
    s_state = ST_WAIT_START;
}

/*
 * Reset the sensor over the executable channel.
 *
 * The BNO085 can come out of power-up wedged: it answers every command with an
 * SHTP error, never sends the advertisement it is supposed to send at boot,
 * and stays that way indefinitely. Nothing in the SHTP conversation recovers
 * it, because the conversation is what is broken - the part has to be reset.
 *
 * Pulling RST low does it, but that needs a wire we do not have. One byte on
 * channel 1 does the same thing in software, so the firmware can recover on
 * its own rather than depending on someone reaching for a jumper.
 */
void imu_soft_reset(void)
{
#if IMU_USE_RST_PIN
    /* Active low. 10 ms is far longer than the part needs, and this only runs
       at startup or after a failure, so there is no reason to be tight. */
    HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(IMU_RST_PORT, IMU_RST_PIN, GPIO_PIN_SET);
#else
    uint8_t p = SHTP_EXEC_RESET;

    shtp_send(SHTP_CH_EXECUTABLE, &p, 1u);
#endif
}

/*
 * The simplest command SH-2 defines: two bytes, no parameters, and the reply
 * is a Product ID Response on channel 2.
 *
 * Set Feature is 17 bytes with a dozen fields, so an error against it could be
 * any one of them. This has nothing to get wrong. If it is rejected too, the
 * fault is in the transport - bytes arriving corrupted, or a sensor that will
 * not accept commands at all - and no amount of adjusting Set Feature will
 * help. If it succeeds while Set Feature fails, the problem really is in that
 * command's contents.
 */
void imu_request_product_id(void)
{
    uint8_t p[2];

    p[0] = SH2_PRODUCT_ID_REQ;
    p[1] = 0;   /* reserved */

    shtp_send(SHTP_CH_CONTROL, p, sizeof(p));
}

void imu_request_reports(void)
{
    /*
     * Space these out. Three frames back to back at 3 Mbaud arrive within
     * ~260 us of each other, and the BNO085 answers every one of them with an
     * SHTP error - the error list grows by exactly one entry per frame sent,
     * which is what a part that cannot keep up looks like. A few milliseconds
     * between requests costs nothing here; this runs at boot and then only
     * while no reports are arriving.
     */
    /*
     * Gyro and accelerometer drive the filter's prediction step, so they get
     * the full rate. The quaternion does not - the estimator derives its own
     * orientation, and the sensor's is only passed through to the Pi, which
     * runs at 250 Hz. Measured with all three at 400 Hz the sensor delivered
     * 378/378/216: it cannot sustain three reports at maximum simultaneously
     * (datasheet 6.9), and the accelerometer was the one it starved.
     */
#if IMU_ENABLE_QUAT
    imu_enable_report(SH2_ROTATION_VECTOR, IMU_QUAT_INTERVAL_US);
    HAL_Delay(IMU_REQUEST_GAP_MS);
#endif
    imu_enable_report(SH2_ACCELEROMETER,   IMU_REPORT_INTERVAL_US);
    HAL_Delay(IMU_REQUEST_GAP_MS);
    imu_enable_report(SH2_GYRO,            IMU_REPORT_INTERVAL_US);
    HAL_Delay(IMU_REQUEST_GAP_MS);

    /* Then ask what actually took. Set Feature is fire-and-forget; this is the
       only way to learn whether the sensor enabled anything. */
    imu_request_feature_status(SH2_ACCELEROMETER);
}

uint16_t imu_tx_snapshot(uint8_t *out, uint16_t max)
{
    uint16_t n = (s_txsnap_len < max) ? s_txsnap_len : max;

    for (uint16_t i = 0; i < n; i++)
    {
        out[i] = s_txsnap[i];
    }

    return n;
}

/*
 * The DMA controller is the only thing that knows exactly how much it has
 * written, so ask it directly instead of trusting the HAL's event callback.
 *
 * The callback cannot be used as a write index: in circular mode the HAL
 * reports Size == RxXferSize (1024) on transfer-complete, but s_rd only ever
 * counts 0..1023, so "while (s_rd != 1024)" never terminates and the whole
 * main loop hangs.
 *
 * CBR1.BNDT counts *down* the bytes still to be written in the current block,
 * so bytes_written = IMU_RX_BUF - BNDT. It reloads to IMU_RX_BUF on wrap.
 */
static uint16_t imu_write_index(void)
{
    if (huart1.hdmarx == NULL)
    {
        return s_rd;
    }

    uint32_t remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);

    /* remaining == 0 means the block just finished and is about to reload;
       remaining > IMU_RX_BUF means the channel is not running yet. Both map
       to index 0, which makes the ring drain cleanly up to the buffer end. */
    if ((remaining == 0u) || (remaining > IMU_RX_BUF))
    {
        return 0u;
    }

    return (uint16_t)(IMU_RX_BUF - remaining);
}

/*
 * Kept so main.c's HAL_UARTEx_RxEventCallback still has somewhere to go.
 * The reported size is deliberately ignored - see imu_write_index() above.
 */
void imu_on_rx_event(uint16_t size)
{
    (void)size;
    s_rx_events++;
}

void imu_service(void)
{
    uint16_t wr = imu_write_index();

    while (s_rd != wr)
    {
        uint8_t b = s_dma[s_rd];

        s_bytes++;

        /* Keep the first bytes verbatim. Rates and counters say whether data
           is arriving; only the bytes themselves say WHAT is arriving, and at
           a wrong baud rate every derived number looks like a wiring fault. */
        if (s_snap_len < IMU_SNAP_LEN)
        {
            s_snap[s_snap_len++] = b;
        }

        feed(b);
        s_rd = (uint16_t)((s_rd + 1u) % IMU_RX_BUF);
    }
}

uint16_t imu_channel_frames(uint8_t ch)
{
    return s_chan_frames[ch & 7u];
}

void imu_report_counts(uint32_t *rv, uint32_t *accel, uint32_t *gyro)
{
    *rv    = s_n_rv;
    *accel = s_n_accel;
    *gyro  = s_n_gyro;
}

uint8_t imu_saw_advertisement(void)
{
    return s_saw_advert;
}

uint8_t imu_saw_reset(void)
{
    return s_saw_reset;
}

uint8_t imu_last_control_response(void)
{
    return s_last_control;
}

uint16_t imu_control_payload(uint8_t *out, uint16_t max)
{
    uint16_t n = (s_ctrl_len < max) ? s_ctrl_len : max;

    for (uint16_t i = 0; i < n; i++)
    {
        out[i] = s_ctrl_payload[i];
    }

    return n;
}

uint16_t imu_snapshot(uint8_t *out, uint16_t max)
{
    uint16_t n = (s_snap_len < max) ? s_snap_len : max;

    for (uint16_t i = 0; i < n; i++)
    {
        out[i] = s_snap[i];
    }

    return n;
}

uint32_t imu_rx_events(void)
{
    return s_rx_events;
}

uint32_t imu_rx_bytes(void)
{
    return s_bytes;
}

uint32_t imu_frames(void)
{
    return s_frames;
}

void imu_get(imu_sample_t *out)
{
    *out = s_sample;
}

/*
 * Restart reception after a UART error.
 *
 * Without this the HAL's weak do-nothing stub runs, the DMA stays aborted, and
 * one noise glitch kills the IMU permanently - RxState sits at READY and not
 * another byte is ever received. On a robot that is the difference between a
 * momentary blip and a dead sensor for the rest of the run.
 *
 * Overrides __weak HAL_UART_ErrorCallback.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    s_errors++;
    s_last_error = huart->ErrorCode;

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                 UART_CLEAR_NEF | UART_CLEAR_OREF);

    /* Re-arming rewinds the DMA to the start of the buffer, so the read index
       has to follow it. A half-assembled frame is worthless after a dropout,
       so the parser restarts hunting for a delimiter too. */
    s_rd    = 0;
    s_flen  = 0;
    s_esc   = 0;
    s_state = ST_WAIT_START;

    s_arm_status = (uint32_t)HAL_UARTEx_ReceiveToIdle_DMA(huart, s_dma, IMU_RX_BUF);
}

uint32_t imu_errors(void)
{
    return s_errors;
}

void imu_diag(imu_diag_t *out)
{
    out->arm_status = s_arm_status;
    out->uart_isr   = huart1.Instance->ISR;
    out->uart_error = s_last_error ? s_last_error : huart1.ErrorCode;
    out->errors     = s_errors;
    out->rx_state   = (uint32_t)huart1.RxState;
    out->rx_events  = s_rx_events;

    /* NDTR counts down as the DMA fills the buffer and reloads at the wrap.
       A value frozen at IMU_RX_BUF means not one byte has ever arrived. */
    out->dma_ndtr = (huart1.hdmarx != NULL)
                        ? __HAL_DMA_GET_COUNTER(huart1.hdmarx)
                        : 0xFFFFFFFFu;
}

/* 0x55/0xAA alternate every bit - the hardest pattern for a marginal link to
   carry, so it fails at a lower bit rate than real data would. */
static const uint8_t s_pattern[16] = {
    0x55, 0xAA, 0x55, 0xAA, 0x00, 0xFF, 0x00, 0xFF,
    0x7E, 0x01, 0x7E, 0x01, 0x12, 0x34, 0x56, 0x78,
};

void imu_tx_test_pattern(void)
{
    uart_tx_spaced(s_pattern, sizeof(s_pattern));
}

uint32_t imu_loopback_check(uint32_t *out_total)
{
    /*
     * Compare what came back against what went out, byte for byte.
     *
     * Counting bytes is not enough: a link that corrupts data still delivers
     * the right NUMBER of bytes, so a byte count looks perfect while every
     * value is wrong. That is exactly the failure we are chasing - the sensor
     * receives our frames and rejects their contents - and only a comparison
     * can see it.
     */
    uint32_t bad = 0;
    uint16_t n   = s_snap_len;

    for (uint16_t i = 0; i < n; i++)
    {
        if (s_snap[i] != s_pattern[i % sizeof(s_pattern)])
        {
            bad++;
        }
    }

    if (out_total != NULL)
    {
        *out_total = n;
    }

    return bad;
}
