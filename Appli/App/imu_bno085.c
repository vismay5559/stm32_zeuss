#include "imu_bno085.h"
#include "main.h"
#include <string.h>

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

#define SHTP_CH_CONTROL      2u
#define SHTP_CH_INPUT_REPORT 3u

#define SH2_SET_FEATURE      0xFDu
#define SH2_BASE_TIMESTAMP   0xFBu
#define SH2_GYRO             0x02u
#define SH2_LINEAR_ACCEL     0x04u
#define SH2_ROTATION_VECTOR  0x05u

#define IMU_REPORT_INTERVAL_US  2500u   /* 400 Hz */

#define IMU_RX_BUF   1024u
#define IMU_FRAME_MAX 512u

static uint8_t s_dma[IMU_RX_BUF] __attribute__((section("noncacheable_buffer"), aligned(32)));

static uint16_t s_rd;
static volatile uint32_t s_rx_events;

static uint8_t  s_frame[IMU_FRAME_MAX];
static uint16_t s_flen;
static uint8_t  s_esc;
static uint8_t  s_state;
static uint8_t  s_tx_seq;

enum { ST_WAIT_START = 0, ST_WAIT_PROTO, ST_IN_FRAME };

static imu_sample_t s_sample;

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
    hdr[3] = s_tx_seq++;

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

    HAL_UART_Transmit(&huart1, out, n, 20);
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
            updated = 1;
        }
        else if ((id == SH2_LINEAR_ACCEL) && ((i + 10u) <= n))
        {
            s_sample.accel[0] = q_to_float(&p[i + 4], 8);
            s_sample.accel[1] = q_to_float(&p[i + 6], 8);
            s_sample.accel[2] = q_to_float(&p[i + 8], 8);
            i += 10u;
            updated = 1;
        }
        else if ((id == SH2_GYRO) && ((i + 10u) <= n))
        {
            s_sample.gyro[0] = q_to_float(&p[i + 4], 9);
            s_sample.gyro[1] = q_to_float(&p[i + 6], 9);
            s_sample.gyro[2] = q_to_float(&p[i + 8], 9);
            i += 10u;
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
    memset(&s_sample, 0, sizeof(s_sample));
    s_sample.quat[0] = 1.0f;

    s_rd    = 0;
    s_rx_events = 0;
    s_flen  = 0;
    s_esc   = 0;
    s_state = ST_WAIT_START;
    s_tx_seq = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_dma, IMU_RX_BUF);

    /* The BNO085 needs a moment after power-up before it accepts control reports. */
    HAL_Delay(100);

    imu_enable_report(SH2_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US);
    imu_enable_report(SH2_LINEAR_ACCEL,    IMU_REPORT_INTERVAL_US);
    imu_enable_report(SH2_GYRO,            IMU_REPORT_INTERVAL_US);
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
        feed(s_dma[s_rd]);
        s_rd = (uint16_t)((s_rd + 1u) % IMU_RX_BUF);
    }
}

uint32_t imu_rx_events(void)
{
    return s_rx_events;
}

void imu_get(imu_sample_t *out)
{
    *out = s_sample;
}
