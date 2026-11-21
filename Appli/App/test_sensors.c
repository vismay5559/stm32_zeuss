#include "test_sensors.h"
#include "critical.h"
#include "imu_bno085.h"
#include "link_proto.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * Every sensor at once: both CAN buses, the four spring encoders, the IMU and
 * the two foot switches.
 *
 * READ ONLY. Nothing is sent to a drive - no SDO, no state request, no
 * setpoint - so no axis can be armed from this mode. The FDCAN filters accept
 * everything and the controllers only listen. The IMU is the one thing we
 * transmit to, and only the SHTP requests that make it start reporting.
 *
 * One status block per second on the ST-LINK console (115200 8N1). Each line
 * ends in OK or says what is wrong, so the block can be read top to bottom
 * without decoding hex.
 */

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

/* ===================================================================== */
/*  CAN - receive only                                                    */
/* ===================================================================== */

/* CANSimple: standard ID = node << 5 | command. See act_odrive.c. */
#define CMD_HEARTBEAT  0x01u
#define CMD_ENCODER    0x09u
#define CAN_NODES      64u

/* Expected rates once the CAN_CFG rates are saved: heartbeat 10 ms, encoder
   1 ms. A drive still on the factory 100 ms / 10 ms shows up as LOW RATE. */
#define HB_MIN_HZ      80u
#define ENC_MIN_HZ     800u

static const char *const s_joint[4] = { "hip pitch", "hip roll", "knee", "ankle" };

typedef struct
{
    uint32_t frames;
    uint32_t heartbeats;
    uint32_t encoders;
    uint32_t error;
    float    position;
    uint8_t  state;
} can_node_t;

/* Written by the FDCAN interrupts, copied with interrupts masked. */
static can_node_t s_can[2][CAN_NODES];

static uint32_t le_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void can_start(FDCAN_HandleTypeDef *h, unsigned bus)
{
    FDCAN_FilterTypeDef f = {0};

    /* Every standard ID, so a drive on the wrong node ID still shows up
       instead of looking like a missing drive. */
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0u;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0u;
    f.FilterID2    = 0x7FFu;

    if (HAL_FDCAN_ConfigFilter(h, &f) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(h) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u)
            != HAL_OK)
    {
        printf("CAN bus %u: setup FAILED\r\n", bus);
    }
}

void sensors_on_can_rx(FDCAN_HandleTypeDef *hfdcan)
{
    unsigned bus = (hfdcan->Instance == FDCAN2) ? 1u : 0u;
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[64];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK)
        {
            return;
        }
        if (hdr.IdType != FDCAN_STANDARD_ID)
        {
            continue;
        }

        can_node_t *n = &s_can[bus][(hdr.Identifier >> 5) & 0x3Fu];
        uint32_t cmd = hdr.Identifier & 0x1Fu;
        n->frames++;

        if (hdr.DataLength != FDCAN_DLC_BYTES_8)
        {
            continue;
        }
        if (cmd == CMD_HEARTBEAT)
        {
            n->heartbeats++;
            n->error = le_u32(data);
            n->state = data[4];
        }
        else if (cmd == CMD_ENCODER)
        {
            uint32_t bits = le_u32(data);
            n->encoders++;
            memcpy(&n->position, &bits, sizeof(bits));
        }
    }
}

/* Returns the number of drives that look healthy. */
static unsigned can_report(void)
{
    static can_node_t prev[2][CAN_NODES];
    can_node_t now[2][CAN_NODES];
    unsigned good = 0u;

    uint32_t pm = critical_enter();
    memcpy(now, s_can, sizeof(now));
    critical_exit(pm);

    for (unsigned bus = 0u; bus < 2u; bus++)
    {
        for (unsigned node = 1u; node < CAN_NODES; node++)
        {
            can_node_t *c = &now[bus][node];
            can_node_t *p = &prev[bus][node];
            uint32_t frames = c->frames - p->frames;
            uint32_t hb     = c->heartbeats - p->heartbeats;
            uint32_t enc    = c->encoders - p->encoders;

            if (node > 4u)
            {
                /* Only worth a line if something is actually there. */
                if (frames != 0u)
                {
                    printf("  CAN %s node %-2u  %lu frames/s  <-- UNEXPECTED NODE ID\r\n",
                           bus == 0u ? "L" : "R", node, (unsigned long)frames);
                }
                *p = *c;
                continue;
            }

            printf("  CAN %s %-9s ", bus == 0u ? "L" : "R", s_joint[node - 1u]);
            if (frames == 0u)
            {
                printf("SILENT  <-- no power, wiring, or wrong node ID\r\n");
            }
            else
            {
                const char *verdict = "OK";
                if (c->error != 0u)               verdict = "<-- AXIS ERROR";
                else if (isnan(c->position))      verdict = "<-- position NaN (motor encoder not ready)";
                else if (hb < HB_MIN_HZ || enc < ENC_MIN_HZ)
                                                  verdict = "<-- LOW RATE (rates not saved?)";
                else                              good++;

                printf("hb=%3lu/s enc=%4lu/s state=%u err=0x%08lX pos=%+9.4f turns  %s\r\n",
                       (unsigned long)hb, (unsigned long)enc, (unsigned)c->state,
                       (unsigned long)c->error, (double)c->position, verdict);
            }
            *p = *c;
        }
    }
    return good;
}

/* ===================================================================== */
/*  Spring encoders - polled SPI at 1 MHz                                 */
/* ===================================================================== */

/*
 * Polled at 1 MHz from HSI, exactly as the ENC_SCAN bench mode does, because
 * that is the configuration the four sensors have been proven on. The robot's
 * DMA driver runs SPI1 much faster; this test is about the sensors, not that.
 */

#define AS_ERRFL_READ   0x4001u   /* read 0x0001, even parity */
#define AS_DIAAGC_READ  0xFFFCu   /* read 0x3FFC, even parity */
#define AS_ANGLE_READ   0xFFFFu   /* read 0x3FFF, even parity */
#define AS_NOP_READ     0xC000u   /* read 0x0000 - NOT 0x0000, which is a WRITE */

static const char *const s_enc_name[NEXUS_NUM_ENCODERS] = {
    "left hip", "left knee", "right hip", "right knee"
};

static const struct { GPIO_TypeDef *port; uint16_t pin; } s_cs[NEXUS_NUM_ENCODERS] = {
    { enc_cs_l_hip_GPIO_Port,  enc_cs_l_hip_Pin  },
    { enc_cs_l_knee_GPIO_Port, enc_cs_l_knee_Pin },
    { enc_cs_r_hip_GPIO_Port,  enc_cs_r_hip_Pin  },
    { enc_cs_r_knee_GPIO_Port, enc_cs_r_knee_Pin },
};

typedef struct
{
    uint32_t reads;       /* complete transactions */
    uint32_t bad;         /* any frame with bad parity, EF set, or ERRFL != 0 */
    uint32_t zero;        /* every word 0x0000: MISO held low, chip not answering */
    uint32_t ones;        /* every word 0xFFFF: nobody drove MISO */
    uint16_t errfl_or;    /* ERRFL bits seen this second */
    uint16_t angle;       /* last good angle, 14 bit */
    uint16_t amin, amax;
    uint16_t diag;        /* last good DIAAGC */
} enc_stat_t;

static enc_stat_t s_enc[NEXUS_NUM_ENCODERS];
static uint8_t    s_enc_ok;   /* SPI clock came up at 1 MHz */

/* ~1 us at 600 MHz. The AS5047P needs 350 ns CS high and CS-to-clock. */
static void short_pause(void)
{
    for (volatile unsigned i = 0u; i < 200u; i++) { }
}

static unsigned even_parity_ok(uint16_t w)
{
    w ^= w >> 8; w ^= w >> 4; w ^= w >> 2; w ^= w >> 1;
    return (w & 1u) == 0u;
}

/* One 16-bit frame inside its own CS pulse; returns the reply. */
static uint16_t as_frame(unsigned i, uint16_t tx)
{
    uint16_t rx = 0xA55Au;

    HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_RESET);
    short_pause();
    (void)HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1u, 2u);
    HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_SET);
    short_pause();
    return rx;
}

static unsigned enc_init_1mhz(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_PeriphCLKInitTypeDef clk = {0};

    for (unsigned c = 0u; c < NEXUS_NUM_ENCODERS; c++)
    {
        HAL_GPIO_WritePin(s_cs[c].port, s_cs[c].pin, GPIO_PIN_SET);
    }

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSIDiv              = RCC_HSI_DIV1;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK || HAL_SPI_DeInit(&hspi1) != HAL_OK)
    {
        return 0u;
    }
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        return 0u;
    }
    __HAL_RCC_CLKP_CONFIG(RCC_CLKPSOURCE_HSI);
    clk.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    clk.Spi1ClockSelection   = RCC_SPI1CLKSOURCE_CLKP;
    if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
    {
        return 0u;
    }
    return HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI1) == 64000000u;
}

/*
 * One sensor, four frames, each reply answering the previous command:
 *   ERRFL read  -> (stale)
 *   DIAAGC read -> ERRFL     (reading it also clears the latched EF)
 *   ANGLE read  -> DIAAGC
 *   NOP read    -> ANGLE
 */
static void enc_poll_one(unsigned i)
{
    enc_stat_t *s = &s_enc[i];

    (void)as_frame(i, AS_ERRFL_READ);
    uint16_t errfl = as_frame(i, AS_DIAAGC_READ);
    uint16_t diag  = as_frame(i, AS_ANGLE_READ);
    uint16_t angle = as_frame(i, AS_NOP_READ);

    s->reads++;

    /* All-zero and all-one replies both have valid parity and no EF, so they
       pass every check below while meaning "no sensor". Catch them first. */
    if (errfl == 0u && diag == 0u && angle == 0u) { s->zero++; s->bad++; return; }
    if (errfl == 0xFFFFu && diag == 0xFFFFu && angle == 0xFFFFu) { s->ones++; s->bad++; return; }

    if (!even_parity_ok(errfl) || !even_parity_ok(diag) || !even_parity_ok(angle) ||
        ((errfl | diag | angle) & 0x4000u) != 0u || (errfl & 0x0007u) != 0u)
    {
        s->errfl_or |= (uint16_t)(errfl & 0x0007u);
        s->bad++;
        return;
    }

    uint16_t a = angle & 0x3FFFu;
    s->angle = a;
    s->diag  = diag & 0x3FFFu;
    if (a < s->amin) s->amin = a;
    if (a > s->amax) s->amax = a;
}

static void enc_poll_all(void)
{
    for (unsigned i = 0u; i < NEXUS_NUM_ENCODERS; i++)
    {
        enc_poll_one(i);
    }
}

static unsigned enc_report(void)
{
    unsigned good = 0u;

    if (!s_enc_ok)
    {
        printf("  ENC  SPI1 could not be set to 1 MHz - encoders not tested\r\n");
        return 0u;
    }

    for (unsigned i = 0u; i < NEXUS_NUM_ENCODERS; i++)
    {
        enc_stat_t *s = &s_enc[i];
        uint32_t ok = s->reads - s->bad;
        uint16_t d  = s->diag;
        unsigned agc = d & 0xFFu;
        const char *verdict;

        if (s->reads == 0u)            verdict = "<-- not polled";
        else if (s->zero == s->reads)  verdict = "<-- all 0000: not answering (power/GND/MISO)";
        else if (s->ones == s->reads)  verdict = "<-- all FFFF: not selected or not connected";
        else if (ok == 0u)             verdict = "<-- every read rejected";
        else if (d & 0x0800u)          verdict = "<-- MAGNET TOO WEAK (MAGL)";
        else if (d & 0x0400u)          verdict = "<-- MAGNET TOO STRONG (MAGH)";
        else if (d & 0x0200u)          verdict = "<-- CORDIC overflow (COF)";
        else if ((d & 0x0100u) == 0u)  verdict = "<-- offset loop not ready (LF=0)";
        else if (s->bad != 0u)         verdict = "OK, but some reads failed";
        else                           verdict = "OK";

        if (ok == s->reads && s->reads != 0u && (d & 0x0F00u) == 0x0100u)
        {
            good++;
        }

        if (ok == 0u)
        {
            printf("  ENC  %-10s  %lu/%lu good  ERRFL bits=%X  %s\r\n",
                   s_enc_name[i], (unsigned long)ok, (unsigned long)s->reads,
                   (unsigned)s->errfl_or, verdict);
        }
        else
        {
            printf("  ENC  %-10s  %7.2f deg (raw %5u, moved %4u)  AGC=%3u  "
                   "%lu/%lu good  %s\r\n",
                   s_enc_name[i], (double)s->angle * (360.0 / 16384.0),
                   (unsigned)s->angle, (unsigned)(s->amax - s->amin), agc,
                   (unsigned long)ok, (unsigned long)s->reads, verdict);
        }

        s->reads = s->bad = s->zero = s->ones = 0u;
        s->errfl_or = 0u;
        s->amin = 0x3FFFu;
        s->amax = 0u;
    }
    return good;
}

/* ===================================================================== */
/*  IMU                                                                   */
/* ===================================================================== */

#define IMU_EXPECTED_HZ  400u

static unsigned imu_report(void)
{
    static uint32_t prev_seq, prev_bytes, prev_errs;
    imu_sample_t s;
    imu_get(&s);

    uint32_t bytes = imu_rx_bytes();
    uint32_t errs  = imu_errors();
    uint32_t hz    = s.seq - prev_seq;
    uint32_t dB    = bytes - prev_bytes;
    uint32_t dE    = errs - prev_errs;
    prev_seq = s.seq; prev_bytes = bytes; prev_errs = errs;

    float qn = sqrtf(s.quat[0]*s.quat[0] + s.quat[1]*s.quat[1] +
                     s.quat[2]*s.quat[2] + s.quat[3]*s.quat[3]);
    float an = sqrtf(s.accel[0]*s.accel[0] + s.accel[1]*s.accel[1] +
                     s.accel[2]*s.accel[2]);

    const char *verdict;
    unsigned good = 0u;
    if (dB == 0u)                                 verdict = "<-- NOTHING ON THE WIRE (power, PD6<-SDA, PS1 strap)";
    else if (hz == 0u && !imu_saw_advertisement()) verdict = "<-- bytes but no handshake yet (resetting)";
    else if (hz == 0u)                            verdict = "<-- talking but no reports (requesting)";
    else if (dE != 0u)                            verdict = "<-- UART errors";
    else if (hz < IMU_EXPECTED_HZ * 4u / 5u)      verdict = "<-- BELOW 400 Hz";
    else if (fabsf(qn - 1.0f) > 0.02f)            verdict = "<-- |q| not 1: misparsed";
    else if (an < 8.8f || an > 10.8f)             verdict = "<-- |a| not ~9.8: misparsed or moving";
    else                                        { verdict = "OK"; good = 1u; }

    printf("  IMU  %3lu Hz  q=%+.3f %+.3f %+.3f %+.3f  |a|=%5.2f m/s2  "
           "w=%+6.3f %+6.3f %+6.3f rad/s  %s\r\n",
           (unsigned long)hz,
           (double)s.quat[0], (double)s.quat[1], (double)s.quat[2], (double)s.quat[3],
           (double)an, (double)s.gyro[0], (double)s.gyro[1], (double)s.gyro[2],
           verdict);

    if (!good)
    {
        /* Straight from the UART and DMA registers: is the STM32 even
           listening? Same fields as the IMU test mode prints. */
        imu_diag_t d;
        imu_diag(&d);
        printf("       totals: %lu bytes %lu frames %lu err (last 0x%lX) rearms=%lu  arm=%lu "
               "isr=0x%08lX rxstate=0x%lX ndtr=%lu evts=%lu advert=%u\r\n",
               (unsigned long)bytes, (unsigned long)imu_frames(), (unsigned long)errs,
               (unsigned long)d.uart_error, (unsigned long)imu_rx_rearms(),
               (unsigned long)d.arm_status,
               (unsigned long)d.uart_isr, (unsigned long)d.rx_state,
               (unsigned long)d.dma_ndtr, (unsigned long)d.rx_events,
               (unsigned)imu_saw_advertisement());
        extern UART_HandleTypeDef huart2;
        printf("       dma.state=%u dma.err=0x%lX rxtype=%lu rxevt=%lu cr1=0x%08lX cr3=0x%08lX\r\n",
               (unsigned)(huart2.hdmarx ? huart2.hdmarx->State : 0xFF),
               (unsigned long)(huart2.hdmarx ? huart2.hdmarx->ErrorCode : 0),
               (unsigned long)huart2.ReceptionType, (unsigned long)huart2.RxEventType,
               (unsigned long)huart2.Instance->CR1, (unsigned long)huart2.Instance->CR3);
    }
    return good;
}

/* ===================================================================== */
/*  Foot switches                                                         */
/* ===================================================================== */

/*
 * One switch per foot, centre of the sole, wired pin-to-GND against the
 * STM32's internal pull-up (docs/PINOUT.md). So the PIN reads 1 with the foot
 * in the air and 0 when it is pressed - both are printed, because "1" alone
 * is exactly the thing two people read opposite ways.
 *
 * Read raw, every loop pass, with no debounce: this test is about the wire.
 * Edges per second is the number that matters - a long unscreened run down a
 * leg beside the motor cables flickers, and a flicker is a foot the estimator
 * thinks has landed.
 */
static const struct
{
    const char   *name;
    const char   *pin;
    GPIO_TypeDef *port;
    uint16_t      bit;
} s_foot[2] = {
    { "left ",  "PE2, CN11-46", L_TOE_GPIO_Port, L_TOE_Pin },
    { "right",  "PE4, CN11-48", R_TOE_GPIO_Port, R_TOE_Pin },
};

static uint8_t  s_foot_level[2];
static uint32_t s_foot_edges[2];
static uint32_t s_foot_low_ms[2];   /* ms the pin spent LOW (pressed) this second */
static uint32_t s_foot_last_ms;

static void foot_sample(uint32_t now)
{
    if (now == s_foot_last_ms)
    {
        return;
    }
    s_foot_last_ms = now;

    for (unsigned i = 0u; i < 2u; i++)
    {
        uint8_t lvl = (HAL_GPIO_ReadPin(s_foot[i].port, s_foot[i].bit) == GPIO_PIN_SET) ? 1u : 0u;
        if (lvl != s_foot_level[i])
        {
            s_foot_edges[i]++;
            s_foot_level[i] = lvl;
        }
        if (lvl == 0u)
        {
            s_foot_low_ms[i]++;
        }
    }
}

static void foot_report(void)
{
    for (unsigned i = 0u; i < 2u; i++)
    {
        const char *verdict =
            (s_foot_edges[i] > 4u) ? "<-- FLICKERING: noise on the wire (see PINOUT.md)" :
            (s_foot_edges[i] > 0u) ? "changed this second" : "steady";

        printf("  FOOT %s %s  pin=%u -> %-7s  pressed %4lu ms of 1000  edges=%lu  %s\r\n",
               s_foot[i].name, s_foot[i].pin, (unsigned)s_foot_level[i],
               s_foot_level[i] ? "OPEN" : "PRESSED",
               (unsigned long)s_foot_low_ms[i], (unsigned long)s_foot_edges[i], verdict);
        s_foot_edges[i]  = 0u;
        s_foot_low_ms[i] = 0u;
    }
}

/* ===================================================================== */

void sensors_init(void)
{
    memset(s_can, 0, sizeof(s_can));
    for (unsigned i = 0u; i < NEXUS_NUM_ENCODERS; i++)
    {
        memset(&s_enc[i], 0, sizeof(s_enc[i]));
        s_enc[i].amin = 0x3FFFu;
    }

    printf("\r\n====== SENSOR CHECK: CAN + ENCODERS + IMU + FOOT SWITCHES ======\r\n");
    printf("Read only. No drive is commanded or armed.\r\n");

    can_start(&hfdcan1, 0u);
    can_start(&hfdcan2, 1u);

    s_enc_ok = (uint8_t)enc_init_1mhz();

    for (unsigned i = 0u; i < 2u; i++)
    {
        s_foot_level[i] = (HAL_GPIO_ReadPin(s_foot[i].port, s_foot[i].bit) == GPIO_PIN_SET) ? 1u : 0u;
    }

    imu_init();
    HAL_TIM_Base_Start(&htim2);

    printf("CAN: both buses, FD, listening.  ENC: SPI1 polled at %s.  "
           "IMU: USART2 SHTP.\r\n",
           s_enc_ok ? "1 MHz" : "FAILED");
    printf("==========================================================\r\n\r\n");
}

void sensors_run(void)
{
    uint32_t last_enc   = HAL_GetTick();
    uint32_t last_imu   = last_enc;
    uint32_t last_print = last_enc;

    for (;;)
    {
        /* Parses the IMU's UART ring; must run far more often than anything
           else here or the 1 KB DMA ring wraps. */
        imu_service();

        uint32_t now = HAL_GetTick();
        foot_sample(now);

        if (s_enc_ok && (now - last_enc) >= 20u)   /* 50 Hz per sensor */
        {
            last_enc = now;
            enc_poll_all();
        }

        /* The BNO085 ignores requests sent before it has booted, so keep
           asking until samples arrive - same order as the IMU test. */
        if ((now - last_imu) >= 500u)
        {
            imu_sample_t s;
            last_imu = now;
            imu_get(&s);
            if (s.seq == 0u)
            {
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

        if ((now - last_print) >= 1000u)
        {
            last_print = now;
            printf("--- t=%lu s ---\r\n", (unsigned long)(now / 1000u));
            unsigned can = can_report();
            unsigned enc = enc_report();
            unsigned imu = imu_report();
            foot_report();
            printf("SUMMARY  CAN %u/8  ENC %u/4  IMU %s\r\n\r\n",
                   can, enc, imu ? "OK" : "FAIL");
            BSP_LED_Toggle(LED_GREEN);
        }
    }
}
