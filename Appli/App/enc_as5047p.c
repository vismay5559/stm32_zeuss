#include "enc_as5047p.h"
#include "critical.h"
#include "dma_buffer.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

/*
 * AS5047P frames (datasheet v2-00, figures 13, 14, 18): 16 bits, MSB first,
 * SPI mode 1, bit 15 even parity over bits 0..14.
 *
 *   command   PARC | R/W(1 = read) | 14-bit address
 *   reply     PARD | EF            | 14-bit data
 *
 * A reply answers the PREVIOUS frame's command, so every reading is one tick
 * old - the same age every tick, which is what matters.
 */
#define AS5047P_CMD_READ_ANGLECOM   0xFFFFu     /* read 0x3FFF, parity 1  */
#define AS5047P_CMD_READ_ERRFL      0x4001u     /* read 0x0001, parity 0  */
#define AS5047P_ERROR_FLAG          0x4000u
#define AS5047P_DATA_MASK           0x3FFFu

/*
 * One sensor per chip select, read one after another in this order.
 *
 * This was two daisy chains of two, which made the ORDER WITHIN a chain carry
 * meaning: the first word back came from whichever sensor sat nearest MISO, so
 * wiring hip and knee the other way round silently swapped two springs in the
 * data. Four separate selects remove that entirely - a reply can only have
 * come from the one sensor that was selected.
 *
 * If a spring turns out to be reported as another one, the wire is on the
 * wrong CS pin; s_cs[] below is the one place to change.
 */
#define ENC_DEVICES         4u
#define ENC_WORDS           1u      /* one sensor selected, one word exchanged */

static const uint8_t s_slot[ENC_DEVICES] = {
    NEXUS_ENC_L_HIP_PITCH,
    NEXUS_ENC_L_KNEE_PITCH,
    NEXUS_ENC_R_HIP_PITCH,
    NEXUS_ENC_R_KNEE_PITCH,
};

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} cs_pin_t;

static const cs_pin_t s_cs[ENC_DEVICES] = {
    { enc_cs_l_hip_GPIO_Port,   enc_cs_l_hip_Pin   },   /* PF1  */
    { enc_cs_l_knee_GPIO_Port,  enc_cs_l_knee_Pin  },   /* PD15 */
    { enc_cs_r_hip_GPIO_Port,   enc_cs_r_hip_Pin   },   /* PD14 */
    { enc_cs_r_knee_GPIO_Port,  enc_cs_r_knee_Pin  },   /* PF5  */
};

_Static_assert(ENC_DEVICES == NEXUS_NUM_ENCODERS,
               "one chip select per encoder, so the two counts are the same");

/* Where such a buffer has to live, and why, is in dma_buffer.h. */
static uint16_t s_tx[ENC_WORDS] NEXUS_DMA_BUFFER;
static uint16_t s_rx[ENC_WORDS] NEXUS_DMA_BUFFER;

static uint16_t s_angle[NEXUS_NUM_ENCODERS];
static uint8_t  s_valid;
static volatile uint8_t s_busy;
static uint8_t  s_dev;                      /* device in flight              */
static uint8_t  s_new_valid;                /* built up across all four      */

/*
 * What each sensor was last asked, so its next reply is read as the right
 * thing.
 *
 * When a frame goes wrong the AS5047P latches the cause in ERRFL and sets EF
 * in its replies until someone reads ERRFL - reading it is what clears it. A
 * driver that only ever asks for the angle would see EF on every reply from
 * then on, and that sensor would stay "untrustworthy" for the rest of the run
 * over one glitch. So a reply with EF set makes the next request to that
 * sensor an ERRFL read; the reply to that is the error register, not an angle,
 * and is skipped; the request after it is the angle again. Two ticks out,
 * then back.
 */
static uint16_t s_cmd[NEXUS_NUM_ENCODERS];

/*
 * A transfer that never completes used to be permanent.
 *
 * s_busy is set before the DMA starts and cleared only in the completion
 * callback, so if that callback never arrived - an SPI error, an aborted
 * channel, a glitch on the bus - enc_start_read() returned early forever. The
 * angles froze at their last values and s_valid stayed at "all good", so
 * health.c never noticed either: it watches the validity mask, which by then
 * was a fossil.
 *
 * Age the in-flight transfer instead, and treat one that overstays as failed.
 */
#define ENC_XFER_TIMEOUT_TICKS  5u

static uint16_t s_busy_ticks;
static uint32_t s_stalls;
static uint32_t s_errors;

static uint8_t even_parity(uint16_t v)
{
    v ^= (uint16_t)(v >> 8);
    v ^= (uint16_t)(v >> 4);
    v ^= (uint16_t)(v >> 2);
    v ^= (uint16_t)(v >> 1);
    return (uint8_t)(v & 1u);
}

static void cs_release_all(void)
{
    for (uint8_t c = 0; c < ENC_DEVICES; c++)
    {
        HAL_GPIO_WritePin(s_cs[c].port, s_cs[c].pin, GPIO_PIN_SET);
    }
}

void enc_init(void)
{
    for (int i = 0; i < NEXUS_NUM_ENCODERS; i++)
    {
        s_cmd[i] = AS5047P_CMD_READ_ANGLECOM;
    }
    memset(s_angle, 0, sizeof(s_angle));
    s_valid      = 0;
    s_new_valid  = 0;
    s_busy       = 0;
    s_dev        = 0;
    s_busy_ticks = 0;
    s_stalls     = 0;
    s_errors     = 0;

    cs_release_all();
}

/* Give up on the transfer in flight and leave the encoders marked invalid. */
static void abort_transfer(void)
{
    (void)HAL_SPI_Abort(&hspi1);
    cs_release_all();

    s_busy       = 0;
    s_busy_ticks = 0;

    /*
     * Clear the validity mask, not just the busy flag. Leaving it set would
     * hand out the last good sample indefinitely as though it were current -
     * and health.c, which only watches this mask, would keep reporting the
     * encoders as fine.
     */
    s_valid = 0;
}

/* Select one sensor and start its transfer. 0 if it started. */
static int start_device(uint8_t dev)
{
    s_tx[0] = s_cmd[s_slot[dev]];

    s_dev = dev;
    HAL_GPIO_WritePin(s_cs[dev].port, s_cs[dev].pin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)s_tx, (uint8_t *)s_rx,
                                    ENC_WORDS) != HAL_OK)
    {
        HAL_GPIO_WritePin(s_cs[dev].port, s_cs[dev].pin, GPIO_PIN_SET);
        return -1;
    }
    return 0;
}

void enc_start_read(void)
{
    if (s_busy)
    {
        if (++s_busy_ticks >= ENC_XFER_TIMEOUT_TICKS)
        {
            s_stalls++;
            abort_transfer();
        }
        return;
    }

    s_busy_ticks = 0;
    s_new_valid  = 0;
    s_busy       = 1;

    if (start_device(0u) != 0)
    {
        s_busy = 0;
    }
}

void enc_on_dma_complete(void)
{
    uint8_t dev = s_dev;

    HAL_GPIO_WritePin(s_cs[dev].port, s_cs[dev].pin, GPIO_PIN_SET);

    {
        uint8_t  e     = s_slot[dev];
        uint16_t reply = s_rx[0];
        uint16_t asked = s_cmd[e];

        /* Even parity is computed over bits 0..14, with bit 15 carrying it. */
        uint8_t parity_ok = (even_parity((uint16_t)(reply & 0x7FFFu)) == ((reply >> 15) & 1u));
        uint8_t flagged   = ((reply & AS5047P_ERROR_FLAG) != 0u);

        if (parity_ok && !flagged && (asked == AS5047P_CMD_READ_ANGLECOM))
        {
            s_angle[e] = (uint16_t)(reply & AS5047P_DATA_MASK);
            s_new_valid |= (uint8_t)(1u << e);
        }

        /* A flagged reply means ERRFL is latched: read it next, which clears
           it. Anything else goes back to (or stays on) the angle. */
        s_cmd[e] = (parity_ok && flagged && (asked != AS5047P_CMD_READ_ERRFL))
                       ? AS5047P_CMD_READ_ERRFL
                       : AS5047P_CMD_READ_ANGLECOM;
    }

    /* On to the next sensor, in the same tick. */
    if ((dev + 1u) < ENC_DEVICES)
    {
        if (start_device((uint8_t)(dev + 1u)) == 0)
        {
            return;
        }
        /* That one would not start. The sensors already read still count;
           the rest are simply not valid this tick. */
    }

    s_busy_ticks = 0;
    s_valid      = s_new_valid;
    s_busy       = 0;
}

void enc_get(uint16_t angle[NEXUS_NUM_ENCODERS], uint8_t *valid_mask)
{
    /* enc_on_dma_complete() runs in the GPDMA1 Channel 0 ISR and rewrites both
       arrays, so read them as one indivisible step. Otherwise the valid mask
       can describe a different sample than the angles handed back. */
    uint32_t primask = critical_enter();

    for (int i = 0; i < NEXUS_NUM_ENCODERS; i++)
    {
        angle[i] = s_angle[i];
    }
    *valid_mask = s_valid;

    critical_exit(primask);
}

uint32_t enc_stalls(void)
{
    return s_stalls;
}

uint32_t enc_errors(void)
{
    return s_errors;
}

/*
 * Overrides the HAL's __weak stub.
 *
 * Without this an SPI error leaves the DMA aborted and s_busy stuck at 1, and
 * the encoders are dead for the rest of the run with nothing to show for it.
 * imu_bno085.c has had the equivalent for its UART for a while; the SPI path
 * never got one.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1)
    {
        return;
    }

    s_errors++;
    abort_transfer();
}
