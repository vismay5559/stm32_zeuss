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

#define ENC_CHAINS          2u
#define ENC_PER_CHAIN       2u

/*
 * Which NEXUS_ENC_* index each word of each chain is. Word 0 comes from the
 * sensor nearest MISO, word 1 from the one nearest MOSI - see the wiring in
 * enc_as5047p.h. If a spring turns out to be reported in its neighbour's slot,
 * swap the two entries for that chain here and nowhere else.
 */
static const uint8_t s_slot[ENC_CHAINS][ENC_PER_CHAIN] = {
    { NEXUS_ENC_L_KNEE_PITCH, NEXUS_ENC_L_HIP_PITCH },      /* chain 0, enc_cs_left  */
    { NEXUS_ENC_R_KNEE_PITCH, NEXUS_ENC_R_HIP_PITCH },      /* chain 1, enc_cs_right */
};

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} cs_pin_t;

static const cs_pin_t s_cs[ENC_CHAINS] = {
    { enc_cs_left_GPIO_Port,  enc_cs_left_Pin  },
    { enc_cs_right_GPIO_Port, enc_cs_right_Pin },
};

/* Where such a buffer has to live, and why, is in dma_buffer.h. */
static uint16_t s_tx[ENC_PER_CHAIN] NEXUS_DMA_BUFFER;
static uint16_t s_rx[ENC_PER_CHAIN] NEXUS_DMA_BUFFER;

static uint16_t s_angle[NEXUS_NUM_ENCODERS];
static uint8_t  s_valid;
static volatile uint8_t s_busy;
static uint8_t  s_chain;                    /* chain in flight            */
static uint8_t  s_new_valid;                /* built up across both chains */

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
    for (uint8_t c = 0; c < ENC_CHAINS; c++)
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
    s_chain      = 0;
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

/* Select one chain and start its transfer. 0 if it started. */
static int start_chain(uint8_t chain)
{
    for (uint8_t w = 0; w < ENC_PER_CHAIN; w++)
    {
        s_tx[w] = s_cmd[s_slot[chain][w]];
    }

    s_chain = chain;
    HAL_GPIO_WritePin(s_cs[chain].port, s_cs[chain].pin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)s_tx, (uint8_t *)s_rx,
                                    ENC_PER_CHAIN) != HAL_OK)
    {
        HAL_GPIO_WritePin(s_cs[chain].port, s_cs[chain].pin, GPIO_PIN_SET);
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

    if (start_chain(0u) != 0)
    {
        s_busy = 0;
    }
}

void enc_on_dma_complete(void)
{
    uint8_t chain = s_chain;

    HAL_GPIO_WritePin(s_cs[chain].port, s_cs[chain].pin, GPIO_PIN_SET);

    for (uint8_t w = 0; w < ENC_PER_CHAIN; w++)
    {
        uint8_t  e     = s_slot[chain][w];
        uint16_t reply = s_rx[w];
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

    /* Left chain done: straight on to the right one, in the same tick. */
    if ((chain + 1u) < ENC_CHAINS)
    {
        if (start_chain((uint8_t)(chain + 1u)) == 0)
        {
            return;
        }
        /* The right chain would not start. The left readings still count;
           the right ones are simply not valid this tick. */
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
