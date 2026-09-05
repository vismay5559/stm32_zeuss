#include "enc_as5048a.h"
#include "critical.h"
#include "dma_buffer.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define AS5048A_CMD_READ_ANGLE  0xFFFFu
#define AS5048A_ERROR_FLAG      0x4000u
#define AS5048A_ANGLE_MASK      0x3FFFu

/* Where such a buffer has to live, and why, is in dma_buffer.h. */
static uint16_t s_tx[NEXUS_NUM_ENCODERS] NEXUS_DMA_BUFFER;
static uint16_t s_rx[NEXUS_NUM_ENCODERS] NEXUS_DMA_BUFFER;

static uint16_t s_angle[NEXUS_NUM_ENCODERS];
static uint8_t  s_valid;
static volatile uint8_t s_busy;

/*
 * A transfer that never completes used to be permanent.
 *
 * s_busy is set before the DMA starts and cleared only in the completion
 * callback, so if that callback never arrived - an SPI error, an aborted
 * channel, a glitch on the bus - enc_start_read() returned early forever. The
 * angles froze at their last values and s_valid stayed at "all four good", so
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

void enc_init(void)
{
    for (int i = 0; i < NEXUS_NUM_ENCODERS; i++)
    {
        s_tx[i] = AS5048A_CMD_READ_ANGLE;
    }
    memset(s_angle, 0, sizeof(s_angle));
    s_valid      = 0;
    s_busy       = 0;
    s_busy_ticks = 0;
    s_stalls     = 0;
    s_errors     = 0;

    HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_SET);
}

/* Give up on the transfer in flight and leave the encoders marked invalid. */
static void abort_transfer(void)
{
    (void)HAL_SPI_Abort(&hspi1);
    HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_SET);

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
    s_busy = 1;

    HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)s_tx, (uint8_t *)s_rx,
                                    NEXUS_NUM_ENCODERS) != HAL_OK)
    {
        HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_SET);
        s_busy = 0;
    }
}

void enc_on_dma_complete(void)
{
    HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_SET);
    s_busy_ticks = 0;

    uint8_t valid = 0;

    for (int i = 0; i < NEXUS_NUM_ENCODERS; i++)
    {
        uint16_t w = s_rx[i];

        /* Even parity is computed over bits 0..14, with bit 15 carrying it. */
        uint8_t parity_ok = (even_parity((uint16_t)(w & 0x7FFFu)) == ((w >> 15) & 1u));

        if (parity_ok && ((w & AS5048A_ERROR_FLAG) == 0u))
        {
            s_angle[i] = (uint16_t)(w & AS5048A_ANGLE_MASK);
            valid |= (uint8_t)(1u << i);
        }
    }

    s_valid = valid;
    s_busy  = 0;
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
