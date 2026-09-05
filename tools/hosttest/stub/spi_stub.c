#include "main.h"

#include <string.h>

SPI_TypeDef host_spi1 = { 1 };
SPI_TypeDef host_spi2 = { 2 };

SPI_HandleTypeDef hspi1 = { &host_spi1 };

#define SENT_CAP  8u

static int      s_busy;
static uint8_t *s_rx;
static uint16_t s_size;
static uint32_t s_refuse;
static uint32_t s_started;
static uint32_t s_aborted;
static uint16_t s_sent[SENT_CAP];
static uint32_t s_sent_n;

void host_spi_reset(void)
{
    s_busy    = 0;
    s_rx      = 0;
    s_size    = 0;
    s_refuse  = 0;
    s_started = 0;
    s_aborted = 0;
    s_sent_n  = 0;
    memset(s_sent, 0, sizeof(s_sent));
}

void host_spi_refuse(uint32_t n)
{
    s_refuse = n;
}

int host_spi_busy(void)
{
    return s_busy;
}

uint32_t host_spi_started(void)
{
    return s_started;
}

uint32_t host_spi_aborted(void)
{
    return s_aborted;
}

uint16_t host_spi_sent_word(uint32_t index)
{
    return (index < s_sent_n) ? s_sent[index] : 0u;
}

void host_spi_reply(const uint16_t *words, uint32_t count)
{
    if (!s_busy || (s_rx == 0))
    {
        return;
    }

    uint32_t n = (count < (uint32_t)s_size) ? count : (uint32_t)s_size;
    memcpy(s_rx, words, n * sizeof(uint16_t));

    s_busy = 0;
    enc_on_dma_complete();
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h,
                                              const uint8_t *tx, uint8_t *rx,
                                              uint16_t size)
{
    (void)h;

    if (s_refuse > 0u)
    {
        s_refuse--;
        return HAL_ERROR;
    }

    if (s_busy)
    {
        return HAL_BUSY;
    }

    s_busy = 1;
    s_rx   = rx;
    s_size = size;
    s_started++;

    s_sent_n = (size < (uint16_t)SENT_CAP) ? (uint32_t)size : SENT_CAP;
    memcpy(s_sent, tx, s_sent_n * sizeof(uint16_t));

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h)
{
    (void)h;

    s_aborted++;
    s_busy = 0;
    s_rx   = 0;
    return HAL_OK;
}
