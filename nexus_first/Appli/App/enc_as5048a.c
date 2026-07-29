#include "enc_as5048a.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define AS5048A_CMD_READ_ANGLE  0xFFFFu
#define AS5048A_ERROR_FLAG      0x4000u
#define AS5048A_ANGLE_MASK      0x3FFFu

static uint16_t s_tx[NEXUS_NUM_ENCODERS] __attribute__((section("noncacheable_buffer"), aligned(32)));
static uint16_t s_rx[NEXUS_NUM_ENCODERS] __attribute__((section("noncacheable_buffer"), aligned(32)));

static uint16_t s_angle[NEXUS_NUM_ENCODERS];
static uint8_t  s_valid;
static volatile uint8_t s_busy;

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
    s_valid = 0;
    s_busy  = 0;

    HAL_GPIO_WritePin(enc_cs_GPIO_Port, enc_cs_Pin, GPIO_PIN_SET);
}

void enc_start_read(void)
{
    if (s_busy)
    {
        return;
    }
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
    for (int i = 0; i < NEXUS_NUM_ENCODERS; i++)
    {
        angle[i] = s_angle[i];
    }
    *valid_mask = s_valid;
}
