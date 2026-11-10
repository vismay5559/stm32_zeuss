#include "test_enc_scan.h"
#include "enc_as5047p.h"
#include "main.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

/* Bench probe: keep DMA entirely out of this run. */
#define ENC_SCAN_POLL_ONLY 1

static const char *const s_names[NEXUS_NUM_ENCODERS] = {
    "left hip", "left knee", "right hip", "right knee"
};

static const struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} s_cs[NEXUS_NUM_ENCODERS] = {
    { enc_cs_l_hip_GPIO_Port, enc_cs_l_hip_Pin },
    { enc_cs_l_knee_GPIO_Port, enc_cs_l_knee_Pin },
    { enc_cs_r_hip_GPIO_Port, enc_cs_r_hip_Pin },
    { enc_cs_r_knee_GPIO_Port, enc_cs_r_knee_Pin },
};

static void poll_raw_spi(void)
{
    uint16_t command = 0xFFFFu; /* AS5047P ANGLECOM read, even parity */
    uint16_t diag_command = 0xFFFCu; /* DIAAGC read, even parity */
    uint16_t nop_command = 0x0000u;

    /* With every sensor deselected, only PB4's pull-up should drive MISO.
       This distinguishes a bus driven during clocks from a CS selection bug. */
    for (unsigned c = 0; c < NEXUS_NUM_ENCODERS; c++)
        HAL_GPIO_WritePin(s_cs[c].port, s_cs[c].pin, GPIO_PIN_SET);
    HAL_Delay(1);
    GPIO_PinState no_cs_idle = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
    const uint16_t no_cs_tx[3] = {0x0000u, 0xFFFFu, 0xA5A5u};
    uint16_t no_cs_rx[3] = {0xA55Au, 0xA55Au, 0xA55Au};
    HAL_StatusTypeDef no_cs_status[3];
    for (unsigned n = 0; n < 3; n++)
        no_cs_status[n] = HAL_SPI_TransmitReceive(
            &hspi1, (uint8_t *)&no_cs_tx[n], (uint8_t *)&no_cs_rx[n], 1u, 10u);
    GPIO_PinState no_cs_after = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
    printf("  NO_CS PB4=%u/%u poll=%u/%u/%u tx=0000/FFFF/A5A5 rx=%04X/%04X/%04X\r\n",
           (unsigned)no_cs_idle, (unsigned)no_cs_after,
           (unsigned)no_cs_status[0], (unsigned)no_cs_status[1],
           (unsigned)no_cs_status[2], (unsigned)no_cs_rx[0],
           (unsigned)no_cs_rx[1], (unsigned)no_cs_rx[2]);

    /* Two CS pulses per sensor: each reply belongs to the previous command.
       This bypasses DMA and the production decoder to expose the actual SPI
       word that the peripheral receives from each selected sensor. */
    for (unsigned i = 0; i < NEXUS_NUM_ENCODERS; i++)
    {
        uint16_t first = 0xA55Au, second = 0xA55Au;
        uint16_t ignored = 0xA55Au, diag = 0xA55Au;
        HAL_StatusTypeDef first_status, second_status, diag_status;
        for (unsigned c = 0; c < NEXUS_NUM_ENCODERS; c++)
            HAL_GPIO_WritePin(s_cs[c].port, s_cs[c].pin, GPIO_PIN_SET);
        HAL_Delay(1);
        GPIO_PinState idle = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_RESET);
        GPIO_PinState cs_low = HAL_GPIO_ReadPin(s_cs[i].port, s_cs[i].pin);
        unsigned cs_low_mask = 0u;
        for (unsigned c = 0; c < NEXUS_NUM_ENCODERS; c++)
            if (HAL_GPIO_ReadPin(s_cs[c].port, s_cs[c].pin) == GPIO_PIN_RESET)
                cs_low_mask |= 1u << c;
        HAL_Delay(1); /* Exceeds the AS5047P CS-to-clock setup time. */
        first_status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&command,
                                               (uint8_t *)&first, 1u, 10u);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_RESET);
        second_status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&command,
                                                (uint8_t *)&second, 1u, 10u);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_RESET);
        (void)HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&diag_command,
                                      (uint8_t *)&ignored, 1u, 10u);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_RESET);
        diag_status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&nop_command,
                                              (uint8_t *)&diag, 1u, 10u);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_SET);
        GPIO_PinState cs_high = HAL_GPIO_ReadPin(s_cs[i].port, s_cs[i].pin);

        printf("  %-10s CS=%u/%u low_mask=%X PB4_idle=%u poll=%u/%u/%u angle=%04X/%04X "
               "DIAAGC=%04X spierr=0x%lX\r\n",
               s_names[i], (unsigned)cs_low, (unsigned)cs_high,
               cs_low_mask, (unsigned)idle, (unsigned)first_status,
               (unsigned)second_status, (unsigned)diag_status,
               (unsigned)first, (unsigned)second, (unsigned)diag,
               (unsigned long)HAL_SPI_GetError(&hspi1));
    }
}

void encscan_init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};

    /* Only this diagnostic mode changes SPI1's clock. HSI runs nominally at
       64 MHz, and the SPI prescaler is 64, giving nominally 1 MHz SCK. PLL1Q
       is 100 MHz here, so none of its power-of-two divisors gives 1 MHz. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSIDiv = RCC_HSI_DIV1;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK ||
        HAL_SPI_DeInit(&hspi1) != HAL_OK)
    {
        printf("SPI 1 MHz clock setup FAILED\r\n");
        for (;;) { HAL_Delay(1000); }
    }

    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        printf("SPI 1 MHz init FAILED\r\n");
        for (;;) { HAL_Delay(1000); }
    }
    __HAL_RCC_CLKP_CONFIG(RCC_CLKPSOURCE_HSI);
    clock.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    clock.Spi1ClockSelection = RCC_SPI1CLKSOURCE_CLKP;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK)
    {
        printf("SPI1 HSI clock selection FAILED\r\n");
        for (;;) { HAL_Delay(1000); }
    }
    uint32_t kernel_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI1);
    printf("SPI1 kernel %lu Hz / 64 = %lu Hz SCK (nominal)\r\n",
           (unsigned long)kernel_hz, (unsigned long)(kernel_hz / 64u));
    printf("SPI1 DataSize=%lu CPOL=%lu CPHA=%lu FirstBit=%lu NSS=%lu\r\n",
           (unsigned long)hspi1.Init.DataSize,
           (unsigned long)hspi1.Init.CLKPolarity,
           (unsigned long)hspi1.Init.CLKPhase,
           (unsigned long)hspi1.Init.FirstBit,
           (unsigned long)hspi1.Init.NSS);
    if (kernel_hz != 64000000u)
    {
        printf("SPI1 is not at 1 MHz; scan stopped\r\n");
        for (;;) { HAL_Delay(1000); }
    }

    printf("SPI polling probe at 1 MHz (two raw words per CS):\r\n");
    poll_raw_spi();
#if !ENC_SCAN_POLL_ONLY
    enc_init();
#endif
    printf("SPI1 spring encoder scan: PF1, PD15, PD14, PF5\r\n");
    printf("Raw magnet angles only; zeros/signs are not calibrated. "
           "No motor commands sent.\r\n");
}

void encscan_run(void)
{
#if ENC_SCAN_POLL_ONLY
    for (;;)
    {
        poll_raw_spi();
        HAL_Delay(1000);
    }
#else
    uint16_t angle[NEXUS_NUM_ENCODERS];
    uint16_t minimum[NEXUS_NUM_ENCODERS];
    uint16_t maximum[NEXUS_NUM_ENCODERS];
    uint16_t valid_count[NEXUS_NUM_ENCODERS] = {0};
    uint8_t valid;

    for (unsigned i = 0; i < NEXUS_NUM_ENCODERS; i++)
    {
        minimum[i] = 0x3FFFu;
        maximum[i] = 0u;
    }

    for (;;)
    {
        /* A 1 ms gap gives the four background DMA transfers time to finish.
           Starting another scan every 5 ms stays well below the SPI limit. */
        enc_start_read();
        HAL_Delay(1);
        enc_get(angle, &valid);

        for (unsigned i = 0; i < NEXUS_NUM_ENCODERS; i++)
        {
            if ((valid & (1u << i)) == 0u)
            {
                continue;
            }
            valid_count[i]++;
            if (angle[i] < minimum[i]) minimum[i] = angle[i];
            if (angle[i] > maximum[i]) maximum[i] = angle[i];
        }

        static unsigned samples;
        if (++samples >= 200u)
        {
            samples = 0u;
            if (valid_count[0] == 200u && valid_count[1] == 200u &&
                valid_count[2] == 200u && valid_count[3] == 200u &&
                minimum[0] == 0u && minimum[1] == 0u &&
                minimum[2] == 0u && minimum[3] == 0u &&
                maximum[0] == 0u && maximum[1] == 0u &&
                maximum[2] == 0u && maximum[3] == 0u)
            {
                /* A low MISO returns 0x0000, which has correct parity. Four
                   perfectly zero angles are therefore not proof of four good
                   sensors, even when the driver's validity mask is full. */
                printf("!! all four replies are zero; check shared PB4 MISO "
                       "and sensor power\r\n");
            }
            printf("SPI: stalls=%lu errors=%lu\r\n",
                   (unsigned long)enc_stalls(),
                   (unsigned long)enc_errors());
            for (unsigned i = 0; i < NEXUS_NUM_ENCODERS; i++)
            {
                printf("  %-10s CS valid=%3u/200 raw=%5u "
                       "angle=%7.2f deg range=%5u..%5u\r\n",
                       s_names[i], (unsigned)valid_count[i],
                       (unsigned)angle[i],
                       (double)angle[i] * (360.0 / 16384.0),
                       (unsigned)(valid_count[i] ? minimum[i] : 0u),
                       (unsigned)(valid_count[i] ? maximum[i] : 0u));
                valid_count[i] = 0u;
                minimum[i] = 0x3FFFu;
                maximum[i] = 0u;
            }
            printf("\r\n");
        }
        HAL_Delay(4);
    }
#endif
}
