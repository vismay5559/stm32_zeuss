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

/*
 * Net integrity probe: is the SPI1 bus electrically what the schematic says?
 *
 * WHY THIS EXISTS. The first run of this scan read angle=0000 from all four
 * encoders with spierr=0, and a meter on MISO showed 1.5-2.5 V wandering
 * rather than either rail. That is not a short to ground - against PB4's
 * internal pull-up it implies a sink of tens of kilohms - and it is not an SPI
 * misconfiguration, because the peripheral clocks without error.
 *
 * What it IS consistent with is MISO bridged to another signal on the same
 * header. CPOL is LOW, so SCK idles LOW: a MISO-to-SCK bridge holds MISO down
 * at idle (so the firmware reads 0) and makes a DC meter average the clock
 * while the scan runs (so the meter reads mid-rail). PB3 is CN7 pin 15 and PB4
 * is CN7 pin 19 - two positions apart, with pin 17 between them.
 *
 * So rather than guess, drive one pin and read another. A bridge announces
 * itself. Everything here is done with SPI1 PARKED and the pins as plain GPIO,
 * so the peripheral cannot confuse the result, and it runs before any encoder
 * is selected - all four chip selects are held high throughout, so nothing on
 * the far end is driving the bus and a reading can only come from the wiring.
 *
 * Reading the table: with a sound bus, MISO follows its own pull resistor and
 * nothing else. Any row where MISO tracks a pin being driven is a short
 * between those two, and it is at the connector or in the harness, not in the
 * silicon.
 */
#define MISO_PORT   GPIOB
#define MISO_PIN    GPIO_PIN_4
#define SCK_PORT    GPIOB
#define SCK_PIN     GPIO_PIN_3
#define MOSI_PORT   GPIOD
#define MOSI_PIN    GPIO_PIN_7

static void miso_as_input(uint32_t pull)
{
    GPIO_InitTypeDef g = {0};

    g.Pin   = MISO_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = pull;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MISO_PORT, &g);
    HAL_Delay(2);                       /* let the pull settle the net */
}

static void pin_as_output(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level)
{
    GPIO_InitTypeDef g = {0};

    HAL_GPIO_WritePin(port, pin, level);
    g.Pin   = pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &g);
    HAL_GPIO_WritePin(port, pin, level);
    HAL_Delay(2);
}

static unsigned miso(void)
{
    return (HAL_GPIO_ReadPin(MISO_PORT, MISO_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}

struct bus_pin { const char *name; GPIO_TypeDef *port; uint16_t pin; const char *where; };

/* Every signal on the SPI1 bus, so each can be driven or floated by name. */
static struct bus_pin bus_pins(unsigned i)
{
    struct bus_pin t[2 + NEXUS_NUM_ENCODERS] = {
        { "SCK",  SCK_PORT,  SCK_PIN,  "PB3, CN7 pin 15" },
        { "MOSI", MOSI_PORT, MOSI_PIN, "PD7, CN9 pin 2"  },
        { "CS left hip",   s_cs[0].port, s_cs[0].pin, "PF1"  },
        { "CS left knee",  s_cs[1].port, s_cs[1].pin, "PD15" },
        { "CS right hip",  s_cs[2].port, s_cs[2].pin, "PD14" },
        { "CS right knee", s_cs[3].port, s_cs[3].pin, "PF5"  },
    };
    return t[i];
}
#define BUS_PIN_COUNT (2u + NEXUS_NUM_ENCODERS)

static void pin_float(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef g = {0};

    g.Pin  = pin;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &g);
}

/*
 * EVERY bus signal high-impedance. This is the baseline each test starts from,
 * and leaving it out is how the first two versions of this probe produced
 * answers that were not about the pin they named: pins left driven by an
 * earlier step were still driving during a later one, so "no bridge to MOSI"
 * was measured with SCK held low and MISO pulled the wrong way.
 */
static void bus_all_float(void)
{
    /*
     * SCK and MOSI float; MISO floats; but every CHIP SELECT is driven HIGH,
     * which is the idle state that keeps the sensors DESELECTED.
     *
     * Floating them, as this did at first, leaves each sensor's CS input
     * indeterminate - a floating CMOS input can read LOW, so the sensor
     * decides it is selected and starts driving MISO. Every reading then
     * describes a sensor talking rather than the wiring, which is how this
     * probe came to report that MISO "follows MOSI high" and that four chip
     * selects all did the same thing.
     */
    pin_float(SCK_PORT, SCK_PIN);
    pin_float(MOSI_PORT, MOSI_PIN);
    for (unsigned c = 0; c < NEXUS_NUM_ENCODERS; c++)
    {
        pin_as_output(s_cs[c].port, s_cs[c].pin, GPIO_PIN_SET);
    }
    pin_float(MISO_PORT, MISO_PIN);
    HAL_Delay(2);
}

static void probe_bus_nets(void)
{
    printf("\r\nSPI1 net probe - SPI parked, every signal floated between tests\r\n");

    HAL_SPI_DeInit(&hspi1);
    bus_all_float();

    /*
     * 1. Is MISO free? Nothing else on the bus is driving, so it must follow
     *    whichever pull is enabled. Anything else is an external load.
     */
    miso_as_input(GPIO_PULLUP);
    unsigned up = miso();
    miso_as_input(GPIO_PULLDOWN);
    unsigned down = miso();
    printf("  MISO alone        pull-up=%u pull-down=%u   -> %s\r\n",
           up, down,
           (up == 1u && down == 0u) ? "FREE"
           : (up == 0u && down == 0u) ? "HELD LOW externally"
           : (up == 1u && down == 1u) ? "HELD HIGH externally"
                                      : "inconsistent - retry");

    /*
     * 2. Every other signal against MISO, BOTH directions, one at a time with
     *    everything else floating.
     *
     *    Driving the other pin LOW while MISO is pulled UP catches a bridge
     *    that drags MISO down - the case that matters for a chip select, which
     *    idles high and asserts low. Driving it HIGH with MISO pulled DOWN
     *    catches the other polarity. One direction alone is not a test: the
     *    earlier version only did the second and reported "no bridge" on all
     *    four chip selects while the right leg was reading 0000.
     *
     *    BOTH directions is also what separates a bridge from a sensor simply
     *    answering. A bridge is a push-pull pin wired to MISO, so it wins
     *    against either pull and MISO tracks it up AND down. A live sensor
     *    drives MISO only while its own CS is asserted and tri-states when it
     *    is released, so MISO returns to its pull in one of the two
     *    directions. Only "tracks both ways" is a bridge.
     */
    for (unsigned i = 0; i < BUS_PIN_COUNT; i++)
    {
        struct bus_pin b = bus_pins(i);

        bus_all_float();
        miso_as_input(GPIO_PULLUP);
        pin_as_output(b.port, b.pin, GPIO_PIN_RESET);
        unsigned pull_up_drive_low = miso();

        bus_all_float();
        miso_as_input(GPIO_PULLDOWN);
        pin_as_output(b.port, b.pin, GPIO_PIN_SET);
        unsigned pull_dn_drive_high = miso();

        const char *verdict =
            (pull_up_drive_low == 0u && pull_dn_drive_high == 1u)
                ? "*** BRIDGED to MISO ***"
            : (pull_up_drive_low == 0u)
                ? "MISO follows it LOW only"
            : (pull_dn_drive_high == 1u)
                ? "MISO follows it HIGH only"
                : "independent";

        printf("  %-14s %-16s lo->MISO=%u hi->MISO=%u   -> %s\r\n",
               b.name, b.where, pull_up_drive_low, pull_dn_drive_high, verdict);
    }

    /*
     * 3. The pin itself. A push-pull output beats leakage of tens of kilohms
     *    outright, so if MISO will not reach the rail it is driven to, the load
     *    is a real short or a damaged pad. Safe because everything else on the
     *    bus is floating.
     */
    bus_all_float();
    pin_as_output(MISO_PORT, MISO_PIN, GPIO_PIN_SET);
    unsigned drv_hi = miso();
    pin_as_output(MISO_PORT, MISO_PIN, GPIO_PIN_RESET);
    unsigned drv_lo = miso();
    printf("  MISO driven       %-16s high->%u low->%u   -> %s\r\n",
           "PB4, CN7 pin 19", drv_hi, drv_lo,
           (drv_hi == 1u && drv_lo == 0u) ? "pin and net sound"
                                          : "*** hard short or damaged pad ***");

    /* Hand the bus back: chip selects idle HIGH, MISO an input, SPI1 alive. */
    for (unsigned c = 0; c < NEXUS_NUM_ENCODERS; c++)
    {
        pin_as_output(s_cs[c].port, s_cs[c].pin, GPIO_PIN_SET);
    }
    miso_as_input(GPIO_PULLUP);
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        printf("  SPI1 re-init FAILED after the probe\r\n");
    }
    printf("\r\n");
}

/*
 * Spell out ERRFL, because a hex word in a log is a thing nobody decodes at
 * the bench and these three bits each send you somewhere different: a framing
 * error is clock or CS timing, an invalid command is a mangled address or
 * direction bit, a parity error is noise on MOSI or a bad command word.
 */
static const char *errfl_bits(uint16_t errfl)
{
    unsigned v = errfl & 0x0007u;

    if ((errfl & 0x4000u) != 0u) { return " (EF still set!)"; }
    switch (v)
    {
    case 0u: return " ok";
    case 1u: return " FRAMING";
    case 2u: return " INVALID_CMD";
    case 4u: return " PARITY";
    default: return " MULTIPLE";
    }
}

static void poll_raw_spi(void)
{
    uint16_t command = 0xFFFFu; /* AS5047P ANGLECOM read, even parity */
    uint16_t diag_command = 0xFFFCu; /* DIAAGC read, even parity */
    /*
     * 0xC000, not 0x0000. Both look like "do nothing" and only one of them is.
     * Bit 14 of a command frame is the direction bit and 1 means READ, so
     * 0xC000 is "read NOP" - a harmless frame whose only job is to clock the
     * previous command's reply out. 0x0000 is a WRITE command to the same
     * address, and a write command makes the sensor treat the NEXT frame as
     * its data, which desynchronises every reply after it by one and is a good
     * way to latch EF.
     *
     * This diagnostic sent 0x0000 for a long time, which is very likely why
     * the left leg's readings all came back with EF set. enc_as5047p.c, the
     * production driver, never had this bug.
     */
    uint16_t nop_command = 0xC000u;
    uint16_t errfl_command = 0x4001u; /* read ERRFL (0x0001), parity 0 */

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

        /*
         * Read ERRFL first, because EF LATCHES.
         *
         * One malformed frame sets the error flag in every subsequent reply
         * until ERRFL is read, and reading it is what clears it. So a single
         * bad frame at startup makes every angle for the rest of the run look
         * flagged, and the flag says nothing about the sensor or the magnet -
         * which is exactly the trap the left leg's readings fell into.
         *
         * The value is worth having for its own sake. ERRFL bit 0 is a
         * framing error, bit 1 an invalid command, bit 2 a parity error -
         * three different faults that EF alone cannot tell apart, and that
         * point at three different causes on the bench.
         */
        uint16_t errfl = 0xA55Au, discard = 0xA55Au;
        (void)HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&errfl_command,
                                      (uint8_t *)&discard, 1u, 10u);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        (void)HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&nop_command,
                                      (uint8_t *)&errfl, 1u, 10u);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_SET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(s_cs[i].port, s_cs[i].pin, GPIO_PIN_RESET);
        HAL_Delay(1);

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

        printf("  %-10s CS=%u/%u low_mask=%X PB4_idle=%u poll=%u/%u/%u "
               "ERRFL=%04X%s angle=%04X/%04X DIAAGC=%04X spierr=0x%lX\r\n",
               s_names[i], (unsigned)cs_low, (unsigned)cs_high,
               cs_low_mask, (unsigned)idle, (unsigned)first_status,
               (unsigned)second_status, (unsigned)diag_status,
               (unsigned)errfl, errfl_bits(errfl),
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

    probe_bus_nets();
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
