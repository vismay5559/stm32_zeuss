#include "watchdog.h"
#include "main.h"

/* Key register commands. */
#define IWDG_KEY_RELOAD   0x0000AAAAu
#define IWDG_KEY_UNLOCK   0x00005555u
#define IWDG_KEY_START    0x0000CCCCu

/*
 * Timeout budget.
 *
 * The LSI is nominally 32 kHz but is specified over a wide range, so the real
 * timeout is not the nominal one - and only the SHORT end matters, because
 * that is where a healthy loop gets reset by mistake.
 *
 *     prescaler /32  ->  1 kHz nominal  ->  RLR 100  ->  100 ms nominal
 *     worst-case fast LSI (~60 kHz)     ->  about      53 ms
 *
 * What has to fit inside that shorter figure is the longest legitimate gap
 * between two refreshes. Today that is the 2-second status printf, which
 * blocks for ~8.4 ms at 115200 baud (97 characters), plus one tick of work.
 * Roughly 6x margin against the worst-case clock, which is the right side of
 * comfortable for something that reboots the robot when it is wrong.
 *
 * If the loop ever gains a longer blocking section, this number moves before
 * that section lands - not after the first spurious reset on the bench.
 */
#define IWDG_PRESCALER_CODE   3u     /* 0=/4 1=/8 2=/16 3=/32 4=/64 ... */
#define IWDG_RELOAD           100u   /* 12-bit, so 0..4095              */
#define IWDG_TIMEOUT_MS       100u

/*
 * Writes to PR/RLR take several LSI cycles to land and the matching SR bit
 * stays set until they do. At 600 MHz an LSI cycle is ~19k CPU cycles, so a
 * generous bound is still only a few milliseconds - and bounding it is the
 * point: an unbounded poll here hangs the board when the LSI is not running,
 * turning the safety device into the thing that bricks it.
 */
#define IWDG_SR_SPINS   2000000u

static uint8_t s_was_watchdog;
static uint8_t s_running;

static uint8_t wait_sr_clear(uint32_t mask)
{
    for (uint32_t i = 0; i < IWDG_SR_SPINS; i++)
    {
        if ((IWDG->SR & mask) == 0u)
        {
            return 1u;
        }
    }
    return 0u;
}

void wdg_init_reset_cause(void)
{
    s_was_watchdog = (RCC->RSR & RCC_RSR_IWDGRSTF) ? 1u : 0u;

    /* Clear the flags so the NEXT reset's cause is unambiguous. They are
       sticky across resets and would otherwise accumulate. */
    RCC->RSR |= RCC_RSR_RMVF;
}

uint8_t wdg_reset_was_watchdog(void)
{
    return s_was_watchdog;
}

uint8_t wdg_start(void)
{
    /* Starting also forces the LSI on; there is no separate clock enable. */
    IWDG->KR = IWDG_KEY_START;

    IWDG->KR = IWDG_KEY_UNLOCK;

    IWDG->PR = IWDG_PRESCALER_CODE;
    if (!wait_sr_clear(IWDG_SR_PVU))
    {
        return 0u;
    }

    IWDG->RLR = IWDG_RELOAD;
    if (!wait_sr_clear(IWDG_SR_RVU))
    {
        return 0u;
    }

    /* Reload once so the first window starts from a full count rather than
       from whatever the counter happened to hold. */
    IWDG->KR = IWDG_KEY_RELOAD;

    s_running = 1u;
    return 1u;
}

void wdg_refresh(void)
{
    if (s_running)
    {
        IWDG->KR = IWDG_KEY_RELOAD;
    }
}

uint32_t wdg_timeout_ms(void)
{
    return IWDG_TIMEOUT_MS;
}
