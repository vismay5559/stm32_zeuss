/*
 * Host test for watchdog.c.
 *
 * The watchdog is the one piece of this firmware whose whole job is to act
 * when everything else has stopped working, which makes it the piece least
 * likely to be exercised before it is needed. These tests run its
 * configuration sequence against stubbed registers so at least the ordering,
 * the key values and the bounded-spin failure path are known to be right.
 */

#include "watchdog.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

static int s_fail;

#define CHECK(cond, fmt, ...)                                            \
    do {                                                                 \
        if (!(cond)) {                                                   \
            s_fail++;                                                    \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__);                  \
        }                                                                \
    } while (0)

#define KEY_RELOAD  0x0000AAAAu
#define KEY_UNLOCK  0x00005555u
#define KEY_START   0x0000CCCCu

static void reset_regs(void)
{
    memset((void *)&host_iwdg, 0, sizeof(host_iwdg));
    memset((void *)&host_rcc,  0, sizeof(host_rcc));
}

static void test_start_configures_and_reloads(void)
{
    printf("start: unlock, set the period, then reload\n");

    reset_regs();

    CHECK(wdg_start() == 1, "start reported failure with a healthy LSI");

    /* Last thing written must be a reload, so the first supervised window
       begins from a full count rather than wherever the counter sat. */
    CHECK(host_iwdg.KR == KEY_RELOAD,
          "last key written was 0x%04X, expected a reload (0x%04X)",
          (unsigned)host_iwdg.KR, (unsigned)KEY_RELOAD);

    CHECK(host_iwdg.RLR != 0, "reload value was never written");
    CHECK(host_iwdg.RLR <= 0xFFFu,
          "reload value %u does not fit the 12-bit register",
          (unsigned)host_iwdg.RLR);
    CHECK(host_iwdg.PR <= 6u, "prescaler code %u is out of range",
          (unsigned)host_iwdg.PR);

    /*
     * The declared timeout has to match the registers, or every margin
     * argument built on it is fiction.
     *   period_ms = reload * (4 << PR) / 32 kHz
     */
    uint32_t div      = 4u << host_iwdg.PR;
    uint32_t computed = (host_iwdg.RLR * div) / 32u;

    CHECK(computed == wdg_timeout_ms(),
          "registers give %u ms but wdg_timeout_ms() says %u ms",
          (unsigned)computed, (unsigned)wdg_timeout_ms());
}

static void test_refresh_writes_the_reload_key(void)
{
    printf("refresh: writes the reload key\n");

    reset_regs();
    (void)wdg_start();

    host_iwdg.KR = 0;
    wdg_refresh();

    CHECK(host_iwdg.KR == KEY_RELOAD,
          "refresh wrote 0x%04X, expected 0x%04X",
          (unsigned)host_iwdg.KR, (unsigned)KEY_RELOAD);
}

static void test_dead_lsi_is_reported_not_hung(void)
{
    printf("a stopped LSI reports failure instead of hanging\n");

    reset_regs();
    host_iwdg_stick_sr(IWDG_SR_PVU);

    CHECK(wdg_start() == 0, "claimed success while the status bit never cleared");
}

static void test_refresh_is_inert_before_start(void)
{
    printf("refresh before start does nothing\n");

    reset_regs();
    wdg_refresh();

    CHECK(host_iwdg.KR == 0,
          "refresh touched KR (0x%04X) before the watchdog was started",
          (unsigned)host_iwdg.KR);
}

static void test_reset_cause(void)
{
    printf("a watchdog reset is latched and the flags are cleared\n");

    reset_regs();
    host_rcc.RSR = RCC_RSR_IWDGRSTF;

    wdg_init_reset_cause();

    CHECK(wdg_reset_was_watchdog() == 1, "did not report a watchdog reset");
    CHECK((host_rcc.RSR & RCC_RSR_RMVF) != 0,
          "did not clear the reset flags, so the next boot would inherit them");

    reset_regs();
    wdg_init_reset_cause();
    CHECK(wdg_reset_was_watchdog() == 0, "reported a watchdog reset after a clean boot");
}

int main(void)
{
    printf("watchdog.c host tests\n---------------------\n");

    /* Order matters: refresh-before-start has to run before anything starts
       the watchdog, since starting it is a one-way door in the real part. */
    test_refresh_is_inert_before_start();
    test_start_configures_and_reloads();
    test_refresh_writes_the_reload_key();
    test_dead_lsi_is_reported_not_hung();
    test_reset_cause();

    printf("---------------------\n%s (%d failure%s)\n",
           s_fail ? "FAILED" : "PASSED", s_fail, s_fail == 1 ? "" : "s");

    return s_fail ? 1 : 0;
}
