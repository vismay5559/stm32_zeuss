/*
 * Host test for health.c - the part that decides whether the robot is well
 * enough to be allowed to move.
 *
 * safety.c will not arm while anything here reports a fault, so a fault that
 * fails to appear means a robot that starts moving on a broken sensor, and a
 * fault that appears too eagerly means one that will not start at all. Both
 * are easy to introduce by adjusting a threshold and both are invisible
 * without a test.
 *
 * The counters health.c watches live in other modules, so they are stubbed
 * here and driven directly. That is the point: it can be asked what happens
 * after two hundred silent ticks without waiting two hundred milliseconds or
 * owning a robot.
 */

#include "health.h"
#include "link_proto.h"

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

/* ---- the world health.c looks at, under our control ----------------- */

static uint32_t s_can_rx[2];
static uint32_t s_cmds;
static uint32_t s_overruns;

uint32_t act_rx_count(uint8_t bus)     { return s_can_rx[bus & 1u]; }
uint32_t link_usb_cmd_count(void)      { return s_cmds; }
uint32_t app_overruns(void)            { return s_overruns; }

/* Unused by health.c but referenced through its headers. */
uint32_t act_bus_off_count(uint8_t bus) { (void)bus; return 0; }
uint8_t  act_tx_error_count(uint8_t bus){ (void)bus; return 0; }
uint32_t act_tx_dropped(uint8_t bus)    { (void)bus; return 0; }

/* health.c wants EVERY encoder valid; it compares against
   (1 << NEXUS_NUM_ENCODERS) - 1, so a wider mask is not "all of them". */
#define ALL_ENC_OK  ((uint8_t)((1u << NEXUS_NUM_ENCODERS) - 1u))

static imu_sample_t s_imu;

static void reset_world(void)
{
    memset(&s_imu, 0, sizeof(s_imu));
    s_can_rx[0] = s_can_rx[1] = 0;
    s_cmds      = 0;
    s_overruns  = 0;
    health_init(HEALTH_EXPECTED_ROBOT);
}

/* One tick with everything alive: every counter moves. */
static void healthy_tick(void)
{
    s_imu.seq++;
    s_can_rx[0]++;
    s_can_rx[1]++;
    s_cmds++;
    health_tick(&s_imu, ALL_ENC_OK);
}

static void healthy_ticks(int n)
{
    for (int i = 0; i < n; i++)
    {
        healthy_tick();
    }
}

/* ---- the tests -------------------------------------------------------- */

static void test_a_healthy_robot_reports_nothing(void)
{
    printf("everything alive means no faults and no blink\n");

    reset_world();
    healthy_ticks(500);

    CHECK(health_faults() == 0u,
          "a healthy robot reported faults 0x%lx", (unsigned long)health_faults());
    CHECK(health_blink_code() == 0u,
          "a healthy robot asked for %u blinks", health_blink_code());
}

static void test_a_silent_sensor_is_noticed_but_not_instantly(void)
{
    printf("a sensor going quiet is caught, after a grace period, not on tick one\n");

    reset_world();
    healthy_ticks(100);

    /* Stop the IMU. Everything else keeps running. */
    for (int i = 0; i < 5; i++)
    {
        s_can_rx[0]++; s_can_rx[1]++; s_cmds++;
        health_tick(&s_imu, ALL_ENC_OK);      /* seq deliberately not bumped */
    }
    CHECK((health_faults() & HEALTH_IMU) == 0u,
          "a five-tick gap in the IMU was reported as a fault - too eager");

    for (int i = 0; i < 200; i++)
    {
        s_can_rx[0]++; s_can_rx[1]++; s_cmds++;
        health_tick(&s_imu, ALL_ENC_OK);
    }
    CHECK((health_faults() & HEALTH_IMU) != 0u,
          "the IMU was silent for 200 ticks and no fault was raised");

    /* Only the IMU should be implicated. */
    CHECK((health_faults() & (HEALTH_CAN1 | HEALTH_CAN2 | HEALTH_LINK)) == 0u,
          "a silent IMU also blamed the buses or the link: 0x%lx",
          (unsigned long)health_faults());
}

static void test_each_subsystem_is_blamed_separately(void)
{
    printf("each subsystem going quiet raises its own fault and no other\n");

    struct { const char *name; uint32_t bit; int stop_imu, stop_can0, stop_can1, stop_cmds; }
    cases[4] = {
        { "IMU",    HEALTH_IMU,  1, 0, 0, 0 },
        { "CAN1",   HEALTH_CAN1, 0, 1, 0, 0 },
        { "CAN2",   HEALTH_CAN2, 0, 0, 1, 0 },
        { "Pi link",HEALTH_LINK, 0, 0, 0, 1 },
    };

    for (int k = 0; k < 4; k++)
    {
        reset_world();
        healthy_ticks(50);

        for (int i = 0; i < 400; i++)
        {
            if (!cases[k].stop_imu)  { s_imu.seq++; }
            if (!cases[k].stop_can0) { s_can_rx[0]++; }
            if (!cases[k].stop_can1) { s_can_rx[1]++; }
            if (!cases[k].stop_cmds) { s_cmds++; }
            health_tick(&s_imu, ALL_ENC_OK);
        }

        uint32_t f = health_faults();
        CHECK((f & cases[k].bit) != 0u, "%s went quiet and was not blamed", cases[k].name);
        CHECK((f & ~cases[k].bit & ~(uint32_t)HEALTH_TIMING) == 0u,
              "%s going quiet also blamed something else: 0x%lx",
              cases[k].name, (unsigned long)f);
    }
}

static void test_a_bad_encoder_reading_is_tolerated_briefly(void)
{
    printf("the odd bad encoder reading is tolerated; a persistent one is not\n");

    reset_world();
    healthy_ticks(50);

    /* A handful of bad reads - the sort a bump on the leg produces. */
    for (int i = 0; i < 5; i++)
    {
        s_imu.seq++; s_can_rx[0]++; s_can_rx[1]++; s_cmds++;
        health_tick(&s_imu, 0x00u);
    }
    CHECK((health_faults() & HEALTH_ENC) == 0u,
          "five bad encoder reads were treated as a fault - too eager");

    for (int i = 0; i < 50; i++)
    {
        s_imu.seq++; s_can_rx[0]++; s_can_rx[1]++; s_cmds++;
        health_tick(&s_imu, 0x00u);
    }
    CHECK((health_faults() & HEALTH_ENC) != 0u,
          "the encoders were unreadable for 50 ticks and no fault was raised");
}

static void test_a_sensor_fault_follows_the_live_state(void)
{
    printf("a sensor fault appears while the part is out and goes when it returns\n");

    /*
     * Sensor faults are NOT latched - set_fault() clears the bit as soon as
     * the subsystem is answering again. So health_faults() reports what is
     * broken now, which is what safety.c needs before it will arm.
     *
     * (The header used to claim these latched. They do not, and the claim was
     * mine - this test is what caught it.)
     */
    reset_world();
    healthy_ticks(50);

    for (int i = 0; i < 300; i++)   /* IMU silent */
    {
        s_can_rx[0]++; s_can_rx[1]++; s_cmds++;
        health_tick(&s_imu, ALL_ENC_OK);
    }
    CHECK((health_faults() & HEALTH_IMU) != 0u, "the IMU fault never appeared");

    healthy_ticks(500);             /* and back to normal */

    CHECK((health_faults() & HEALTH_IMU) == 0u,
          "the IMU came back but the fault stayed - sensor faults should follow "
          "the live state, only timing latches");
}

static void test_timing_faults_can_be_acknowledged(void)
{
    printf("a timing fault latches, and clearing it does not need a power cycle\n");

    reset_world();
    healthy_ticks(50);
    CHECK((health_faults() & HEALTH_TIMING) == 0u, "setup: timing already faulted");

    s_overruns = 1;                 /* one missed tick */
    healthy_tick();
    CHECK((health_faults() & HEALTH_TIMING) != 0u,
          "a missed tick did not raise a timing fault");

    healthy_ticks(200);
    CHECK((health_faults() & HEALTH_TIMING) != 0u,
          "the timing fault cleared itself");

    health_clear_latched();
    healthy_ticks(10);
    CHECK((health_faults() & HEALTH_TIMING) == 0u,
          "acknowledging the overrun did not clear the timing fault");

    /* A FURTHER overrun after the acknowledgement must raise it again. */
    s_overruns = 2;
    healthy_tick();
    CHECK((health_faults() & HEALTH_TIMING) != 0u,
          "a second missed tick after acknowledgement was ignored");
}

static void test_only_watched_subsystems_are_reported(void)
{
    printf("narrowing what is watched hides the rest\n");

    reset_world();
    health_set_expected(HEALTH_IMU);     /* bench setup: IMU only */
    CHECK(health_expected() == HEALTH_IMU, "health_expected did not follow the change");

    healthy_ticks(50);
    for (int i = 0; i < 400; i++)        /* everything but the IMU goes quiet */
    {
        s_imu.seq++;
        health_tick(&s_imu, ALL_ENC_OK);
    }

    CHECK(health_faults() == 0u,
          "unwatched subsystems were still reported: 0x%lx",
          (unsigned long)health_faults());

    /* Widening again must reveal what was there all along. */
    health_set_expected(HEALTH_EXPECTED_ROBOT);
    CHECK((health_faults() & (HEALTH_CAN1 | HEALTH_CAN2)) != 0u,
          "the buses were quiet the whole time but widening did not reveal it");
}

static void test_the_blink_code_names_the_first_fault(void)
{
    printf("the blink code names a fault, lowest number first\n");

    reset_world();
    healthy_ticks(50);

    /* Stop the IMU (blink 1) and both buses (blinks 3 and 4) together. */
    for (int i = 0; i < 400; i++)
    {
        s_cmds++;
        health_tick(&s_imu, ALL_ENC_OK);
    }

    CHECK(health_blink_code() == 1u,
          "with the IMU and both buses down the code was %u, expected 1",
          health_blink_code());

    /* With only the buses down it should name the first of those. */
    reset_world();
    healthy_ticks(50);
    for (int i = 0; i < 400; i++)
    {
        s_imu.seq++; s_cmds++;
        health_tick(&s_imu, ALL_ENC_OK);
    }
    CHECK(health_blink_code() == 3u,
          "with both buses down the code was %u, expected 3", health_blink_code());
}

int main(void)
{
    printf("health.c host tests\n");
    printf("-------------------\n");

    test_a_healthy_robot_reports_nothing();
    test_a_silent_sensor_is_noticed_but_not_instantly();
    test_each_subsystem_is_blamed_separately();
    test_a_bad_encoder_reading_is_tolerated_briefly();
    test_a_sensor_fault_follows_the_live_state();
    test_timing_faults_can_be_acknowledged();
    test_only_watched_subsystems_are_reported();
    test_the_blink_code_names_the_first_fault();

    printf("-------------------\n");
    if (s_fail)
    {
        printf("FAILED (%d)\n", s_fail);
        return 1;
    }
    printf("PASSED (0 failures)\n");
    return 0;
}
