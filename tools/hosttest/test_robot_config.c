/*
 * Host test for robot_config.c - specifically the encoder conversion.
 *
 * The numbers in robot_config.h are placeholders and a test cannot say
 * anything about whether they match the robot. What it CAN pin down is the
 * conversion around them: spring deflection is a small signed quantity either
 * side of a mechanical zero, and the raw AS5048A count is an unsigned 0..2pi
 * angle that wraps. Getting that wrong puts a 6.28 rad step into the torque
 * the Pi computes, once per revolution, for any joint whose rest position
 * happens to sit near the wrap point.
 */

#include "robot_config.h"

#include <math.h>
#include <stdio.h>

static int s_fail;

#define CHECK(cond, fmt, ...)                                            \
    do {                                                                 \
        if (!(cond)) {                                                   \
            s_fail++;                                                    \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__);                  \
        }                                                                \
    } while (0)

#define COUNTS      16384
#define TWO_PI      6.28318531f
#define PI_F        3.14159265f
#define PER_COUNT   (TWO_PI / (float)COUNTS)

static void test_always_within_pi(void)
{
    printf("every raw count maps into -pi..+pi\n");

    for (int e = 0; e < NEXUS_NUM_ENCODERS; e++)
    {
        for (int raw = 0; raw < COUNTS; raw += 37)   /* 443 samples per encoder */
        {
            float d = robot_spring_deflection((uint8_t)e, (uint16_t)raw);

            if (!(d >= -PI_F - 1e-3f && d <= PI_F + 1e-3f))
            {
                s_fail++;
                printf("  FAIL  enc %d raw %d -> %f, outside +/-pi\n",
                       e, raw, (double)d);
                return;      /* one report is enough */
            }
        }
    }
}

/*
 * The thing the old code got wrong. Two counts either side of the zero must
 * differ by two counts' worth of angle - not by a full turn.
 */
static void test_continuous_across_the_zero(void)
{
    printf("deflection is continuous through the mechanical zero\n");

    for (int e = 0; e < NEXUS_NUM_ENCODERS; e++)
    {
        uint16_t zero = g_spring_enc[e].zero_counts;

        uint16_t just_below = (uint16_t)((zero + COUNTS - 1) % COUNTS);
        uint16_t just_above = (uint16_t)((zero + 1) % COUNTS);

        float below = robot_spring_deflection((uint8_t)e, just_below);
        float above = robot_spring_deflection((uint8_t)e, just_above);

        CHECK(fabsf(above - below) < (3.0f * PER_COUNT),
              "enc %d: one count either side of zero differ by %f rad "
              "(expected about %f) - the wrap is not being folded",
              e, (double)fabsf(above - below), (double)(2.0f * PER_COUNT));

        /* And they must straddle zero with the right signs. */
        CHECK(below < 0.0f, "enc %d: one count below zero read %f, expected negative",
              e, (double)below);
        CHECK(above > 0.0f, "enc %d: one count above zero read %f, expected positive",
              e, (double)above);
    }
}

static void test_zero_reads_zero(void)
{
    printf("the calibrated zero reads exactly zero deflection\n");

    for (int e = 0; e < NEXUS_NUM_ENCODERS; e++)
    {
        float d = robot_spring_deflection((uint8_t)e, g_spring_enc[e].zero_counts);

        CHECK(fabsf(d) < 1e-6f, "enc %d: zero count read %f", e, (double)d);
    }
}

static void test_half_turn_is_pi(void)
{
    printf("half a turn from zero is +/-pi, not 0 and not 2pi\n");

    for (int e = 0; e < NEXUS_NUM_ENCODERS; e++)
    {
        uint16_t half = (uint16_t)((g_spring_enc[e].zero_counts + COUNTS / 2) % COUNTS);
        float    d    = robot_spring_deflection((uint8_t)e, half);

        CHECK(fabsf(fabsf(d) - PI_F) < 1e-3f,
              "enc %d: half a turn read %f, expected +/-pi", e, (double)d);
    }
}

static void test_out_of_range_index(void)
{
    printf("an out-of-range encoder index reads zero rather than off the end\n");

    CHECK(robot_spring_deflection(NEXUS_NUM_ENCODERS, 1234u) == 0.0f,
          "out-of-range index did not return 0");
    CHECK(robot_spring_deflection(200u, 1234u) == 0.0f,
          "far out-of-range index did not return 0");
}

/*
 * Not a correctness test - a reminder. If someone sets the calibrated flag
 * without touching the numbers, this fails and says why.
 */
static void test_calibration_flag_matches_the_data(void)
{
    printf("the calibrated flag is not set while the values are placeholders\n");

    if (!robot_config_is_calibrated())
    {
        printf("  note: robot_config.h is marked UNCALIBRATED, so the estimator\n"
               "        will report CONVERGING and never OK. That is correct\n"
               "        until the robot has been measured.\n");
        return;
    }

    int all_default = 1;

    for (int leg = 0; leg < 2; leg++)
    {
        for (int j = 0; j < KIN_LEG_JOINTS; j++)
        {
            if (g_leg_joints[leg][j].offset != 0.0f)
            {
                all_default = 0;
            }
        }
    }
    for (int e = 0; e < NEXUS_NUM_ENCODERS; e++)
    {
        if (g_spring_enc[e].zero_counts != 0u)
        {
            all_default = 0;
        }
    }

    CHECK(!all_default,
          "ROBOT_CONFIG_CALIBRATED is 1 but every joint offset and encoder "
          "zero is still at its placeholder value");
}

int main(void)
{
    printf("robot_config.c host tests\n-------------------------\n");

    test_always_within_pi();
    test_continuous_across_the_zero();
    test_zero_reads_zero();
    test_half_turn_is_pi();
    test_out_of_range_index();
    test_calibration_flag_matches_the_data();

    printf("-------------------------\n%s (%d failure%s)\n",
           s_fail ? "FAILED" : "PASSED", s_fail, s_fail == 1 ? "" : "s");

    return s_fail ? 1 : 0;
}
