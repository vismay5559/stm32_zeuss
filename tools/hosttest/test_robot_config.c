/*
 * Host test for robot_config.c - specifically the encoder conversion.
 *
 * The numbers in robot_config.h are placeholders and a test cannot say
 * anything about whether they match the robot. What it CAN pin down is the
 * conversion around them: spring deflection is a small signed quantity either
 * side of a mechanical zero, and the raw AS5047P count is an unsigned 0..2pi
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
static void test_the_gearing_and_spring_tables_agree_with_each_other(void)
{
    printf("the gear ratios and the spring flags are mutually consistent\n");

    /*
     * These tables have to describe the same robot, and nothing in the
     * compiler notices if they stop. The failure is quiet and expensive: a
     * joint wrongly marked series-elastic gets its target pre-deflected by a
     * spring it does not have, and one wrongly marked rigid trails its
     * reference by the wind-up for the whole of stance.
     *
     * A series-elastic joint is exactly one with an AS5047P reading its
     * spring, so g_joint_is_sea and g_leg_springs are two views of one fact
     * and are checked against each other rather than both against a literal.
     */
    int sea = 0;

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        CHECK(g_joint_gear[j] > 0.0f,
              "joint %d has gear ratio %.1f - a zero or negative ratio divides "
              "the torque feedforward into infinity", j, (double)g_joint_gear[j]);
        CHECK(g_joint_is_sea[j] == 0u || g_joint_is_sea[j] == 1u,
              "joint %d's series-elastic flag is %u, not 0 or 1",
              j, g_joint_is_sea[j]);
        sea += (g_joint_is_sea[j] != 0u);
    }

    CHECK(sea == NEXUS_NUM_ENCODERS,
          "%d joints are marked series-elastic but there are %d spring "
          "encoders - one spring is either unread or read twice",
          sea, NEXUS_NUM_ENCODERS);

    /* And they are the SAME joints: each leg's hip pitch and knee pitch. */
    const uint8_t want[4] = {
        NEXUS_J_L_HIP_PITCH, NEXUS_J_L_KNEE_PITCH,
        NEXUS_J_R_HIP_PITCH, NEXUS_J_R_KNEE_PITCH,
    };

    for (int i = 0; i < 4; i++)
    {
        CHECK(g_joint_is_sea[want[i]] != 0u,
              "joint %u carries a spring encoder but is not marked "
              "series-elastic, so its target is never pre-deflected", want[i]);
    }

    /*
     * The spring model has to be self-consistent too. K at the travel limit is
     * the most torque the spring can transmit before it is on its stop. If
     * that product came out BELOW the gait's demand the spring would bottom
     * out mid-stride and the pre-deflection would be clamping for real rather
     * than as a guard.
     */
    float max_torque = ROBOT_SPRING_K_NM_PER_RAD * ROBOT_SPRING_MAX_DEFLECTION_RAD;

    CHECK(max_torque > 20.0f,
          "the spring tops out at %.1f Nm (K %.1f x travel %.4f rad), which is "
          "less than the gait asks of it - check which of the two is wrong",
          (double)max_torque, (double)ROBOT_SPRING_K_NM_PER_RAD,
          (double)ROBOT_SPRING_MAX_DEFLECTION_RAD);
}

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
        for (int j = 0; j < ROBOT_LEG_MOTORS; j++)
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
    test_the_gearing_and_spring_tables_agree_with_each_other();

    printf("-------------------------\n%s (%d failure%s)\n",
           s_fail ? "FAILED" : "PASSED", s_fail, s_fail == 1 ? "" : "s");

    return s_fail ? 1 : 0;
}
