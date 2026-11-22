/*
 * Host test for soft_start.c - how an arm gets from a standing robot into the
 * gait.
 *
 * What a failure here would mean on the robot: the first armed tick jumps the
 * legs 20-35 degrees at full stiffness with the feet on the ground, or the
 * gait clock starts while the robot is still being brought to its start pose,
 * or a policy's residual reaches the drives before the gait is running.
 *
 * Every expected value is written down before the module runs.
 */

#include "soft_start.h"

#include <math.h>
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

#define N          NEXUS_NUM_JOINTS
#define RAMP       ((int)(SOFT_START_RAMP_S * 1000.0f + 0.5f))
#define DEG        (1.0f / 360.0f)          /* one degree, in turns */

/* Standing: every joint at zero, except the right hip roll at its real 4 deg. */
static float s_stand[N];
/* The phase-0 pose of the trained gait, as printed from the policy's own
   table on 2026-11-22 (degrees -> turns), the pose a held arm must end at. */
static const float s_pose0_deg[N] = { 22.4f, -5.4f, -2.9f, -18.7f, -21.8f, 6.5f, 35.1f, -13.6f };
static float s_pose0[N];
/* A running gait somewhere else entirely, moving, and pushing. */
static float s_gait[N], s_gvel[N], s_gtau[N];

static void setup(void)
{
    for (int j = 0; j < N; j++)
    {
        s_stand[j] = 0.0f;
        s_pose0[j] = s_pose0_deg[j] * DEG;
        s_gait[j]  = (10.0f + (float)j) * DEG;
        s_gvel[j]  = 0.05f;
        s_gtau[j]  = 3.0f + (float)j;
    }
    s_stand[5] = 4.0f * DEG;
}

static void tick(float ref[N], float vel[N], float tau[N])
{
    soft_start_tick(s_gait, s_gvel, s_gtau, s_pose0, ref, vel, tau);
}

static void test_a_held_arm_starts_where_the_robot_is_and_ends_at_the_start_pose(void)
{
    printf("a held arm: first target is the standing pose, last is the phase-0 pose\n");
    setup();
    soft_start_idle();
    soft_start_armed(s_stand, 1u);

    float ref[N], vel[N], tau[N], prev[N];
    tick(ref, vel, tau);

    /* Tick 0: exactly where it stands. Anything else is a jump. */
    for (int j = 0; j < N; j++)
    {
        CHECK(fabsf(ref[j] - s_stand[j]) < 1e-7f,
              "joint %d: first target %.4f deg, standing at %.4f - a jump on arming",
              j, (double)(ref[j] * 360.0f), (double)(s_stand[j] * 360.0f));
        CHECK(vel[j] == 0.0f && tau[j] == 0.0f,
              "joint %d: the ramp starts with velocity %.4f / torque %.2f, expected 0",
              j, (double)vel[j], (double)tau[j]);
    }
    CHECK(soft_start_clock_frozen(), "the gait clock runs during a held ramp");
    CHECK(soft_start_ignores_residual(), "a residual is passed through during the ramp");

    /*
     * Smoothstep peak speed is 1.5x the average. The largest move here is the
     * right knee, 35.1 deg in 3 s: no tick may step more than 1.5 * 35.1/3000
     * deg, about 0.018 deg - against the 35 deg in 4 ms the old arm did.
     */
    float worst_step = 0.0f, worst_allowed = 0.0f;
    memcpy(prev, ref, sizeof(prev));
    for (int t = 1; t < RAMP; t++)
    {
        tick(ref, vel, tau);
        for (int j = 0; j < N; j++)
        {
            float step    = fabsf(ref[j] - prev[j]);
            float allowed = 1.5f * fabsf(s_pose0[j] - s_stand[j]) / (float)RAMP + 1e-7f;
            if (step - allowed > worst_step - worst_allowed)
            {
                worst_step = step;
                worst_allowed = allowed;
            }
            /* The velocity hint must be the slope actually being commanded. */
            float slope = (ref[j] - prev[j]) * 1000.0f;
            CHECK(fabsf(vel[j] - slope) < 2e-4f || t < 2,
                  "t=%d joint %d: velocity feedforward %.5f, commanded slope %.5f turns/s",
                  t, j, (double)vel[j], (double)slope);
            CHECK(tau[j] == 0.0f, "t=%d joint %d: torque %.2f pushed while held", t, j, (double)tau[j]);
        }
        memcpy(prev, ref, sizeof(prev));
        if (s_fail > 20) { return; }
    }
    CHECK(worst_step <= worst_allowed,
          "a ramp tick stepped %.5f deg, smoothstep allows %.5f",
          (double)(worst_step * 360.0f), (double)(worst_allowed * 360.0f));

    tick(ref, vel, tau);   /* the last ramp tick */
    CHECK(soft_start_mode() == SOFT_HOLD, "after %d ticks the mode is %d, expected HOLD",
          RAMP, (int)soft_start_mode());
    for (int k = 0; k < 500; k++) { tick(ref, vel, tau); }
    for (int j = 0; j < N; j++)
    {
        CHECK(fabsf(ref[j] - s_pose0[j]) < 1e-6f && vel[j] == 0.0f && tau[j] == 0.0f,
              "joint %d: holding at %.3f deg with v %.4f, expected the phase-0 pose %.3f still",
              j, (double)(ref[j] * 360.0f), (double)vel[j], (double)(s_pose0[j] * 360.0f));
    }
    CHECK(soft_start_clock_frozen(), "the clock started on its own during the hold");
    CHECK(soft_start_ignores_residual(), "a residual is passed through during the hold");
}

static void test_clearing_hold_releases_into_the_gait(void)
{
    printf("clearing HOLD at the start pose starts the gait; setting it again does not stop it\n");
    setup();
    soft_start_idle();
    soft_start_armed(s_stand, 1u);
    float ref[N], vel[N], tau[N];
    for (int t = 0; t < RAMP + 10; t++) { tick(ref, vel, tau); }

    soft_start_hold_flag(1u);               /* still held: nothing changes */
    CHECK(soft_start_mode() == SOFT_HOLD, "re-sending HOLD left the hold");

    soft_start_hold_flag(0u);
    CHECK(soft_start_mode() == SOFT_RUN, "clearing HOLD did not release (mode %d)",
          (int)soft_start_mode());
    CHECK(!soft_start_clock_frozen(), "released, but the clock is still frozen");
    CHECK(!soft_start_ignores_residual(), "released, but the residual is still ignored");

    tick(ref, vel, tau);
    for (int j = 0; j < N; j++)
    {
        CHECK(ref[j] == s_gait[j] && vel[j] == s_gvel[j] && tau[j] == s_gtau[j],
              "joint %d: running, but not the plain gait", j);
    }

    /* HOLD again once running: to stop the robot, ENABLE off - not a freeze
       mid-stride with a foot in the air. */
    soft_start_hold_flag(1u);
    CHECK(soft_start_mode() == SOFT_RUN, "HOLD stopped a running gait");
}

static void test_hold_cleared_mid_ramp_finishes_the_ramp_first(void)
{
    printf("HOLD cleared during the ramp: the ramp still finishes at the start pose, then runs\n");
    setup();
    soft_start_idle();
    soft_start_armed(s_stand, 1u);
    float ref[N], vel[N], tau[N];
    for (int t = 0; t < RAMP / 2; t++) { tick(ref, vel, tau); }

    soft_start_hold_flag(0u);
    CHECK(soft_start_mode() == SOFT_RAMP, "clearing HOLD cut the ramp short");
    CHECK(soft_start_clock_frozen(),
          "the clock started while the ramp was still heading for the frozen pose");

    for (int t = RAMP / 2; t < RAMP; t++) { tick(ref, vel, tau); }
    for (int j = 0; j < N; j++)
    {
        CHECK(fabsf(ref[j] - s_pose0[j]) < 1e-3f * DEG + 1e-6f,
              "joint %d: the ramp ended at %.3f deg, not the start pose %.3f",
              j, (double)(ref[j] * 360.0f), (double)(s_pose0[j] * 360.0f));
    }
    CHECK(soft_start_mode() == SOFT_RUN, "ended in mode %d, expected RUN", (int)soft_start_mode());
}

static void test_an_unheld_arm_blends_into_the_running_gait(void)
{
    printf("an arm without HOLD ramps into the stride in progress, feedforward faded in\n");
    setup();
    soft_start_idle();
    soft_start_armed(s_stand, 0u);
    CHECK(!soft_start_clock_frozen(), "an unheld arm froze the clock");

    float ref[N], vel[N], tau[N];
    tick(ref, vel, tau);
    for (int j = 0; j < N; j++)
    {
        CHECK(fabsf(ref[j] - s_stand[j]) < 1e-7f, "joint %d: unheld arm jumped on tick 0", j);
        CHECK(tau[j] == 0.0f, "joint %d: full torque %.2f on the first tick", j, (double)tau[j]);
    }
    for (int t = 1; t < RAMP / 2; t++) { tick(ref, vel, tau); }
    /* Halfway: smoothstep(0.5) = 0.5, so half the gait's torque. */
    CHECK(fabsf(tau[0] - 0.5f * s_gtau[0]) < 0.02f * s_gtau[0],
          "halfway through, torque %.3f, expected ~%.3f", (double)tau[0], (double)(0.5f * s_gtau[0]));
    for (int t = RAMP / 2; t < RAMP; t++) { tick(ref, vel, tau); }
    CHECK(soft_start_mode() == SOFT_RUN, "unheld ramp ended in mode %d, expected RUN",
          (int)soft_start_mode());
    tick(ref, vel, tau);
    CHECK(ref[3] == s_gait[3] && tau[3] == s_gtau[3], "after the ramp, not the plain gait");
}

static void test_disarming_means_the_next_arm_ramps_again(void)
{
    printf("disarm, re-arm somewhere else: the ramp starts again from the new pose\n");
    setup();
    soft_start_idle();
    soft_start_armed(s_stand, 0u);
    float ref[N], vel[N], tau[N];
    for (int t = 0; t < RAMP + 5; t++) { tick(ref, vel, tau); }

    soft_start_idle();
    CHECK(soft_start_mode() == SOFT_IDLE, "idle did not reset the mode");
    CHECK(soft_start_ignores_residual(), "idle passes a residual");

    float elsewhere[N];
    for (int j = 0; j < N; j++) { elsewhere[j] = -7.0f * DEG; }
    soft_start_armed(elsewhere, 1u);
    tick(ref, vel, tau);
    CHECK(fabsf(ref[2] - elsewhere[2]) < 1e-7f,
          "re-arm started from %.3f deg, the robot is at %.3f",
          (double)(ref[2] * 360.0f), (double)(elsewhere[2] * 360.0f));
}

static void test_return_from_the_hold_goes_back_to_where_it_armed(void)
{
    printf("RETURN at the start pose: ramps back to the arming pose, holds it, no jump\n");
    setup();
    soft_start_idle();
    soft_start_armed(s_stand, 1u);
    float ref[N], vel[N], tau[N], prev[N];
    for (int t = 0; t < RAMP + 100; t++) { tick(ref, vel, tau); }
    CHECK(soft_start_mode() == SOFT_HOLD, "setup: not holding (mode %d)", (int)soft_start_mode());

    soft_start_return_flag(1u);
    CHECK(soft_start_mode() == SOFT_RETURN, "RETURN in HOLD did not start the way back (mode %d)",
          (int)soft_start_mode());

    /* First tick back: still exactly at the start pose - no jump. */
    memcpy(prev, ref, sizeof(prev));
    tick(ref, vel, tau);
    for (int j = 0; j < N; j++)
    {
        CHECK(fabsf(ref[j] - s_pose0[j]) < 1e-7f,
              "joint %d: first tick of the return at %.4f deg, was holding %.4f - a jump",
              j, (double)(ref[j] * 360.0f), (double)(s_pose0[j] * 360.0f));
    }

    /* The way back is as gentle as the way out, pushes nothing, leaves the clock frozen. */
    float worst = 0.0f;
    for (int t = 1; t < RAMP; t++)
    {
        memcpy(prev, ref, sizeof(prev));
        tick(ref, vel, tau);
        for (int j = 0; j < N; j++)
        {
            float over = fabsf(ref[j] - prev[j])
                       - (1.5f * fabsf(s_pose0[j] - s_stand[j]) / (float)RAMP + 1e-7f);
            if (over > worst) { worst = over; }
            CHECK(tau[j] == 0.0f, "t=%d joint %d: torque %.2f pushed on the way back", t, j,
                  (double)tau[j]);
        }
        CHECK(soft_start_clock_frozen(), "t=%d: the gait clock ran on the way back", t);
        CHECK(soft_start_ignores_residual(), "t=%d: residual passed on the way back", t);
        if (s_fail > 20) { return; }
    }
    CHECK(worst <= 0.0f, "a return tick stepped %.5f deg more than smoothstep allows",
          (double)(worst * 360.0f));

    tick(ref, vel, tau);
    CHECK(soft_start_mode() == SOFT_HOME, "after the return ramp the mode is %d, expected HOME",
          (int)soft_start_mode());
    for (int k = 0; k < 500; k++) { tick(ref, vel, tau); }
    for (int j = 0; j < N; j++)
    {
        CHECK(fabsf(ref[j] - s_stand[j]) < 1e-6f && vel[j] == 0.0f && tau[j] == 0.0f,
              "joint %d: home at %.3f deg (v %.4f), expected the arming pose %.3f, still",
              j, (double)(ref[j] * 360.0f), (double)vel[j], (double)(s_stand[j] * 360.0f));
    }

    /* Home is final: clearing HOLD does not set off the gait from a standing robot. */
    soft_start_hold_flag(0u);
    soft_start_return_flag(0u);
    CHECK(soft_start_mode() == SOFT_HOME, "clearing HOLD at home started the gait (mode %d)",
          (int)soft_start_mode());
}

static void test_return_mid_ramp_turns_round_where_it_is(void)
{
    printf("RETURN halfway out: turns round from the current reference, ends at the arming pose\n");
    setup();
    soft_start_idle();
    soft_start_armed(s_stand, 1u);
    float ref[N], vel[N], tau[N], here[N];
    for (int t = 0; t < RAMP / 2; t++) { tick(ref, vel, tau); }
    memcpy(here, ref, sizeof(here));

    soft_start_return_flag(1u);
    tick(ref, vel, tau);
    for (int j = 0; j < N; j++)
    {
        CHECK(fabsf(ref[j] - here[j]) < 1e-7f,
              "joint %d: turned round from %.4f deg, the reference was at %.4f - a jump",
              j, (double)(ref[j] * 360.0f), (double)(here[j] * 360.0f));
    }
    for (int t = 1; t <= RAMP; t++) { tick(ref, vel, tau); }
    for (int j = 0; j < N; j++)
    {
        CHECK(fabsf(ref[j] - s_stand[j]) < 1e-6f, "joint %d: ended at %.3f deg, not the arming pose",
              j, (double)(ref[j] * 360.0f));
    }
}

static void test_return_is_refused_while_walking(void)
{
    printf("RETURN while the gait runs does nothing - a foot may be in the air\n");
    setup();
    soft_start_idle();
    soft_start_armed(s_stand, 0u);                 /* unheld: heading for RUN */
    float ref[N], vel[N], tau[N];
    tick(ref, vel, tau);
    soft_start_return_flag(1u);
    CHECK(soft_start_mode() == SOFT_RAMP, "RETURN hijacked a ramp into the running gait");
    for (int t = 0; t < RAMP + 5; t++) { tick(ref, vel, tau); }
    CHECK(soft_start_mode() == SOFT_RUN, "setup: not running (mode %d)", (int)soft_start_mode());
    soft_start_return_flag(1u);
    CHECK(soft_start_mode() == SOFT_RUN, "RETURN stopped a running gait mid-stride");

    soft_start_idle();
    soft_start_return_flag(1u);
    CHECK(soft_start_mode() == SOFT_IDLE, "RETURN did something while disarmed");
}

int main(void)
{
    printf("soft_start.c host tests\n");
    printf("-----------------------\n");

    test_a_held_arm_starts_where_the_robot_is_and_ends_at_the_start_pose();
    test_clearing_hold_releases_into_the_gait();
    test_hold_cleared_mid_ramp_finishes_the_ramp_first();
    test_an_unheld_arm_blends_into_the_running_gait();
    test_disarming_means_the_next_arm_ramps_again();
    test_return_from_the_hold_goes_back_to_where_it_armed();
    test_return_mid_ramp_turns_round_where_it_is();
    test_return_is_refused_while_walking();

    printf("-----------------------\n");
    if (s_fail)
    {
        printf("FAILED (%d)\n", s_fail);
        return 1;
    }
    printf("PASSED (0 failures)\n");
    return 0;
}
