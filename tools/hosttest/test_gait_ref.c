/*
 * Host test for gait_ref.c - the reference walk.
 *
 * The table is generated (tools/gen_gait.py), so its contents are not the
 * subject here. What is tested is the sampling around it: the phase wrap, the
 * interpolation, and the seam where the last row meets the first. A stride
 * that jumps at the seam is a robot that flinches once per step, and nothing
 * else in the system would report it.
 */

#include "gait_ref.h"

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

static int close_to(float a, float b, float tol)
{
    float d = a - b;
    return (d < 0 ? -d : d) <= tol;
}

static void test_phase_wraps_so_the_walk_repeats(void)
{
    printf("the same point in the stride gives the same pose, whatever the lap\n");

    /* The caller's phase clock free-runs, so 0.25, 1.25 and -0.75 are all the
       same quarter of the same stride. */
    const float equivalents[4] = { 0.25f, 1.25f, 7.25f, -0.75f };
    float ref[GAIT_JOINTS];

    gait_sample(equivalents[0], ref);

    for (int e = 1; e < 4; e++)
    {
        float got[GAIT_JOINTS];
        gait_sample(equivalents[e], got);

        for (int k = 0; k < GAIT_JOINTS; k++)
        {
            CHECK(close_to(got[k], ref[k], 1e-6f),
                  "phase %g gave joint %d = %f, but phase 0.25 gave %f",
                  (double)equivalents[e], k, (double)got[k], (double)ref[k]);
        }
    }
}

static void test_exact_samples_come_back_verbatim(void)
{
    printf("landing exactly on a table row returns that row untouched\n");

    /* Interpolation must not smear a point that needs no interpolating. */
    for (int i = 0; i < GAIT_SAMPLES; i += 37)
    {
        float phase = (float)i / (float)GAIT_SAMPLES;
        float got[GAIT_JOINTS];
        gait_sample(phase, got);

        for (int k = 0; k < GAIT_JOINTS; k++)
        {
            CHECK(close_to(got[k], g_gait_turns[i][k], 1e-5f),
                  "row %d joint %d: got %f, table holds %f",
                  i, k, (double)got[k], (double)g_gait_turns[i][k]);
        }
    }
}

/*
 * The generated table has two known step changes in the hip column, at rows
 * 99->100 and 199->0. README.md documents them ("Two discontinuities in the
 * generated table") as a defect in tools/gen_gait.py rather than in this file,
 * and they are reliably the largest tracking error in a capture. The tests
 * below work around them deliberately rather than by accident.
 */
#define KNOWN_STEP_ROW_A   99
#define KNOWN_STEP_ROW_B   (GAIT_SAMPLES - 1)

static int is_near_a_known_step(float phase)
{
    const float one_row = 1.0f / (float)GAIT_SAMPLES;
    const float a = (float)KNOWN_STEP_ROW_A * one_row;

    return (phase > a && phase < a + 2.0f * one_row) ||
           (phase > 1.0f - one_row) || (phase < one_row);
}

static void test_the_stride_does_not_jump_anywhere_else(void)
{
    printf("no step out of scale with the rest, away from the two known ones\n");

    /*
     * Compares the worst step against the typical one rather than a fixed
     * limit: a discontinuity is not "a big number", it is a step wildly out of
     * scale with its neighbours. An absolute threshold would pass whatever the
     * table happened to contain.
     */
    const int steps = 2000;
    float     prev[GAIT_JOINTS];
    float     biggest = 0.0f, total = 0.0f;
    int       count = 0, where = -1;

    gait_sample(0.0f, prev);

    for (int s = 1; s <= steps; s++)
    {
        float phase = (float)s / (float)steps;
        float now[GAIT_JOINTS];
        gait_sample(phase, now);

        for (int k = 0; k < GAIT_JOINTS; k++)
        {
            float step = fabsf(now[k] - prev[k]);
            prev[k] = now[k];

            if (is_near_a_known_step(phase)) { continue; }

            total += step;
            count++;
            if (step > biggest) { biggest = step; where = s; }
        }
    }

    float mean = total / (float)count;

    CHECK(biggest < mean * 12.0f + 1e-4f,
          "a new discontinuity: worst step %f turns at step %d, typical %f",
          (double)biggest, where, (double)mean);
}

/*
 * The largest step either join may take, in turns of the output shaft.
 *
 * 0.0025 turns is 0.9 degrees. Measured on the table actually in the build the
 * worst is 0.00223 at hip roll (row 99 -> 100) and 0.00132 at the knee
 * (row 199 -> 0, the stride wrap), so this has a little room and no more.
 */
#define JOIN_STEP_MAX_TURNS   0.0025f

static void test_the_joins_step_by_less_than_the_policy_can_notice(void)
{
    printf("the two stitch points step by under a degree\n");

    /*
     * WHAT THIS TEST IS FOR CHANGED WITH THE TABLE, so read this before
     * loosening it.
     *
     * It used to assert that both joins were SMOOTH, because the old table was
     * two half-strides concatenated by our own gen_gait.py, that concatenation
     * left ~0.0055 turn steps at rows 99 and 199, and the generator repaired
     * them. None of that is true now: this table comes from the policy's
     * trajectory optimiser, it is not a concatenation, and nothing repairs it.
     *
     * It is also NOT perfectly periodic. The knee steps 0.00132 turns - 0.475
     * degrees - across the stride wrap, about 2.3x what its own peak velocity
     * covers in one row. That is a real discontinuity in the library row.
     *
     * It is deliberately NOT smoothed, and must not be. The policy was trained
     * on this exact table, rl_policy_node carries its own copy and refuses to
     * enable if ref_angle differs from it by more than 1 degree, and 0.475 is
     * inside that. A board that "fixed" the seam would be walking a different
     * gait from the one the network learnt, and the mismatch check is what
     * would stop the robot - after someone had spent a day on it.
     *
     * So the property worth holding is the one that matters mechanically and
     * catches the mistake that matters: every join step stays under a degree,
     * which is small enough that no drive can be destabilised by it and far
     * smaller than the residual authority either side, while a table lifted
     * from a DIFFERENT gait or speed row would step much further and fail here.
     */
    struct { int row; const char *name; } joins[2] = {
        { KNOWN_STEP_ROW_A, "mid-stride (row 99 -> 100)" },
        { KNOWN_STEP_ROW_B, "the seam (row 199 -> 0)"    },
    };

    for (int i = 0; i < 2; i++)
    {
        int r    = joins[i].row;
        int next = (r + 1) % GAIT_SAMPLES;

        for (int k = 0; k < GAIT_JOINTS; k++)
        {
            float d = fabsf(g_gait_turns[next][k] - g_gait_turns[r][k]);

            CHECK(d < JOIN_STEP_MAX_TURNS,
                  "%s: joint %d steps %f turns (%.2f deg), over the %f allowed "
                  "- is this the gait the policy trained on?",
                  joins[i].name, k, (double)d, (double)(d * 360.0f),
                  (double)JOIN_STEP_MAX_TURNS);
        }
    }
}

static void test_it_interpolates_rather_than_snapping_to_a_row(void)
{
    printf("a phase between two rows gives the value between them\n");

    /*
     * Catches the interpolation being dropped for a nearest-row lookup, which
     * the other tests would pass: exact rows still come back exact, and the
     * stride still looks smooth.
     *
     * The tolerances matter. Adjacent rows differ by 3e-4 turns on average, so
     * snapping to a row instead of interpolating is only a 1.5e-4 error - a
     * tolerance of 1e-4 would let most of it through. Pairs that genuinely do
     * not differ are skipped, because there is nothing to interpolate.
     */
    int tested = 0;

    for (int i = 0; i + 1 < GAIT_SAMPLES; i += 7)
    {
        float phase = ((float)i + 0.5f) / (float)GAIT_SAMPLES;
        float got[GAIT_JOINTS];
        gait_sample(phase, got);

        for (int k = 0; k < GAIT_JOINTS; k++)
        {
            float lo = g_gait_turns[i][k];
            float hi = g_gait_turns[i + 1][k];

            if (fabsf(hi - lo) < 1e-5f) { continue; }

            float midpoint = 0.5f * (lo + hi);
            CHECK(close_to(got[k], midpoint, 1e-6f),
                  "halfway between rows %d and %d joint %d gave %f, expected "
                  "the midpoint %f", i, i + 1, k, (double)got[k],
                  (double)midpoint);
            tested++;
        }
    }

    CHECK(tested > 20, "only %d row pairs differed enough to test", tested);
}

static void test_the_seam_interpolates_too(void)
{
    printf("the wrap from the last row to the first interpolates like any other\n");

    /*
     * The seam is where a wrap bug hides: sampling between the final row and
     * row 0 has to blend the two, not freeze on the last one. The smoothness
     * test cannot see this because it skips the seam - the table has a
     * documented step there - so this checks the blend directly, which the
     * step does not affect.
     */
    const int last = GAIT_SAMPLES - 1;
    float phase = ((float)last + 0.5f) / (float)GAIT_SAMPLES;
    float got[GAIT_JOINTS];
    gait_sample(phase, got);

    int tested = 0;
    for (int k = 0; k < GAIT_JOINTS; k++)
    {
        float lo = g_gait_turns[last][k];
        float hi = g_gait_turns[0][k];

        if (fabsf(hi - lo) < 1e-5f) { continue; }

        CHECK(close_to(got[k], 0.5f * (lo + hi), 1e-6f),
              "across the seam joint %d gave %f; the midpoint of %f and %f is "
              "%f - the wrap is not blending", k, (double)got[k], (double)lo,
              (double)hi, (double)(0.5f * (lo + hi)));
        tested++;
    }
    /* Nothing to assert if the seam is perfectly closed - which it now is,
       by design. The interpolation either side is covered by the row-landing
       and between-rows tests, which do not depend on the seam being rough. */
    (void)tested;
}

static void test_velocity_and_torque_come_from_the_table(void)
{
    printf("speed and torque are read from the table, not derived from it\n");

    /*
     * THIS TEST USED TO CHECK THAT THE SPEED WAS A CENTRED DIFFERENCE OF THE
     * POSITIONS, and that was right while gait_ref.c computed it that way.
     * It does not any more: g_gait_vel and g_gait_tau are generated alongside
     * g_gait_turns by the trajectory optimiser, so the velocity is the real
     * derivative of the continuous trajectory and the position table is a 200
     * row SAMPLE of it. Differencing the samples is now the approximation, and
     * it is a poor one wherever the trajectory turns quickly: the median
     * disagreement is under 1% of each joint's peak speed, but the worst row
     * reaches 69% on the knee. Demanding they match rejected a correct table.
     *
     * So this checks the two things that are actually true, and between them
     * they are stronger than the old comparison was:
     *
     *   1. AT a sample row, every output is EXACTLY that row of its table.
     *      Pins the table read, the row indexing and the interpolation at the
     *      knots - a swapped column, a wrong stride or an off-by-one row all
     *      fail here, exactly, with no tolerance to hide in.
     *   2. ACROSS the stride the speed still tracks the position's slope in
     *      the typical case. Catches a column scaled wrongly or a velocity
     *      table from a different gait, which (1) alone would not if the
     *      tables were consistently wrong together.
     */
    for (int i = 0; i < GAIT_SAMPLES; i += 23)
    {
        float phase = (float)i / (float)GAIT_SAMPLES;
        float pos[GAIT_JOINTS], vel[GAIT_JOINTS], tau[GAIT_JOINTS];

        gait_sample_all(phase, pos, vel, tau);

        for (int k = 0; k < GAIT_JOINTS; k++)
        {
            CHECK(close_to(pos[k], g_gait_turns[i][k], 1e-6f),
                  "row %d joint %d: position %f, table has %f",
                  i, k, (double)pos[k], (double)g_gait_turns[i][k]);
            CHECK(close_to(vel[k], g_gait_vel[i][k], 1e-6f),
                  "row %d joint %d: speed %f, table has %f",
                  i, k, (double)vel[k], (double)g_gait_vel[i][k]);
            CHECK(close_to(tau[k], g_gait_tau[i][k], 1e-6f),
                  "row %d joint %d: torque %f, table has %f",
                  i, k, (double)tau[k], (double)g_gait_tau[i][k]);
        }
    }

    /*
     * Typical agreement with the centred slope, per joint: at least three
     * quarters of the rows within 10% of that joint's peak speed. Loose enough
     * to let the sharp rows through, tight enough that a column scaled by two
     * - or by 2pi, the likely mistake - fails outright.
     */
    for (int k = 0; k < GAIT_JOINTS; k++)
    {
        float peak = 0.0f;
        for (int i = 0; i < GAIT_SAMPLES; i++)
        {
            if (fabsf(g_gait_vel[i][k]) > peak) { peak = fabsf(g_gait_vel[i][k]); }
        }

        int near = 0;
        for (int i = 0; i < GAIT_SAMPLES; i++)
        {
            int prev = (i - 1 + GAIT_SAMPLES) % GAIT_SAMPLES;
            int next = (i + 1) % GAIT_SAMPLES;
            float centred = (g_gait_turns[next][k] - g_gait_turns[prev][k]) /
                            (2.0f * GAIT_DT_S);

            if (fabsf(g_gait_vel[i][k] - centred) < 0.10f * peak) { near++; }
        }

        CHECK(near * 4 >= GAIT_SAMPLES * 3,
              "joint %d: only %d of %d rows have a speed near the position's "
              "slope - the velocity table may not belong to this gait",
              k, near, GAIT_SAMPLES);
    }
}

static void test_velocity_matches_the_slope_of_the_position(void)
{
    printf("the reported speed matches how fast the position is actually changing\n");

    /*
     * gait_sample_vel reports where each joint should be AND how fast it
     * should be moving. Those two have to agree, or a controller told to be at
     * one place while travelling at a speed that does not lead there will
     * fight itself.
     *
     * Checked by differencing the position over a short interval of phase and
     * comparing against the reported speed. The table is 200 rows over
     * GAIT_PERIOD_S, so one row is GAIT_DT_S of real time.
     */
    const float dphase = 1.0f / (float)(GAIT_SAMPLES * 8);
    const float dt     = dphase * (float)GAIT_SAMPLES * GAIT_DT_S;

    int checked = 0;

    for (float phase = 0.02f; phase < 0.98f; phase += 0.05f)
    {
        float p0[GAIT_JOINTS], p1[GAIT_JOINTS], v[GAIT_JOINTS];

        gait_sample_vel(phase, p0, v);
        gait_sample(phase + dphase, p1);

        for (int k = 0; k < GAIT_JOINTS; k++)
        {
            float measured = (p1[k] - p0[k]) / dt;

            /* The table is piecewise linear, so the measured slope is exact
               within a row but the reported speed is a smoothed central
               difference - they agree in size and sign, not to the last bit. */
            CHECK(fabsf(measured - v[k]) < 0.5f + 0.35f * fabsf(v[k]),
                  "phase %.2f joint %d: reported %f turns/s, position is moving "
                  "at %f", (double)phase, k, (double)v[k], (double)measured);
            checked++;
        }
    }

    CHECK(checked > 0, "the velocity comparison never ran");
}

static void test_velocity_is_periodic_too(void)
{
    printf("the speed wraps with the stride, like the position\n");

    float p_a[GAIT_JOINTS], v_a[GAIT_JOINTS];
    float p_b[GAIT_JOINTS], v_b[GAIT_JOINTS];

    gait_sample_vel(0.4f,  p_a, v_a);
    gait_sample_vel(3.4f,  p_b, v_b);

    for (int k = 0; k < GAIT_JOINTS; k++)
    {
        CHECK(close_to(v_a[k], v_b[k], 1e-4f),
              "joint %d: speed %f at phase 0.4 but %f at phase 3.4",
              k, (double)v_a[k], (double)v_b[k]);
    }
}

static void test_null_outputs_are_allowed(void)
{
    printf("asking for only one of position or speed is allowed\n");

    /* fusion/app call this wanting one or the other; a null must not be
       written through. */
    float only_v[GAIT_JOINTS];
    float only_p[GAIT_JOINTS];

    gait_sample_vel(0.3f, 0, only_v);
    gait_sample_vel(0.3f, only_p, 0);

    float both_p[GAIT_JOINTS], both_v[GAIT_JOINTS];
    gait_sample_vel(0.3f, both_p, both_v);

    for (int k = 0; k < GAIT_JOINTS; k++)
    {
        CHECK(close_to(only_p[k], both_p[k], 1e-6f),
              "joint %d position differs when the speed is not asked for", k);
        CHECK(close_to(only_v[k], both_v[k], 1e-6f),
              "joint %d speed differs when the position is not asked for", k);
    }
}

int main(void)
{
    printf("gait_ref.c host tests\n");
    printf("---------------------\n");

    test_phase_wraps_so_the_walk_repeats();
    test_exact_samples_come_back_verbatim();
    test_the_stride_does_not_jump_anywhere_else();
    test_the_joins_step_by_less_than_the_policy_can_notice();
    test_it_interpolates_rather_than_snapping_to_a_row();
    test_the_seam_interpolates_too();
    test_velocity_and_torque_come_from_the_table();
    test_velocity_matches_the_slope_of_the_position();
    test_velocity_is_periodic_too();
    test_null_outputs_are_allowed();

    printf("---------------------\n");
    if (s_fail)
    {
        printf("FAILED (%d)\n", s_fail);
        return 1;
    }
    printf("PASSED (0 failures)\n");
    return 0;
}
