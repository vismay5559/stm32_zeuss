/*
 * Host test for kinematics.c - where a foot is, given the leg's joint angles.
 *
 * This feeds the estimator's contact update: the leg says where the foot
 * should be relative to the body, the foot is known not to be moving because
 * it is on the ground, and the difference between those two is what corrects
 * the estimate of where the BODY is. Get the geometry wrong and the filter is
 * corrected towards somewhere the robot has never been.
 *
 * The tests do not check against hand-computed numbers from the same chain
 * that produced them - that would only prove the code agrees with itself.
 * They check things that must be true of any correct leg: a straight leg
 * hangs its own length below the hip, swinging the hip moves the foot along a
 * circle of that radius, a foot can never be further from the hip than the leg
 * is long, and the Jacobian must predict what actually happens when a joint
 * moves a little.
 */

#include "kinematics.h"
#include "robot_config.h"

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

#define LEG_LEN  (ROBOT_THIGH_LENGTH_M + ROBOT_SHANK_LENGTH_M + ROBOT_FOOT_HEIGHT_M)

static int close_to(inekf_real_t a, inekf_real_t b, inekf_real_t tol)
{
    inekf_real_t d = a - b;
    return (d < 0 ? -d : d) <= tol;
}

static inekf_real_t dist(const inekf_real_t *a, const inekf_real_t *b)
{
    inekf_real_t dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static void test_defaults_are_the_configured_geometry(void)
{
    printf("kin_defaults reports the geometry from robot_config.h\n");

    kin_params_t p;
    memset(&p, 0, sizeof(p));
    kin_defaults(&p);

    CHECK(close_to(p.thigh_length, ROBOT_THIGH_LENGTH_M, 1e-6f), "thigh length");
    CHECK(close_to(p.shank_length, ROBOT_SHANK_LENGTH_M, 1e-6f), "shank length");
    CHECK(close_to(p.foot_height,  ROBOT_FOOT_HEIGHT_M,  1e-6f), "foot height");

    /* The hips sit either side of the centre line, left positive in y. */
    CHECK(close_to(p.left_hip_offset[1],   ROBOT_HIP_OFFSET_Y_M, 1e-6f), "left hip y");
    CHECK(close_to(p.right_hip_offset[1], -ROBOT_HIP_OFFSET_Y_M, 1e-6f), "right hip y");
    CHECK(close_to(p.left_hip_offset[1], -p.right_hip_offset[1], 1e-6f),
          "the hips are not symmetric about the centre line");
}

static void test_straight_leg_hangs_its_own_length(void)
{
    printf("with every joint at zero the foot hangs one leg-length below the hip\n");

    kin_params_t p;
    kin_defaults(&p);

    const inekf_real_t q[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    inekf_real_t foot[3];

    kin_foot(&p, p.left_hip_offset, q, foot, NULL);

    CHECK(close_to(foot[0], 0.0f, 1e-5f), "straight leg is not directly below: x=%f",
          (double)foot[0]);
    CHECK(close_to(foot[1], p.left_hip_offset[1], 1e-5f),
          "straight leg drifted sideways: y=%f", (double)foot[1]);
    CHECK(close_to(foot[2], -LEG_LEN, 1e-5f),
          "straight leg is not one leg-length down: z=%f (expected %f)",
          (double)foot[2], (double)-LEG_LEN);
}

static void test_hip_pitch_swings_the_foot_on_a_circle(void)
{
    printf("swinging the hip moves the foot along a circle of leg-length radius\n");

    kin_params_t p;
    kin_defaults(&p);

    for (int deg = -60; deg <= 60; deg += 30)
    {
        const inekf_real_t th = (inekf_real_t)deg * 3.14159265f / 180.0f;
        const inekf_real_t q[4] = { th, 0.0f, 0.0f, 0.0f };
        inekf_real_t foot[3];

        kin_foot(&p, p.left_hip_offset, q, foot, NULL);

        /* Whatever the chain does internally, a rigid leg pivoting at the hip
           keeps the foot exactly one leg-length away from it. */
        CHECK(close_to(dist(foot, p.left_hip_offset), LEG_LEN, 1e-4f),
              "at %d deg the foot is %f from the hip, not %f",
              deg, (double)dist(foot, p.left_hip_offset), (double)LEG_LEN);

        /* Pitch is about y, so the foot must stay in the hip's y-plane. */
        CHECK(close_to(foot[1], p.left_hip_offset[1], 1e-5f),
              "hip pitch moved the foot sideways at %d deg", deg);

        /* Positive pitch carries the foot forward in -x, and it always
           stays below the hip for these angles. */
        CHECK(foot[2] < p.left_hip_offset[2], "the foot rose above the hip at %d deg", deg);
    }
}

static void test_hip_roll_moves_the_foot_sideways_only(void)
{
    printf("hip roll moves the foot sideways, not fore and aft\n");

    kin_params_t p;
    kin_defaults(&p);

    const inekf_real_t q[4] = { 0.0f, 0.35f, 0.0f, 0.0f };
    inekf_real_t foot[3];
    kin_foot(&p, p.left_hip_offset, q, foot, NULL);

    CHECK(close_to(foot[0], 0.0f, 1e-5f),
          "roll moved the foot fore/aft: x=%f", (double)foot[0]);
    CHECK(close_to(dist(foot, p.left_hip_offset), LEG_LEN, 1e-4f),
          "roll changed the distance from the hip");
    CHECK(foot[1] > p.left_hip_offset[1],
          "positive roll did not carry the foot outward");
}

static void test_the_foot_is_never_further_than_the_leg_is_long(void)
{
    printf("no combination of joints puts the foot beyond the leg's reach\n");

    kin_params_t p;
    kin_defaults(&p);

    for (int a = -6; a <= 6; a += 3)
    {
        for (int b = -6; b <= 6; b += 3)
        {
            for (int c = -6; c <= 6; c += 3)
            {
                const inekf_real_t q[4] = {
                    (inekf_real_t)a * 0.25f, (inekf_real_t)b * 0.25f,
                    (inekf_real_t)c * 0.25f, (inekf_real_t)a * 0.15f,
                };
                inekf_real_t foot[3];
                kin_foot(&p, p.right_hip_offset, q, foot, NULL);

                inekf_real_t reach = dist(foot, p.right_hip_offset);
                CHECK(reach <= LEG_LEN + 1e-4f,
                      "reach %f exceeds the leg length %f at q=(%.2f,%.2f,%.2f)",
                      (double)reach, (double)LEG_LEN,
                      (double)q[0], (double)q[1], (double)q[2]);
            }
        }
    }
}

static void test_bending_the_knee_shortens_the_reach(void)
{
    printf("bending the knee brings the foot closer to the hip\n");

    kin_params_t p;
    kin_defaults(&p);

    inekf_real_t prev = LEG_LEN + 1.0f;

    for (int deg = 0; deg <= 90; deg += 30)
    {
        const inekf_real_t q[4] = { 0.0f, 0.0f,
                                    (inekf_real_t)deg * 3.14159265f / 180.0f, 0.0f };
        inekf_real_t foot[3];
        kin_foot(&p, p.left_hip_offset, q, foot, NULL);

        inekf_real_t reach = dist(foot, p.left_hip_offset);
        CHECK(reach < prev + 1e-5f,
              "bending the knee to %d deg did not shorten the reach (%f then %f)",
              deg, (double)prev, (double)reach);
        prev = reach;
    }

    CHECK(prev < LEG_LEN - 0.05f,
          "a 90 degree knee bend barely shortened the leg (%f vs %f)",
          (double)prev, (double)LEG_LEN);
}

static void test_the_jacobian_predicts_what_actually_happens(void)
{
    printf("the Jacobian predicts the foot's motion for a small joint move\n");

    /*
     * The strongest check available without a second implementation: the
     * Jacobian claims d(foot)/d(joint), so for a small step dq it must predict
     * the change the forward kinematics actually produces. This is independent
     * of how the Jacobian is worked out, so it would catch a wrong step size,
     * a transposed layout, or a column in the wrong place.
     */
    kin_params_t p;
    kin_defaults(&p);

    const inekf_real_t q[4]  = { 0.20f, -0.15f, 0.60f, -0.10f };
    const inekf_real_t dq[4] = { 1e-3f, -8e-4f, 6e-4f,  9e-4f };

    inekf_real_t p0[3], J[12];
    kin_foot(&p, p.left_hip_offset, q, p0, J);

    inekf_real_t q2[4];
    for (int i = 0; i < 4; i++)
    {
        q2[i] = q[i] + dq[i];
    }

    inekf_real_t p1[3];
    kin_foot(&p, p.left_hip_offset, q2, p1, NULL);

    for (int r = 0; r < 3; r++)
    {
        inekf_real_t predicted = 0.0f;
        for (int c = 0; c < KIN_LEG_JOINTS; c++)
        {
            predicted += J[r * KIN_LEG_JOINTS + c] * dq[c];
        }
        inekf_real_t actual = p1[r] - p0[r];

        /* Second-order terms are O(|dq|^2) ~ 1e-6, so anything much larger
           than that is the Jacobian being wrong rather than curvature. */
        CHECK(close_to(predicted, actual, 5e-6f),
              "row %d: predicted %e, actually moved %e", r,
              (double)predicted, (double)actual);
    }
}

static void test_a_joint_that_cannot_move_the_foot_has_a_zero_column(void)
{
    printf("the ankle cannot move the foot along the axis it turns about\n");

    /*
     * Every joint in this leg is a pitch except hip roll, and all the pitch
     * joints turn about y - so none of them can move the contact point in y.
     * Those Jacobian entries must be zero, whatever the pose.
     */
    kin_params_t p;
    kin_defaults(&p);

    const inekf_real_t q[4] = { 0.3f, 0.0f, 0.5f, -0.2f };
    inekf_real_t foot[3], J[12];
    kin_foot(&p, p.left_hip_offset, q, foot, J);

    const int pitch_cols[3] = { 0, 2, 3 };   /* hip pitch, knee, ankle */
    for (int k = 0; k < 3; k++)
    {
        inekf_real_t dy = J[1 * KIN_LEG_JOINTS + pitch_cols[k]];
        CHECK(close_to(dy, 0.0f, 1e-5f),
              "pitch joint %d moves the foot sideways by %e per radian",
              pitch_cols[k], (double)dy);
    }
}

int main(void)
{
    printf("kinematics.c host tests\n");
    printf("-----------------------\n");

    test_defaults_are_the_configured_geometry();
    test_straight_leg_hangs_its_own_length();
    test_hip_pitch_swings_the_foot_on_a_circle();
    test_hip_roll_moves_the_foot_sideways_only();
    test_the_foot_is_never_further_than_the_leg_is_long();
    test_bending_the_knee_shortens_the_reach();
    test_the_jacobian_predicts_what_actually_happens();
    test_a_joint_that_cannot_move_the_foot_has_a_zero_column();

    printf("-----------------------\n");
    if (s_fail)
    {
        printf("FAILED (%d)\n", s_fail);
        return 1;
    }
    printf("PASSED (0 failures)\n");
    return 0;
}
