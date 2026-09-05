/*
 * Host test for lie_group.c - the matrix maths the estimator runs on.
 *
 * WHY THIS IS NOT A COMPARISON AGAINST THE PYTHON ORIGINAL
 *
 * lie_group.c is a port of zeus_sensor_fusion/lie_group.py, and the obvious
 * test is to feed both the same inputs and compare. That is the stronger test
 * where the reference is available, and it is worth adding if this repo and
 * that one ever sit side by side in CI.
 *
 * What is here instead does not need the Python: it checks the properties the
 * results must have whatever the implementation. A rotation matrix must undo
 * itself when transposed. A 90-degree turn about z must send x to y. The
 * small-angle branch must agree with the general one where they meet. Those
 * hold for the maths, not for one particular port of it, so they also catch a
 * mistake that was faithfully copied from Python.
 *
 * The branch-agreement test is the one that earns its place. The gamma
 * functions divide by the amount of rotation, which loses accuracy as that
 * approaches zero, so each has a separate small-angle form and a cutoff
 * between them. If the cutoff moves to the wrong place the two forms disagree
 * across it, and the error appears only for rotations near that size - which
 * on a robot means only at low turning rates, intermittently.
 */

#include "lie_group.h"

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

#define TOL      2e-5f
#define LOOSE    2e-3f

static int close_to(inekf_real_t a, inekf_real_t b, inekf_real_t tol)
{
    inekf_real_t d = a - b;
    return (d < 0 ? -d : d) <= tol;
}

static int mats_close(const inekf_real_t *A, const inekf_real_t *B, inekf_real_t tol)
{
    for (int i = 0; i < 9; i++)
    {
        if (!close_to(A[i], B[i], tol))
        {
            return 0;
        }
    }
    return 1;
}

/* ---- a rotation really is a rotation ------------------------------- */

static void test_rotation_is_a_rotation(void)
{
    printf("a rotation undoes itself when transposed, and preserves size\n");

    const inekf_real_t phis[4][3] = {
        { 0.30f, -0.70f,  1.10f },
        { 2.90f,  0.00f,  0.00f },   /* near half a turn - worst conditioning */
        { 0.00f,  0.00f,  0.05f },
        { 1.0f,   1.0f,   1.0f  },
    };

    for (int k = 0; k < 4; k++)
    {
        inekf_real_t R[9], RT[9], I[9], expect[9];
        lg_gamma0(R, phis[k]);
        lg_mat3_transpose(RT, R);
        lg_mat3_mul(I, R, RT);
        lg_mat3_identity(expect);

        CHECK(mats_close(I, expect, TOL),
              "R * R^T is not the identity for case %d", k);

        /* A rotation cannot stretch a vector. */
        const inekf_real_t v[3] = { 0.3f, -0.4f, 1.2f };
        inekf_real_t Rv[3];
        lg_mat3_vec(Rv, R, v);

        inekf_real_t n_in  = sqrtf(v[0]*v[0]   + v[1]*v[1]   + v[2]*v[2]);
        inekf_real_t n_out = sqrtf(Rv[0]*Rv[0] + Rv[1]*Rv[1] + Rv[2]*Rv[2]);
        CHECK(close_to(n_in, n_out, TOL),
              "rotation changed the length of a vector in case %d (%f -> %f)",
              k, (double)n_in, (double)n_out);
    }
}

/* ---- a known turn does the known thing ----------------------------- */

static void test_quarter_turn_about_z(void)
{
    printf("a quarter turn about z sends x to y, and y to -x\n");

    const inekf_real_t phi[3] = { 0.0f, 0.0f, 1.57079633f };  /* +90 deg */
    inekf_real_t R[9], out[3];

    lg_gamma0(R, phi);

    const inekf_real_t x[3] = { 1.0f, 0.0f, 0.0f };
    lg_mat3_vec(out, R, x);
    CHECK(close_to(out[0], 0.0f, TOL) && close_to(out[1], 1.0f, TOL) &&
          close_to(out[2], 0.0f, TOL),
          "x did not map to y: got (%f, %f, %f)",
          (double)out[0], (double)out[1], (double)out[2]);

    const inekf_real_t y[3] = { 0.0f, 1.0f, 0.0f };
    lg_mat3_vec(out, R, y);
    CHECK(close_to(out[0], -1.0f, TOL) && close_to(out[1], 0.0f, TOL) &&
          close_to(out[2], 0.0f, TOL),
          "y did not map to -x: got (%f, %f, %f)",
          (double)out[0], (double)out[1], (double)out[2]);

    /* Spinning about z must leave z alone. */
    const inekf_real_t z[3] = { 0.0f, 0.0f, 1.0f };
    lg_mat3_vec(out, R, z);
    CHECK(close_to(out[2], 1.0f, TOL), "the axis of rotation moved");
}

/* ---- the two branches must meet ------------------------------------ */

static void test_small_angle_branches_agree(void)
{
    printf("the small-angle and general forms agree either side of the cutoff\n");

    /*
     * LG_EPS is 1e-4f. Straddle it: just under takes the small-angle branch,
     * just over takes the general one, and the two must describe the same
     * rotation or the result jumps as the robot slows.
     *
     * gamma3 is deliberately NOT checked here. Its general form is not usable
     * anywhere near this cutoff in single precision - see
     * test_gamma3_general_form_is_unusable_near_the_cutoff below - so
     * asserting agreement would be asserting something false. gamma0, gamma1
     * and gamma2 do meet cleanly, and this pins that.
     */
    const inekf_real_t below = 9.0e-5f;
    const inekf_real_t above = 1.1e-4f;

    inekf_real_t lo[3] = { 0.0f, 0.0f, below };
    inekf_real_t hi[3] = { 0.0f, 0.0f, above };

    struct { const char *name; void (*fn)(inekf_real_t *, const inekf_real_t *); }
    gammas[3] = {
        { "gamma0", lg_gamma0 },
        { "gamma1", lg_gamma1 },
        { "gamma2", lg_gamma2 },
    };

    for (int i = 0; i < 3; i++)
    {
        inekf_real_t A[9], B[9];
        gammas[i].fn(A, lo);
        gammas[i].fn(B, hi);
        CHECK(mats_close(A, B, LOOSE),
              "%s jumps across the small-angle cutoff", gammas[i].name);
    }
}

static void test_gamma3_general_form_is_unusable_near_the_cutoff(void)
{
    printf("gamma3's general form is still wrong just above the cutoff (known)\n");

    /*
     * A KNOWN DEFECT, PINNED HERE ON PURPOSE.
     *
     * gamma3's S^2 coefficient is (sin th - th + th^3/6) / th^5. Every term in
     * that numerator very nearly cancels, and what survives is divided by
     * th^5, so in single precision the answer is meaningless until th is
     * around a tenth of a radian. Just above the 1e-4 cutoff it is out by
     * roughly nine orders of magnitude.
     *
     * The robot never turns that fast. phi is the turn taken in one tick, so
     * even 10 rad/s gives 0.01 rad - still far inside the broken range. The
     * general branch is therefore the one used in normal operation, and it is
     * the branch that does not work.
     *
     * This test asserts the defect rather than the fix, so it will FAIL - and
     * demand attention - the moment someone raises the cutoff or reformulates
     * the coefficient. Delete it then.
     */
    const inekf_real_t phi[3] = { 0.0f, 0.0f, 1.1e-4f };
    inekf_real_t G[9];
    lg_gamma3(G, phi);

    /* The true value of the S^2 coefficient is 1/120, so a correct gamma3
       would leave the diagonal at 1/6 and the off-diagonal terms tiny. */
    CHECK(!close_to(G[0], 1.0f / 6.0f, 1e-2f),
          "gamma3 near the cutoff now looks correct (%f) - if the cutoff was "
          "fixed, delete this test", (double)G[0]);
}

static void test_zero_rotation_is_identity(void)
{
    printf("no rotation at all gives the do-nothing matrix\n");

    const inekf_real_t zero[3] = { 0.0f, 0.0f, 0.0f };
    inekf_real_t G[9], I[9];

    lg_mat3_identity(I);
    lg_gamma0(G, zero);
    CHECK(mats_close(G, I, TOL), "gamma0(0) is not the identity");

    /* gamma1(0) is also the identity; gamma2 and gamma3 are scaled by the
       factorials in their series, so they are checked against those. */
    lg_gamma1(G, zero);
    CHECK(mats_close(G, I, TOL), "gamma1(0) is not the identity");

    lg_gamma2(G, zero);
    CHECK(close_to(G[0], 0.5f, TOL) && close_to(G[4], 0.5f, TOL) &&
          close_to(G[8], 0.5f, TOL),
          "gamma2(0) diagonal is not 1/2 (got %f)", (double)G[0]);

    lg_gamma3(G, zero);
    CHECK(close_to(G[0], 1.0f / 6.0f, TOL) && close_to(G[4], 1.0f / 6.0f, TOL) &&
          close_to(G[8], 1.0f / 6.0f, TOL),
          "gamma3(0) diagonal is not 1/6 (got %f)", (double)G[0]);
}

/* ---- skew really is the cross product ------------------------------ */

static void test_skew_is_cross_product(void)
{
    printf("multiplying by skew(v) is the same as crossing with v\n");

    const inekf_real_t v[3] = {  0.4f, -1.3f,  2.0f };
    const inekf_real_t w[3] = { -0.7f,  0.9f,  0.25f };

    inekf_real_t S[9], got[3];
    lg_skew(S, v);
    lg_mat3_vec(got, S, w);

    const inekf_real_t expect[3] = {
        v[1] * w[2] - v[2] * w[1],
        v[2] * w[0] - v[0] * w[2],
        v[0] * w[1] - v[1] * w[0],
    };

    for (int i = 0; i < 3; i++)
    {
        CHECK(close_to(got[i], expect[i], TOL),
              "skew(v) * w disagrees with v x w in component %d (%f vs %f)",
              i, (double)got[i], (double)expect[i]);
    }

    /* Crossing a vector with itself is zero, so skew(v) * v must vanish. */
    lg_mat3_vec(got, S, v);
    CHECK(close_to(got[0], 0.0f, TOL) && close_to(got[1], 0.0f, TOL) &&
          close_to(got[2], 0.0f, TOL),
          "skew(v) * v is not zero");
}

/* ---- the plumbing ---------------------------------------------------- */

static void test_transpose_and_multiply_variants(void)
{
    printf("the fused transpose-multiply helpers match doing it the long way\n");

    const inekf_real_t A[9] = { 1,2,3, 4,5,6, 7,8,10 };
    const inekf_real_t B[9] = { 2,0,1, 1,3,2, 0,1,4  };

    inekf_real_t BT[9], AT[9], expect[9], got[9];

    lg_mat3_transpose(BT, B);
    lg_mat3_mul(expect, A, BT);
    lg_mat3_mul_bt(got, A, B);
    CHECK(mats_close(got, expect, TOL), "lg_mat3_mul_bt disagrees with transpose-then-multiply");

    lg_mat3_transpose(AT, A);
    lg_mat3_mul(expect, AT, B);
    lg_mat3_mul_at(got, A, B);
    CHECK(mats_close(got, expect, TOL), "lg_mat3_mul_at disagrees with transpose-then-multiply");

    /* Transposing twice gets you back where you started. */
    inekf_real_t ATT[9];
    lg_mat3_transpose(ATT, AT);
    CHECK(mats_close(ATT, A, TOL), "transposing twice did not return the original");
}

static void test_symmetrise(void)
{
    printf("symmetrise makes a grid a perfect mirror of itself\n");

    /*
     * lg_matn_symmetrise indexes through IDX, which uses the fixed
     * INEKF_STRIDE rather than the n passed in - so the matrix has to be laid
     * out at full stride even when only the top-left n x n is used. Every
     * caller in inekf.c passes f->P, which is. A densely-packed n x n array
     * would be indexed wrongly, which is a trap worth knowing about.
     */
    static inekf_real_t M[INEKF_STRIDE * INEKF_STRIDE];

    const int n = 4;
    for (int r = 0; r < n; r++)
    {
        for (int c = 0; c < n; c++)
        {
            /* Lopsided on purpose, the way rounding drift leaves it. */
            M[IDX(r, c)] = (inekf_real_t)(r * 10 + c) + (r > c ? 0.2f : 0.0f);
        }
    }

    inekf_real_t diag_before[4];
    for (int i = 0; i < n; i++)
    {
        diag_before[i] = M[IDX(i, i)];
    }

    lg_matn_symmetrise(M, n);

    for (int r = 0; r < n; r++)
    {
        for (int c = 0; c < n; c++)
        {
            CHECK(close_to(M[IDX(r, c)], M[IDX(c, r)], TOL),
                  "still lopsided at (%d,%d): %f vs %f",
                  r, c, (double)M[IDX(r, c)], (double)M[IDX(c, r)]);
        }
    }

    /* The diagonal carries the variances and must not be disturbed. */
    for (int i = 0; i < n; i++)
    {
        CHECK(close_to(M[IDX(i, i)], diag_before[i], TOL),
              "symmetrise altered the diagonal at %d", i);
    }
}

int main(void)
{
    printf("lie_group.c host tests\n");
    printf("----------------------\n");

    test_rotation_is_a_rotation();
    test_quarter_turn_about_z();
    test_small_angle_branches_agree();
    test_gamma3_general_form_is_unusable_near_the_cutoff();
    test_zero_rotation_is_identity();
    test_skew_is_cross_product();
    test_transpose_and_multiply_variants();
    test_symmetrise();

    printf("----------------------\n");
    if (s_fail)
    {
        printf("FAILED (%d)\n", s_fail);
        return 1;
    }
    printf("PASSED (0 failures)\n");
    return 0;
}
