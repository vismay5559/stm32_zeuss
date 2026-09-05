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

/*
 * For phi along z the three coefficients can be read straight back out of the
 * matrix: G = a I + b S + c S^2, and with phi = (0,0,th) that gives
 * G[8] = a, G[1] = -b*th, G[0] = a - c*th^2.
 */
static void split_coeffs(void (*fn)(inekf_real_t *, const inekf_real_t *),
                         inekf_real_t th,
                         inekf_real_t *a_out, inekf_real_t *b_out, inekf_real_t *c_out)
{
    const inekf_real_t phi[3] = { 0.0f, 0.0f, th };
    inekf_real_t G[9];
    fn(G, phi);
    *a_out = G[8];
    *b_out = -G[1] / th;
    *c_out = (G[8] - G[0]) / (th * th);
}

static void test_branches_agree_at_the_crossover(void)
{
    printf("the series and closed-form branches agree where they meet\n");

    /*
     * LG_SERIES_MAX is 1.0f: just under takes the series, just over the closed
     * form. Compare the COEFFICIENTS rather than the matrices - the matrices
     * also differ because the rotation itself is 0.002 rad larger on one side,
     * which would swamp the thing being measured.
     */
    struct { const char *name; void (*fn)(inekf_real_t *, const inekf_real_t *); }
    gammas[4] = {
        { "gamma0", lg_gamma0 }, { "gamma1", lg_gamma1 },
        { "gamma2", lg_gamma2 }, { "gamma3", lg_gamma3 },
    };

    for (int i = 0; i < 4; i++)
    {
        inekf_real_t a_lo, b_lo, c_lo, a_hi, b_hi, c_hi;
        split_coeffs(gammas[i].fn, 0.999f, &a_lo, &b_lo, &c_lo);
        split_coeffs(gammas[i].fn, 1.001f, &a_hi, &b_hi, &c_hi);

        CHECK(close_to(b_lo, b_hi, 1e-3f),
              "%s S term steps at the crossover: %f -> %f",
              gammas[i].name, (double)b_lo, (double)b_hi);
        CHECK(close_to(c_lo, c_hi, 1e-3f),
              "%s S^2 term steps at the crossover: %f -> %f",
              gammas[i].name, (double)c_lo, (double)c_hi);
    }
}

static void test_coefficients_match_a_high_precision_reference(void)
{
    printf("the coefficients match a double-precision reference, both sides\n");

    /*
     * Reference values computed in double at two rotations: 0.5 rad exercises
     * the series branch, 1.5 rad the closed form. Both are far enough from
     * zero that the double closed form is itself trustworthy - below about
     * 1e-3 even double cancels, which is why no reference is quoted there.
     *
     * These are the numbers the whole file exists to produce. Before the
     * series was widened, gamma3's c at 0.5 rad came out of the f32 closed
     * form as -0.0035 against a true 0.008284 - wrong sign, wrong size.
     */
    struct { inekf_real_t th; const char *name;
             void (*fn)(inekf_real_t *, const inekf_real_t *);
             inekf_real_t a, b, c; } cases[8] = {
        { 0.5f, "gamma0", lg_gamma0, 1.0f,        0.958851077f, 0.489669752f },
        { 0.5f, "gamma1", lg_gamma1, 1.0f,        0.489669752f, 0.164595691f },
        { 0.5f, "gamma2", lg_gamma2, 0.5f,        0.164595691f, 0.041320990f },
        { 0.5f, "gamma3", lg_gamma3, 1.0f / 6.0f, 0.041320990f, 0.008283902f },
        { 1.5f, "gamma0", lg_gamma0, 1.0f,        0.664996658f, 0.413005688f },
        { 1.5f, "gamma1", lg_gamma1, 1.0f,        0.413005688f, 0.148890374f },
        { 1.5f, "gamma2", lg_gamma2, 0.5f,        0.148890374f, 0.038664139f },
        { 1.5f, "gamma3", lg_gamma3, 1.0f / 6.0f, 0.038664139f, 0.007900574f },
    };

    for (int i = 0; i < 8; i++)
    {
        inekf_real_t a, b, c;
        split_coeffs(cases[i].fn, cases[i].th, &a, &b, &c);

        CHECK(close_to(a, cases[i].a, 1e-5f),
              "%s at %.1f: constant term %f, expected %f",
              cases[i].name, (double)cases[i].th, (double)a, (double)cases[i].a);
        CHECK(close_to(b, cases[i].b, 1e-4f),
              "%s at %.1f: S term %f, expected %f",
              cases[i].name, (double)cases[i].th, (double)b, (double)cases[i].b);
        CHECK(close_to(c, cases[i].c, 1e-4f),
              "%s at %.1f: S^2 term %f, expected %f",
              cases[i].name, (double)cases[i].th, (double)c, (double)cases[i].c);
    }
}

static void test_coefficients_hold_across_the_working_range(void)
{
    printf("the S^2 terms stay right across the turn rates that can be probed\n");

    /*
     * phi is the rotation taken in ONE tick, so at 1 kHz these span roughly
     * 50 to 500 rad/s. This is the range that used to be served by the
     * closed-form branch, where gamma3's S^2 term was out by orders of
     * magnitude.
     *
     * The range does not go lower because it cannot be measured from here,
     * not because it is untested. The S^2 term contributes c * th^2 to the
     * matrix; below about 6e-3 rad that is smaller than one ulp of the
     * constant term, so it cannot be read back out - and by the same token it
     * cannot affect the result either. The reference test above pins the
     * formula itself, which is what carries the correctness down to zero.
     */
    const inekf_real_t ths[4] = { 0.05f, 0.1f, 0.3f, 0.5f };

    struct { const char *name; void (*fn)(inekf_real_t *, const inekf_real_t *);
             inekf_real_t c0; } cases[3] = {
        { "gamma1", lg_gamma1, 1.0f / 6.0f   },
        { "gamma2", lg_gamma2, 1.0f / 24.0f  },
        { "gamma3", lg_gamma3, 1.0f / 120.0f },
    };

    for (int i = 0; i < 4; i++)
    {
        for (int k = 0; k < 3; k++)
        {
            inekf_real_t a, b, c;
            split_coeffs(cases[k].fn, ths[i], &a, &b, &c);

            /* The S^2 term falls away from its zero-rotation value slowly -
               by th^2/(something) - so a few percent at half a radian is
               expected. Orders of magnitude are not. */
            CHECK(c > cases[k].c0 * 0.9f && c < cases[k].c0 * 1.05f,
                  "%s S^2 at th=%g is %f, expected near %f",
                  cases[k].name, (double)ths[i], (double)c, (double)cases[k].c0);
        }
    }
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
    test_branches_agree_at_the_crossover();
    test_coefficients_match_a_high_precision_reference();
    test_coefficients_hold_across_the_working_range();
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
