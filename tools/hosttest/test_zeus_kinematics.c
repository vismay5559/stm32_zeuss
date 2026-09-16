/*
 * Host test for zeus_kinematics.c - toe and heel positions, and their
 * Jacobians, in the IMU frame, from the URDF-generated model.
 *
 * The answers come from somewhere else: Pinocchio, a separate rigid-body
 * library, reading the same zeus.urdf in double precision
 * (zeus_kinematics_ref.h, written by tools/gen_kinematics.py). Agreement
 * means the generator walked the URDF correctly - joint order, the waist
 * walked backwards, signs, frames - and that the float code on the board
 * keeps enough precision for the estimator.
 *
 * The Jacobian is also checked against the C code's own finite differences,
 * so a wrong column is caught even if the reference were regenerated from a
 * bad model.
 */

#include "zeus_kinematics.h"
#include "zeus_kinematics_ref.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_fail;

#define CHECK(cond, ...)                                                 \
    do {                                                                 \
        if (!(cond)) {                                                   \
            s_fail++;                                                    \
            printf("  FAIL  ");                                          \
            printf(__VA_ARGS__);                                         \
            printf("\n");                                                \
        }                                                                \
    } while (0)

/* float32 through ~10 transforms at ~1 m: tens of microns would already be
   a modelling error, not rounding. */
#define TOL_P   2e-5        /* m      */
#define TOL_J   1e-4        /* m/rad  */

static void test_matches_pinocchio(void)
{
    printf("toe and heel, and their Jacobians, match Pinocchio (%d poses)\n", ZEUS_KIN_REF_N);

    double worst_p = 0.0, worst_j = 0.0;

    for (int n = 0; n < ZEUS_KIN_REF_N; n++)
    {
        const zeus_kin_ref_t *r = &zeus_kin_ref[n];
        float q[ZEUS_KIN_NQ];
        float p[ZEUS_KIN_POINTS][3];
        float J[ZEUS_KIN_POINTS][3 * ZEUS_KIN_NQ];

        for (int i = 0; i < ZEUS_KIN_NQ; i++)
        {
            q[i] = (float)r->q[i];
        }
        CHECK(zeus_kin_foot((zeus_kin_side_t)r->side, q, p, J) == 0, "ref %d: call failed", n);

        for (int k = 0; k < 3; k++)
        {
            double e = fabs((double)p[r->point][k] - r->p[k]);
            worst_p = (e > worst_p) ? e : worst_p;
            CHECK(e < TOL_P, "ref %d side %d point %d: p[%d] %.6f, Pinocchio %.6f",
                  n, r->side, r->point, k, (double)p[r->point][k], r->p[k]);
        }
        for (int i = 0; i < 3 * ZEUS_KIN_NQ; i++)
        {
            double e = fabs((double)J[r->point][i] - r->J[i]);
            worst_j = (e > worst_j) ? e : worst_j;
            CHECK(e < TOL_J, "ref %d side %d point %d: J[%d,%d] %.6f, Pinocchio %.6f",
                  n, r->side, r->point, i / ZEUS_KIN_NQ, i % ZEUS_KIN_NQ,
                  (double)J[r->point][i], r->J[i]);
        }
    }
    printf("  worst: p %.2e m, J %.2e m/rad\n", worst_p, worst_j);
}

static void test_jacobian_predicts_small_moves(void)
{
    printf("each Jacobian column predicts what nudging that joint does\n");

    const float q0[ZEUS_KIN_NQ] = { 0.3f, -0.1f, -0.6f, 0.25f, 0.05f, -0.04f, 0.1f, -0.08f };
    const float h = 1e-3f;

    for (int side = 0; side < 2; side++)
    {
        float p0[ZEUS_KIN_POINTS][3], J[ZEUS_KIN_POINTS][3 * ZEUS_KIN_NQ];
        zeus_kin_foot((zeus_kin_side_t)side, q0, p0, J);

        for (int i = 0; i < ZEUS_KIN_NQ; i++)
        {
            float qp[ZEUS_KIN_NQ], qm[ZEUS_KIN_NQ];
            float pp[ZEUS_KIN_POINTS][3], pm[ZEUS_KIN_POINTS][3];
            memcpy(qp, q0, sizeof qp);
            memcpy(qm, q0, sizeof qm);
            qp[i] += h;
            qm[i] -= h;
            zeus_kin_foot((zeus_kin_side_t)side, qp, pp, NULL);
            zeus_kin_foot((zeus_kin_side_t)side, qm, pm, NULL);

            for (int k = 0; k < ZEUS_KIN_POINTS; k++)
            {
                for (int r = 0; r < 3; r++)
                {
                    float fd = (pp[k][r] - pm[k][r]) / (2.0f * h);
                    CHECK(fabsf(fd - J[k][r * ZEUS_KIN_NQ + i]) < 2e-3f,
                          "side %d point %d: dp[%d]/dq[%d] finite diff %.5f, J %.5f",
                          side, k, r, i, (double)fd, (double)J[k][r * ZEUS_KIN_NQ + i]);
                }
            }
        }
    }
}

static void test_the_robot_is_the_right_way_up(void)
{
    printf("at q = 0: feet below the IMU, toe ahead of heel, left foot left\n");

    const float q[ZEUS_KIN_NQ] = { 0 };
    float L[ZEUS_KIN_POINTS][3], R[ZEUS_KIN_POINTS][3];
    zeus_kin_foot(ZEUS_KIN_LEFT, q, L, NULL);
    zeus_kin_foot(ZEUS_KIN_RIGHT, q, R, NULL);

    CHECK(L[ZEUS_KIN_TOE][2] < -0.3f && R[ZEUS_KIN_TOE][2] < -0.3f, "feet not below the IMU");
    CHECK(L[ZEUS_KIN_TOE][0] > L[ZEUS_KIN_HEEL][0], "left toe behind its heel");
    CHECK(R[ZEUS_KIN_TOE][0] > R[ZEUS_KIN_HEEL][0], "right toe behind its heel");
    CHECK(L[ZEUS_KIN_TOE][1] > R[ZEUS_KIN_TOE][1], "left foot is not to the left");

    /* Positive hip pitch swings the foot back, on both legs: the URDF clean-up
       made every pitch axis +Y, so the two legs must agree. */
    float qp[ZEUS_KIN_NQ] = { 0 };
    float Lp[ZEUS_KIN_POINTS][3], Rp[ZEUS_KIN_POINTS][3];
    qp[ZEUS_KIN_Q_HIP_PITCH] = 0.2f;
    zeus_kin_foot(ZEUS_KIN_LEFT, qp, Lp, NULL);
    zeus_kin_foot(ZEUS_KIN_RIGHT, qp, Rp, NULL);
    CHECK(Lp[ZEUS_KIN_TOE][0] < L[ZEUS_KIN_TOE][0] && Rp[ZEUS_KIN_TOE][0] < R[ZEUS_KIN_TOE][0],
          "positive hip pitch did not swing both feet back");

    /* The spring adds to the motor angle: same joint axis. */
    float qs[ZEUS_KIN_NQ] = { 0 };
    float Ls[ZEUS_KIN_POINTS][3];
    qs[ZEUS_KIN_Q_HIP_PITCH] = 0.15f;
    qs[ZEUS_KIN_Q_HIP_PITCH_SPRING] = 0.05f;
    zeus_kin_foot(ZEUS_KIN_LEFT, qs, Ls, NULL);
    CHECK(fabsf(Ls[ZEUS_KIN_TOE][0] - Lp[ZEUS_KIN_TOE][0]) < 1e-3f &&
          fabsf(Ls[ZEUS_KIN_TOE][2] - Lp[ZEUS_KIN_TOE][2]) < 1e-3f,
          "0.15 motor + 0.05 spring is not about 0.2 of hip pitch");
}

static void test_bad_side(void)
{
    printf("a side that is neither leaves the outputs alone\n");

    const float q[ZEUS_KIN_NQ] = { 0 };
    float p[ZEUS_KIN_POINTS][3] = { { 7.0f, 7.0f, 7.0f }, { 7.0f, 7.0f, 7.0f } };
    CHECK(zeus_kin_foot((zeus_kin_side_t)2, q, p, NULL) == -1, "side 2 accepted");
    CHECK(p[0][0] == 7.0f && p[1][2] == 7.0f, "p written for a bad side");
    CHECK(strlen(zeus_kin_model_sha) == 16u, "model sha missing");
}

int main(void)
{
    printf("zeus_kinematics.c host tests (model %s)\n-----------------------\n", zeus_kin_model_sha);

    test_matches_pinocchio();
    test_jacobian_predicts_small_moves();
    test_the_robot_is_the_right_way_up();
    test_bad_side();

    printf("-----------------------\n%s (%d failure%s)\n",
           s_fail ? "FAILED" : "PASSED", s_fail, s_fail == 1 ? "" : "s");
    return s_fail ? 1 : 0;
}
