/*
 * Host test for inekf.c - the filter that answers "where is the robot".
 *
 * A filter is easy to test badly. "It runs and produces no NaN" passes while
 * the estimate is quietly wrong, which is the failure that matters: nothing
 * downstream can tell a confident wrong answer from a right one.
 *
 * So these drive it with motion whose answer is known in advance. Hold the
 * robot still and it must not drift. Drop it and it must accelerate downwards
 * at g. Spin it about one axis and it must report exactly that turn. Plant a
 * foot and the drift must come back out. Each of those has an answer that does
 * not depend on how the filter is implemented.
 */

#include "inekf.h"

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

#define DT      0.001f          /* the robot's 1 kHz tick */
#define GRAV    9.81f

static int close_to(inekf_real_t a, inekf_real_t b, inekf_real_t tol)
{
    inekf_real_t d = a - b;
    return (d < 0 ? -d : d) <= tol;
}

static void start(inekf_t *f)
{
    inekf_params_t p;
    inekf_default_params(&p);
    inekf_init(f, &p);
}

/*
 * What an ideal accelerometer reads when the robot is upright and still: it
 * measures the reaction to gravity, so it reports +g upwards, not zero.
 */
static void still_sample(inekf_real_t *omega, inekf_real_t *accel)
{
    omega[0] = omega[1] = omega[2] = 0.0f;
    accel[0] = accel[1] = 0.0f;
    accel[2] = GRAV;
}

static void test_a_robot_standing_still_does_not_wander(void)
{
    printf("held still for a second, the estimate does not run away\n");

    inekf_t f;
    start(&f);

    inekf_real_t w[3], a[3];
    still_sample(w, a);

    for (int i = 0; i < 1000; i++)
    {
        inekf_predict(&f, w, a, DT);
    }

    inekf_real_t v[3];
    inekf_velocity_world(&f, v);

    /*
     * With no correction available the estimate is dead reckoning, so a little
     * drift is expected. What must not happen is it running away: a millimetre
     * per second after a full second is fine, a metre per second is the
     * gravity model being wrong.
     */
    for (int k = 0; k < 3; k++)
    {
        CHECK(fabsf(v[k]) < 0.05f,
              "standing still, velocity component %d drifted to %f m/s",
              k, (double)v[k]);
    }

    CHECK(fabsf(inekf_height(&f)) < 0.05f,
          "standing still, height drifted to %f m", (double)inekf_height(&f));
}

static void test_free_fall_accelerates_downwards_at_g(void)
{
    printf("in free fall the estimate accelerates downwards at g\n");

    /*
     * An accelerometer in free fall reads zero - nothing is pushing back. The
     * filter must therefore conclude the robot is accelerating downwards at g.
     * This is the sharpest check on the sign of gravity, which is easy to get
     * backwards and produces a robot that believes it is flying.
     */
    inekf_t f;
    start(&f);

    inekf_real_t w[3] = { 0.0f, 0.0f, 0.0f };
    inekf_real_t a[3] = { 0.0f, 0.0f, 0.0f };   /* free fall */

    for (int i = 0; i < 500; i++)               /* half a second */
    {
        inekf_predict(&f, w, a, DT);
    }

    inekf_real_t v[3];
    inekf_velocity_world(&f, v);

    const inekf_real_t expect = -GRAV * 0.5f;   /* -4.905 m/s */

    CHECK(close_to(v[2], expect, 0.05f),
          "after half a second of free fall the vertical speed is %f m/s, "
          "expected about %f", (double)v[2], (double)expect);
    CHECK(v[2] < 0.0f, "free fall produced an UPWARD velocity (%f) - the sign "
          "of gravity is inverted", (double)v[2]);
    CHECK(fabsf(v[0]) < 0.05f && fabsf(v[1]) < 0.05f,
          "free fall also moved the robot sideways: (%f, %f)",
          (double)v[0], (double)v[1]);
    CHECK(inekf_height(&f) < 0.0f,
          "the robot fell but its height went up (%f)", (double)inekf_height(&f));
}

static void test_a_steady_spin_is_reported_as_that_spin(void)
{
    printf("spun about one axis, the estimate turns by exactly that much\n");

    inekf_t f;
    start(&f);

    /* A quarter turn about z, taken slowly enough to be unambiguous. */
    const inekf_real_t rate = 1.5707963f;        /* rad/s */
    const int          ticks = 1000;             /* one second -> 90 degrees */

    inekf_real_t w[3] = { 0.0f, 0.0f, rate };
    inekf_real_t a[3];
    still_sample(a, a);                          /* zero omega, then overwrite */
    a[0] = 0.0f; a[1] = 0.0f; a[2] = GRAV;

    for (int i = 0; i < ticks; i++)
    {
        inekf_predict(&f, w, a, DT);
    }

    inekf_real_t q[4];
    inekf_quaternion(&f, q);                     /* w, x, y, z */

    /*
     * A quarter turn about z is w = cos(45 deg), z = sin(45 deg), and the x
     * and y parts stay at zero. The sign of the whole quaternion is arbitrary,
     * so compare magnitudes.
     */
    const inekf_real_t half = 0.70710678f;

    CHECK(close_to(fabsf(q[0]), half, 0.02f),
          "after a quarter turn the scalar part is %f, expected about %f",
          (double)fabsf(q[0]), (double)half);
    CHECK(close_to(fabsf(q[3]), half, 0.02f),
          "after a quarter turn the z part is %f, expected about %f",
          (double)fabsf(q[3]), (double)half);
    CHECK(fabsf(q[1]) < 0.02f && fabsf(q[2]) < 0.02f,
          "turning about z also tilted the robot: x=%f y=%f",
          (double)q[1], (double)q[2]);

    /* However it is signed, a quaternion is a unit vector. */
    inekf_real_t n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    CHECK(close_to(n, 1.0f, 1e-3f), "the quaternion is not unit length: %f",
          (double)n);
}

static void test_the_orientation_stays_a_rotation(void)
{
    printf("after a long irregular motion the orientation is still valid\n");

    /*
     * Integrating rotations repeatedly is where a filter quietly rots: the
     * matrix drifts away from being a rotation and everything derived from it
     * is subtly wrong, with no error reported anywhere. Ten seconds of
     * changing motion, then check the quaternion is still unit length.
     */
    inekf_t f;
    start(&f);

    for (int i = 0; i < 10000; i++)
    {
        inekf_real_t t = (inekf_real_t)i * DT;
        inekf_real_t w[3] = { 0.8f * sinf(t), 0.6f * cosf(0.7f * t), 0.4f };
        inekf_real_t a[3] = { 0.5f * sinf(2.0f * t), 0.3f * cosf(t), GRAV };
        inekf_predict(&f, w, a, DT);
    }

    inekf_real_t q[4];
    inekf_quaternion(&f, q);
    inekf_real_t n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

    CHECK(close_to(n, 1.0f, 1e-2f),
          "after ten seconds the quaternion has length %f, not 1 - the "
          "orientation has stopped being a rotation", (double)n);

    /* And nothing has become NaN, which would poison everything downstream. */
    inekf_real_t v[3];
    inekf_velocity_world(&f, v);
    for (int k = 0; k < 3; k++)
    {
        CHECK(v[k] == v[k], "velocity component %d is NaN", k);
    }
    CHECK(inekf_height(&f) == inekf_height(&f), "height is NaN");
}

static void test_contacts_are_counted_as_they_come_and_go(void)
{
    printf("feet landing and lifting are counted\n");

    inekf_t f;
    start(&f);

    CHECK(inekf_num_contacts(&f) == 0, "a fresh filter already has contacts");

    const inekf_real_t foot[3] = { 0.0f, 0.05f, -0.65f };
    inekf_real_t C[9];
    memset(C, 0, sizeof(C));

    inekf_add_contact(&f, 0, foot, C);
    CHECK(inekf_num_contacts(&f) == 1, "one foot down was not counted");

    inekf_add_contact(&f, 1, foot, C);
    CHECK(inekf_num_contacts(&f) == 2, "two feet down were not counted");

    /* Adding the same slot twice must not count it twice. */
    inekf_add_contact(&f, 1, foot, C);
    CHECK(inekf_num_contacts(&f) == 2,
          "the same foot was counted twice (%d)", inekf_num_contacts(&f));

    inekf_remove_contact(&f, 0);
    CHECK(inekf_num_contacts(&f) == 1, "lifting a foot was not counted");

    /* Removing one that is not down must be harmless. */
    inekf_remove_contact(&f, 0);
    CHECK(inekf_num_contacts(&f) == 1,
          "removing an absent contact changed the count (%d)",
          inekf_num_contacts(&f));

    inekf_remove_contact(&f, 1);
    CHECK(inekf_num_contacts(&f) == 0, "the last foot never lifted");
}

static void test_a_planted_foot_pulls_the_drift_back(void)
{
    printf("a foot on the ground corrects the drift that dead reckoning adds\n");

    /*
     * The whole reason contacts exist. Give the filter a biased accelerometer
     * so its own dead reckoning walks away, then plant a foot and hold it
     * still. The foot is not moving, so the disagreement between where the leg
     * says it is and where the estimate says it is has to be resolved by
     * correcting the estimate - and the drift must come back down.
     */
    inekf_t f;
    start(&f);

    inekf_real_t w[3] = { 0.0f, 0.0f, 0.0f };
    inekf_real_t a[3] = { 0.30f, 0.0f, GRAV };      /* a persistent x bias */

    for (int i = 0; i < 500; i++)
    {
        inekf_predict(&f, w, a, DT);
    }

    inekf_real_t v_before[3];
    inekf_velocity_world(&f, v_before);
    CHECK(fabsf(v_before[0]) > 0.05f,
          "setup: the bias did not produce drift to correct (%f)",
          (double)v_before[0]);

    /* Plant a foot and keep telling the filter it has not moved. */
    const inekf_real_t foot[3] = { 0.0f, 0.05f, -0.65f };
    inekf_real_t C[9];
    memset(C, 0, sizeof(C));

    inekf_add_contact(&f, 0, foot, C);

    for (int i = 0; i < 1000; i++)
    {
        inekf_predict(&f, w, a, DT);
        inekf_update_contact(&f, 0, foot, C);
    }

    inekf_real_t v_after[3];
    inekf_velocity_world(&f, v_after);

    CHECK(fabsf(v_after[0]) < fabsf(v_before[0]),
          "the planted foot did not reduce the drift: %f before, %f after",
          (double)v_before[0], (double)v_after[0]);

    /* And it must not have destroyed the estimate in the process. */
    for (int k = 0; k < 3; k++)
    {
        CHECK(v_after[k] == v_after[k], "velocity %d became NaN after updates", k);
    }
}

/*
 * The contact update's covariance step is computed as rank-3 corrections
 * rather than dense matrix products, because at 27 states the dense form was
 * too slow for four contacts in one tick. The two must give the same numbers,
 * so this repeats the update the textbook way - build H, K, I-KH as full
 * matrices, multiply - and compares every element of P.
 */
#define N_ERR  INEKF_ERR_MAX

static void dense_mul(double *C, const double *A, const double *B, int n, int m, int k)
{
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < k; j++)
        {
            double s = 0.0;
            for (int t = 0; t < m; t++)
            {
                s += A[i * m + t] * B[t * k + j];
            }
            C[i * k + j] = s;
        }
    }
}

static void test_the_fast_update_matches_the_dense_one(void)
{
    printf("the contact update's covariance matches a dense reference\n");

    inekf_t f;
    start(&f);

    /* Build up a full, correlated covariance: move, tilt, and plant two
       contacts so every block of P is populated. */
    inekf_real_t w[3] = { 0.3f, -0.2f, 0.1f };
    inekf_real_t a[3] = { 0.5f, 0.2f, GRAV };
    for (int i = 0; i < 300; i++)
    {
        inekf_predict(&f, w, a, DT);
    }
    const inekf_real_t toe[3]  = {  0.10f, 0.08f, -0.63f };
    const inekf_real_t heel[3] = { -0.10f, 0.08f, -0.63f };
    const inekf_real_t C[9] = { 4e-4f, 1e-5f, 2e-5f,
                                1e-5f, 3e-4f, -1e-5f,
                                2e-5f, -1e-5f, 5e-4f };
    inekf_add_contact(&f, 0, toe, C);
    inekf_add_contact(&f, 1, heel, C);
    for (int i = 0; i < 50; i++)
    {
        inekf_predict(&f, w, a, DT);
    }

    /* The reference, in double precision, from the state before the update. */
    static double P[N_ERR * N_ERR], H[3 * N_ERR], PHt[N_ERR * 3], S[9], Si[9];
    static double K[N_ERR * 3], A[N_ERR * N_ERR], T1[N_ERR * N_ERR], T2[N_ERR * N_ERR];
    static double Ref[N_ERR * N_ERR], N[9], RC[9], KN[N_ERR * 3];

    const int slot = 1;
    const int dr   = INEKF_IDX_D(slot);

    for (int i = 0; i < N_ERR * N_ERR; i++)
    {
        P[i] = (double)f.P[i];
    }
    memset(H, 0, sizeof(H));
    for (int c = 0; c < 3; c++)
    {
        H[c * N_ERR + INEKF_IDX_P + c] =  1.0;
        H[c * N_ERR + dr + c]          = -1.0;
    }

    /* N = R C R^T */
    double Rd[9], Cd[9];
    for (int i = 0; i < 9; i++)
    {
        Rd[i] = (double)f.R[i];
        Cd[i] = (double)C[i];
    }
    dense_mul(RC, Rd, Cd, 3, 3, 3);
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            double s = 0.0;
            for (int t = 0; t < 3; t++)
            {
                s += RC[r * 3 + t] * Rd[c * 3 + t];
            }
            N[r * 3 + c] = s;
        }
    }

    /* PHt = P H^T,  S = H P H^T + N */
    for (int i = 0; i < N_ERR; i++)
    {
        for (int c = 0; c < 3; c++)
        {
            double s = 0.0;
            for (int t = 0; t < N_ERR; t++)
            {
                s += P[i * N_ERR + t] * H[c * N_ERR + t];
            }
            PHt[i * 3 + c] = s;
        }
    }
    dense_mul(S, H, PHt, 3, N_ERR, 3);
    for (int i = 0; i < 9; i++)
    {
        S[i] += N[i];
    }

    double det = S[0] * (S[4] * S[8] - S[5] * S[7]) - S[1] * (S[3] * S[8] - S[5] * S[6]) +
                 S[2] * (S[3] * S[7] - S[4] * S[6]);
    Si[0] =  (S[4] * S[8] - S[5] * S[7]) / det;
    Si[1] = -(S[1] * S[8] - S[2] * S[7]) / det;
    Si[2] =  (S[1] * S[5] - S[2] * S[4]) / det;
    Si[3] = -(S[3] * S[8] - S[5] * S[6]) / det;
    Si[4] =  (S[0] * S[8] - S[2] * S[6]) / det;
    Si[5] = -(S[0] * S[5] - S[2] * S[3]) / det;
    Si[6] =  (S[3] * S[7] - S[4] * S[6]) / det;
    Si[7] = -(S[0] * S[7] - S[1] * S[6]) / det;
    Si[8] =  (S[0] * S[4] - S[1] * S[3]) / det;
    dense_mul(K, PHt, Si, N_ERR, 3, 3);

    /* A = I - K H ;  Ref = A P A^T + K N K^T */
    dense_mul(A, K, H, N_ERR, 3, N_ERR);
    for (int i = 0; i < N_ERR * N_ERR; i++)
    {
        A[i] = -A[i];
    }
    for (int i = 0; i < N_ERR; i++)
    {
        A[i * N_ERR + i] += 1.0;
    }
    dense_mul(T1, A, P, N_ERR, N_ERR, N_ERR);
    for (int i = 0; i < N_ERR; i++)
    {
        for (int j = 0; j < N_ERR; j++)
        {
            double s = 0.0;
            for (int t = 0; t < N_ERR; t++)
            {
                s += T1[i * N_ERR + t] * A[j * N_ERR + t];
            }
            T2[i * N_ERR + j] = s;
        }
    }
    dense_mul(KN, K, N, N_ERR, 3, 3);
    for (int i = 0; i < N_ERR; i++)
    {
        for (int j = 0; j < N_ERR; j++)
        {
            double s = 0.0;
            for (int c = 0; c < 3; c++)
            {
                s += KN[i * 3 + c] * K[j * 3 + c];
            }
            Ref[i * N_ERR + j] = T2[i * N_ERR + j] + s;
        }
    }

    /* The filter's own update, from the same state. */
    const inekf_real_t heel_seen[3] = { -0.095f, 0.083f, -0.628f };
    inekf_update_contact(&f, slot, heel_seen, C);

    double worst = 0.0, scale = 0.0;
    for (int i = 0; i < N_ERR * N_ERR; i++)
    {
        double e = fabs((double)f.P[i] - 0.5 * (Ref[i] + Ref[(i / N_ERR) + (i % N_ERR) * N_ERR]));
        worst = (e > worst) ? e : worst;
        scale = (fabs(Ref[i]) > scale) ? fabs(Ref[i]) : scale;
    }
    printf("  worst element difference %.2e (largest element %.2e)\n", worst, scale);
    CHECK(worst <= 1e-5 * scale,
          "the fast covariance update differs from the dense one by %.3e", worst);
}

static void test_reset_returns_it_to_knowing_nothing(void)
{
    printf("reset clears the estimate but keeps the settings\n");

    inekf_t f;
    start(&f);

    inekf_real_t w[3] = { 0.4f, 0.0f, 0.0f };
    inekf_real_t a[3] = { 1.0f, 0.5f, GRAV };
    for (int i = 0; i < 1000; i++)
    {
        inekf_predict(&f, w, a, DT);
    }

    const inekf_real_t foot[3] = { 0.0f, 0.05f, -0.65f };
    inekf_real_t C[9];
    memset(C, 0, sizeof(C));
    inekf_add_contact(&f, 0, foot, C);

    inekf_reset(&f);

    inekf_real_t v[3], q[4];
    inekf_velocity_world(&f, v);
    inekf_quaternion(&f, q);

    CHECK(close_to(v[0], 0.0f, 1e-4f) && close_to(v[1], 0.0f, 1e-4f) &&
          close_to(v[2], 0.0f, 1e-4f),
          "reset left a velocity behind: (%f, %f, %f)",
          (double)v[0], (double)v[1], (double)v[2]);
    CHECK(close_to(fabsf(q[0]), 1.0f, 1e-4f),
          "reset did not return the robot to upright (scalar part %f)",
          (double)fabsf(q[0]));
    CHECK(inekf_num_contacts(&f) == 0,
          "reset left %d contacts behind", inekf_num_contacts(&f));
}

static void test_an_impossible_time_step_is_refused(void)
{
    printf("a nonsensical time step is ignored rather than acted on\n");

    /*
     * dt comes from a hardware timer, and a glitch there must not be
     * integrated: a huge dt would move the estimate a long way in one go, and
     * a negative one would move it backwards.
     */
    inekf_t f;
    start(&f);

    inekf_real_t w[3] = { 0.0f, 0.0f, 0.0f };
    inekf_real_t a[3] = { 0.0f, 0.0f, GRAV };

    inekf_real_t before[3];
    inekf_velocity_world(&f, before);

    inekf_predict(&f, w, a,  0.0f);
    inekf_predict(&f, w, a, -0.01f);
    inekf_predict(&f, w, a, 100.0f);

    inekf_real_t after[3];
    inekf_velocity_world(&f, after);

    for (int k = 0; k < 3; k++)
    {
        CHECK(close_to(before[k], after[k], 1e-6f),
              "an impossible dt moved velocity component %d from %f to %f",
              k, (double)before[k], (double)after[k]);
    }
}

int main(void)
{
    printf("inekf.c host tests\n");
    printf("------------------\n");

    test_a_robot_standing_still_does_not_wander();
    test_free_fall_accelerates_downwards_at_g();
    test_a_steady_spin_is_reported_as_that_spin();
    test_the_orientation_stays_a_rotation();
    test_contacts_are_counted_as_they_come_and_go();
    test_a_planted_foot_pulls_the_drift_back();
    test_the_fast_update_matches_the_dense_one();
    test_reset_returns_it_to_knowing_nothing();
    test_an_impossible_time_step_is_refused();

    printf("------------------\n");
    if (s_fail)
    {
        printf("FAILED (%d)\n", s_fail);
        return 1;
    }
    printf("PASSED (0 failures)\n");
    return 0;
}
