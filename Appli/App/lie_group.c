#include "lie_group.h"
#include <math.h>
#include <string.h>

/*
 * Below this angle the closed-form series for Gamma0..3 divide by theta^n and
 * lose all precision, so the Taylor expansions are used instead. At 400 Hz a
 * 1e-8 rad step corresponds to ~4e-6 rad/s, far below any real gyro noise, so
 * this branch is taken only when the robot is genuinely still.
 */
#define LG_EPS  1e-8f

/* --------------------------------------------------------------------- */
/*  3x3                                                                    */
/* --------------------------------------------------------------------- */

void lg_mat3_identity(inekf_real_t *M)
{
    memset(M, 0, 9 * sizeof(inekf_real_t));
    M[0] = M[4] = M[8] = 1.0f;
}

void lg_mat3_zero(inekf_real_t *M)
{
    memset(M, 0, 9 * sizeof(inekf_real_t));
}

void lg_mat3_copy(inekf_real_t *dst, const inekf_real_t *src)
{
    memcpy(dst, src, 9 * sizeof(inekf_real_t));
}

void lg_mat3_mul(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            C[i * 3 + j] = A[i * 3 + 0] * B[0 * 3 + j] +
                           A[i * 3 + 1] * B[1 * 3 + j] +
                           A[i * 3 + 2] * B[2 * 3 + j];
        }
    }
}

void lg_mat3_mul_bt(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            C[i * 3 + j] = A[i * 3 + 0] * B[j * 3 + 0] +
                           A[i * 3 + 1] * B[j * 3 + 1] +
                           A[i * 3 + 2] * B[j * 3 + 2];
        }
    }
}

void lg_mat3_mul_at(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            C[i * 3 + j] = A[0 * 3 + i] * B[0 * 3 + j] +
                           A[1 * 3 + i] * B[1 * 3 + j] +
                           A[2 * 3 + i] * B[2 * 3 + j];
        }
    }
}

void lg_mat3_transpose(inekf_real_t *T, const inekf_real_t *M)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            T[i * 3 + j] = M[j * 3 + i];
        }
    }
}

void lg_mat3_scale(inekf_real_t *M, inekf_real_t s)
{
    for (int i = 0; i < 9; i++)
    {
        M[i] *= s;
    }
}

void lg_mat3_add(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B)
{
    for (int i = 0; i < 9; i++)
    {
        C[i] = A[i] + B[i];
    }
}

void lg_mat3_vec(inekf_real_t *y, const inekf_real_t *M, const inekf_real_t *x)
{
    for (int i = 0; i < 3; i++)
    {
        y[i] = M[i * 3 + 0] * x[0] + M[i * 3 + 1] * x[1] + M[i * 3 + 2] * x[2];
    }
}

void lg_mat3_vec_t(inekf_real_t *y, const inekf_real_t *M, const inekf_real_t *x)
{
    for (int i = 0; i < 3; i++)
    {
        y[i] = M[0 * 3 + i] * x[0] + M[1 * 3 + i] * x[1] + M[2 * 3 + i] * x[2];
    }
}

void lg_skew(inekf_real_t *S, const inekf_real_t *v)
{
    S[0] =  0.0f;  S[1] = -v[2];  S[2] =  v[1];
    S[3] =  v[2];  S[4] =  0.0f;  S[5] = -v[0];
    S[6] = -v[1];  S[7] =  v[0];  S[8] =  0.0f;
}

/* --------------------------------------------------------------------- */
/*  SO(3) exponential and integrals - equation (49)                        */
/*                                                                         */
/*  All four have the same shape:  a*I + b*S + c*S^2, differing only in the */
/*  coefficients. Each computes S and S^2 once and blends them.             */
/* --------------------------------------------------------------------- */

static void blend(inekf_real_t *G, const inekf_real_t *S, const inekf_real_t *S2,
                  inekf_real_t a, inekf_real_t b, inekf_real_t c)
{
    for (int i = 0; i < 9; i++)
    {
        G[i] = b * S[i] + c * S2[i];
    }
    G[0] += a;
    G[4] += a;
    G[8] += a;
}

void lg_gamma0(inekf_real_t *G, const inekf_real_t *phi)
{
    inekf_real_t S[9], S2[9];
    inekf_real_t th2 = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
    inekf_real_t th  = sqrtf(th2);

    lg_skew(S, phi);
    lg_mat3_mul(S2, S, S);

    if (th < LG_EPS)
    {
        blend(G, S, S2, 1.0f, 1.0f, 0.5f);
        return;
    }
    blend(G, S, S2, 1.0f, sinf(th) / th, (1.0f - cosf(th)) / th2);
}

void lg_gamma1(inekf_real_t *G, const inekf_real_t *phi)
{
    inekf_real_t S[9], S2[9];
    inekf_real_t th2 = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
    inekf_real_t th  = sqrtf(th2);

    lg_skew(S, phi);
    lg_mat3_mul(S2, S, S);

    if (th < LG_EPS)
    {
        blend(G, S, S2, 1.0f, 0.5f, 1.0f / 6.0f);
        return;
    }
    blend(G, S, S2, 1.0f,
          (1.0f - cosf(th)) / th2,
          (th - sinf(th)) / (th2 * th));
}

void lg_gamma2(inekf_real_t *G, const inekf_real_t *phi)
{
    inekf_real_t S[9], S2[9];
    inekf_real_t th2 = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
    inekf_real_t th  = sqrtf(th2);

    lg_skew(S, phi);
    lg_mat3_mul(S2, S, S);

    if (th < LG_EPS)
    {
        blend(G, S, S2, 0.5f, 1.0f / 6.0f, 1.0f / 24.0f);
        return;
    }
    blend(G, S, S2, 0.5f,
          (th - sinf(th)) / (th2 * th),
          (th2 + 2.0f * cosf(th) - 2.0f) / (2.0f * th2 * th2));
}

void lg_gamma3(inekf_real_t *G, const inekf_real_t *phi)
{
    inekf_real_t S[9], S2[9];
    inekf_real_t th2 = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
    inekf_real_t th  = sqrtf(th2);

    lg_skew(S, phi);
    lg_mat3_mul(S2, S, S);

    if (th < LG_EPS)
    {
        blend(G, S, S2, 1.0f / 6.0f, 1.0f / 24.0f, 1.0f / 120.0f);
        return;
    }
    blend(G, S, S2, 1.0f / 6.0f,
          (th2 * 0.5f - 1.0f + cosf(th)) / (th2 * th2),
          (th - sinf(th) - th2 * th / 6.0f) / (th2 * th2 * th));
}

/* --------------------------------------------------------------------- */
/*  Fixed-stride n x n helpers                                             */
/* --------------------------------------------------------------------- */

void lg_matn_zero(inekf_real_t *M, int n)
{
    (void)n;   /* always clear the whole fixed-size buffer */
    memset(M, 0, INEKF_STRIDE * INEKF_STRIDE * sizeof(inekf_real_t));
}

void lg_matn_identity(inekf_real_t *M, int n)
{
    lg_matn_zero(M, n);
    for (int i = 0; i < n; i++)
    {
        M[IDX(i, i)] = 1.0f;
    }
}

void lg_matn_copy(inekf_real_t *dst, const inekf_real_t *src, int n)
{
    (void)n;
    memcpy(dst, src, INEKF_STRIDE * INEKF_STRIDE * sizeof(inekf_real_t));
}

void lg_matn_mul(inekf_real_t *C, const inekf_real_t *A,
                 const inekf_real_t *B, int n)
{
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            inekf_real_t s = 0.0f;
            for (int k = 0; k < n; k++)
            {
                s += A[IDX(i, k)] * B[IDX(k, j)];
            }
            C[IDX(i, j)] = s;
        }
    }
}

void lg_matn_mul_bt(inekf_real_t *C, const inekf_real_t *A,
                    const inekf_real_t *B, int n)
{
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            inekf_real_t s = 0.0f;
            for (int k = 0; k < n; k++)
            {
                s += A[IDX(i, k)] * B[IDX(j, k)];
            }
            C[IDX(i, j)] = s;
        }
    }
}

void lg_matn_symmetrise(inekf_real_t *M, int n)
{
    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            inekf_real_t m = 0.5f * (M[IDX(i, j)] + M[IDX(j, i)]);
            M[IDX(i, j)] = m;
            M[IDX(j, i)] = m;
        }
    }
}

void lg_matn_set_block3(inekf_real_t *M, int row, int col, const inekf_real_t *B3)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            M[IDX(row + i, col + j)] = B3[i * 3 + j];
        }
    }
}

void lg_matn_set_block3_scaled(inekf_real_t *M, int row, int col,
                               const inekf_real_t *B3, inekf_real_t s)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            M[IDX(row + i, col + j)] = s * B3[i * 3 + j];
        }
    }
}
