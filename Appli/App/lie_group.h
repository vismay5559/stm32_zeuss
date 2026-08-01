#ifndef LIE_GROUP_H
#define LIE_GROUP_H

#include <stdint.h>

/*
 * SE_{N+2}(3) Lie group mathematics for the contact-aided InEKF.
 *
 * Port of zeus_sensor_fusion/lie_group.py to fixed-size C. Notation follows
 * Hartley et al. 2019, "Contact-Aided Invariant Extended Kalman Filtering for
 * Robot State Estimation" - equation numbers in the comments refer to it.
 *
 * The state matrix for the biped, with both feet in contact, is 7x7:
 *
 *     X = [ R   v   p   d_L  d_R ]
 *         [ 0   1   0    0    0  ]
 *         [ 0   0   1    0    0  ]
 *         [ 0   0   0    1    0  ]
 *         [ 0   0   0    0    1  ]
 *
 *   R = body->world rotation, v = world velocity, p = world position,
 *   d_k = world position of contact k.
 *
 * ---------------------------------------------------------------------------
 * WHY FIXED SIZE
 *
 * The Python version resizes X and P with numpy whenever a foot lands or
 * lifts. There is no allocator here and none is wanted in a 1 kHz control
 * loop, so every matrix is declared at its maximum size (2 contacts) and an
 * "active" count says how much of it is in use. Rows and columns belonging to
 * inactive contacts are zeroed, which makes them inert in every product.
 *
 * WHY float RATHER THAN double
 *
 * The M7 has a double-precision FPU, but singles are roughly twice as fast and
 * halve the memory traffic - and covariance propagation is the hot path. The
 * filter uses Joseph-form updates and explicit symmetrisation to stay stable in
 * single precision. If numerical trouble ever appears, changing inekf_real_t
 * to double is the single switch that fixes it.
 * ---------------------------------------------------------------------------
 */

typedef float inekf_real_t;

/* Two feet. */
#define INEKF_MAX_CONTACTS   2

/* Side length of the group matrix X: N + 5. */
#define INEKF_X_DIM(n)       ((n) + 5)
#define INEKF_X_MAX          INEKF_X_DIM(INEKF_MAX_CONTACTS)          /* 7  */

/*
 * Error-state dimension: 3(N+3) pose/contact terms + 6 bias terms.
 * Layout: [ phi(3) | dv(3) | dp(3) | dd_0(3) .. dd_{N-1}(3) | bg(3) | ba(3) ]
 */
#define INEKF_POSE_DIM(n)    (3 * ((n) + 3))
#define INEKF_ERR_DIM(n)     (INEKF_POSE_DIM(n) + 6)
#define INEKF_ERR_MAX        INEKF_ERR_DIM(INEKF_MAX_CONTACTS)        /* 21 */

/*
 * Every error-state matrix is stored row-major with a FIXED stride of
 * INEKF_ERR_MAX, whatever N happens to be. Keeping the stride constant means
 * indexing never changes when a foot lands or lifts - only the loop bounds do.
 */
#define INEKF_STRIDE         INEKF_ERR_MAX
#define IDX(r, c)            ((r) * INEKF_STRIDE + (c))

/* --------------------------------------------------------------------- */
/*  3x3 helpers                                                           */
/* --------------------------------------------------------------------- */

void lg_mat3_identity(inekf_real_t *M);
void lg_mat3_zero(inekf_real_t *M);
void lg_mat3_copy(inekf_real_t *dst, const inekf_real_t *src);

/* C = A * B */
void lg_mat3_mul(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B);
/* C = A * B^T */
void lg_mat3_mul_bt(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B);
/* C = A^T * B */
void lg_mat3_mul_at(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B);

void lg_mat3_transpose(inekf_real_t *T, const inekf_real_t *M);
void lg_mat3_scale(inekf_real_t *M, inekf_real_t s);
void lg_mat3_add(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B);

/* y = M * x */
void lg_mat3_vec(inekf_real_t *y, const inekf_real_t *M, const inekf_real_t *x);
/* y = M^T * x */
void lg_mat3_vec_t(inekf_real_t *y, const inekf_real_t *M, const inekf_real_t *x);

/* 3-vector -> 3x3 skew-symmetric matrix (v)_x */
void lg_skew(inekf_real_t *S, const inekf_real_t *v);

/* --------------------------------------------------------------------- */
/*  SO(3) exponential and its integrals - equation (49)                   */
/* --------------------------------------------------------------------- */

/* Gamma0(phi) = exp(phi^) : the rotation itself. */
void lg_gamma0(inekf_real_t *G, const inekf_real_t *phi);
/* Gamma1(phi) : left Jacobian of SO(3). */
void lg_gamma1(inekf_real_t *G, const inekf_real_t *phi);
/* Gamma2(phi) : double integral of the exponential. */
void lg_gamma2(inekf_real_t *G, const inekf_real_t *phi);
/* Gamma3(phi) : triple integral, needed by the Psi2 bias term. */
void lg_gamma3(inekf_real_t *G, const inekf_real_t *phi);

/* --------------------------------------------------------------------- */
/*  Generic fixed-stride matrix helpers (error-state sized)               */
/* --------------------------------------------------------------------- */

void lg_matn_zero(inekf_real_t *M, int n);
void lg_matn_identity(inekf_real_t *M, int n);
void lg_matn_copy(inekf_real_t *dst, const inekf_real_t *src, int n);

/* C = A * B, all n x n, stride INEKF_STRIDE. C must not alias A or B. */
void lg_matn_mul(inekf_real_t *C, const inekf_real_t *A,
                 const inekf_real_t *B, int n);
/* C = A * B^T */
void lg_matn_mul_bt(inekf_real_t *C, const inekf_real_t *A,
                    const inekf_real_t *B, int n);

/* M = 0.5 (M + M^T). Covariances drift out of symmetry through rounding;
   forcing it back every step is cheap and prevents slow divergence. */
void lg_matn_symmetrise(inekf_real_t *M, int n);

/* Write a 3x3 block into an n x n matrix at (row, col). */
void lg_matn_set_block3(inekf_real_t *M, int row, int col,
                        const inekf_real_t *B3);
/* Same, but scaled: M[row..][col..] = s * B3 */
void lg_matn_set_block3_scaled(inekf_real_t *M, int row, int col,
                               const inekf_real_t *B3, inekf_real_t s);

#endif /* LIE_GROUP_H */
