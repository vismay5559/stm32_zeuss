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

/*
 * WHAT THIS FILE IS, FOR A NON-SPECIALIST
 *
 * The position estimator does a lot of arithmetic on small grids of numbers.
 * A grid of three-by-three numbers is how a rotation is written down - "the
 * robot is tilted like this" - and multiplying two of them together is how
 * you say "turn by this, then turn by that".
 *
 * There is no clever thinking in this file. It is the arithmetic itself:
 * multiply these grids, add those, copy this one. It is written out by hand,
 * for fixed sizes, because the board has no maths library and the whole lot
 * has to finish inside one thousandth of a second.
 *
 * Nothing here knows anything about robots. Names starting lg_mat3_ work on
 * three-by-three grids, lg_matn_ on square grids of any size.
 *
 * Every grid is stored as one flat run of numbers, row by row: the first
 * three are the top row, the next three the middle row, and so on.
 */

/* Set a 3x3 grid to the "do nothing" rotation - the one that leaves anything
   it is applied to unchanged. */
void lg_mat3_identity(inekf_real_t *M);
/* Set every number in a 3x3 grid to zero. */
void lg_mat3_zero(inekf_real_t *M);
/* Copy one 3x3 grid into another. */
void lg_mat3_copy(inekf_real_t *dst, const inekf_real_t *src);

/* C = A * B */
/* C = A x B. For rotations this means "do B, then do A" - the order matters,
   and swapping it gives a different answer. */
void lg_mat3_mul(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B);
/* C = A * B^T */
/* C = A x (B flipped along its diagonal). Kept as its own function because
   flipping B first, then multiplying, would need a scratch grid and twice the
   work for no benefit. */
void lg_mat3_mul_bt(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B);
/* C = A^T * B */
/* C = (A flipped along its diagonal) x B. Same reasoning as above. */
void lg_mat3_mul_at(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B);

/* Flip a grid along its diagonal, so rows become columns. For a rotation this
   happens to be the same as undoing it. */
void lg_mat3_transpose(inekf_real_t *T, const inekf_real_t *M);
/* Multiply every number in the grid by the same amount. */
void lg_mat3_scale(inekf_real_t *M, inekf_real_t s);
/* C = A + B, adding the grids number by number. */
void lg_mat3_add(inekf_real_t *C, const inekf_real_t *A, const inekf_real_t *B);

/* y = M * x */
/* Apply a grid to a single point or direction: y = M x. If M is a rotation,
   this is what actually rotates x. */
void lg_mat3_vec(inekf_real_t *y, const inekf_real_t *M, const inekf_real_t *x);
/* y = M^T * x */
/* Apply the flipped grid to a point: y = (M flipped) x. For a rotation, this
   rotates the opposite way. */
void lg_mat3_vec_t(inekf_real_t *y, const inekf_real_t *M, const inekf_real_t *x);

/* 3-vector -> 3x3 skew-symmetric matrix (v)_x */
/*
 * Build the 3x3 grid that stands in for "cross product with v".
 *
 * This is a bookkeeping trick, not an idea: some operations are easier to
 * write as multiplying by a grid than as a special case, so the three numbers
 * of v get arranged into a grid that has the same effect.
 */
void lg_skew(inekf_real_t *S, const inekf_real_t *v);

/* --------------------------------------------------------------------- */
/*  SO(3) exponential and its integrals - equation (49)                   */
/* --------------------------------------------------------------------- */

/* Gamma0(phi) = exp(phi^) : the rotation itself. */
/*
 * THE FOUR GAMMA FUNCTIONS
 *
 * `phi` describes a rotation as an axis to spin about and how far to spin.
 * These four turn that description into the grids the estimator needs.
 *
 * gamma0 is the rotation itself. The other three are what you get by
 * integrating it once, twice and three times - which is how a constant
 * turning rate becomes a change in orientation, then in velocity, then in
 * position over one interval.
 *
 * Each is computed by a formula for ordinary rotations and by a simpler
 * approximation for very small ones, because the general formula divides by
 * the amount of spin and loses accuracy as that approaches zero.
 */

/* The rotation described by phi. */
void lg_gamma0(inekf_real_t *G, const inekf_real_t *phi);
/* Gamma1(phi) : left Jacobian of SO(3). */
/* The rotation integrated once. */
void lg_gamma1(inekf_real_t *G, const inekf_real_t *phi);
/* Gamma2(phi) : double integral of the exponential. */
/* Integrated twice. */
void lg_gamma2(inekf_real_t *G, const inekf_real_t *phi);
/* Gamma3(phi) : triple integral, needed by the Psi2 bias term. */
/* Integrated three times. This one has the least accuracy to spare, which is
   why the small-rotation cutoff is set where it is. */
void lg_gamma3(inekf_real_t *G, const inekf_real_t *phi);

/* --------------------------------------------------------------------- */
/*  Generic fixed-stride matrix helpers (error-state sized)               */
/* --------------------------------------------------------------------- */

/* Set every number in an n-by-n grid to zero. */
void lg_matn_zero(inekf_real_t *M, int n);
/* Set an n-by-n grid to the "do nothing" one. */
void lg_matn_identity(inekf_real_t *M, int n);
/* Copy one n-by-n grid into another. */
void lg_matn_copy(inekf_real_t *dst, const inekf_real_t *src, int n);

/* C = A * B, all n x n, stride INEKF_STRIDE. C must not alias A or B. */
/* C = A x B, for n-by-n grids. */
void lg_matn_mul(inekf_real_t *C, const inekf_real_t *A,
                 const inekf_real_t *B, int n);
/* C = A * B^T */
/* C = A x (B flipped), for n-by-n grids. */
void lg_matn_mul_bt(inekf_real_t *C, const inekf_real_t *A,
                    const inekf_real_t *B, int n);

/* M = 0.5 (M + M^T). Covariances drift out of symmetry through rounding;
   forcing it back every step is cheap and prevents slow divergence. */
/*
 * Force a grid to be a perfect mirror of itself across its diagonal.
 *
 * The estimator's record of how unsure it is should always be symmetric.
 * Rounding errors nudge it slightly out of shape, and left alone that drift
 * eventually makes the estimator claim impossible things, such as negative
 * uncertainty. This tidies it back after each update.
 */
void lg_matn_symmetrise(inekf_real_t *M, int n);

/* Write a 3x3 block into an n x n matrix at (row, col). */
/* Paste a small 3x3 grid into a larger grid at a given position. The larger
   grids are built out of 3x3 pieces, so this places one of them. */
void lg_matn_set_block3(inekf_real_t *M, int row, int col,
                        const inekf_real_t *B3);
/* Same, but scaled: M[row..][col..] = s * B3 */
/* The same, multiplying the small grid by an amount as it is pasted in. */
void lg_matn_set_block3_scaled(inekf_real_t *M, int row, int col,
                               const inekf_real_t *B3, inekf_real_t s);

#endif /* LIE_GROUP_H */
