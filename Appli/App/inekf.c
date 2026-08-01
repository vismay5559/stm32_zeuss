#include "inekf.h"
#include <math.h>
#include <string.h>

/* Contacts always occupy their own fixed slot, so N is a constant for sizing
   purposes and only the "active" flags change. Using the maximum error-state
   dimension throughout keeps every index stable; inactive contact blocks are
   held at zero and contribute nothing. */
#define NDIM   INEKF_ERR_MAX

/* --------------------------------------------------------------------- */

void inekf_default_params(inekf_params_t *p)
{
    p->noise_gyro        = 0.002f;
    p->noise_accel       = 0.04f;
    p->noise_gyro_bias   = 0.001f;
    p->noise_accel_bias  = 0.001f;
    p->noise_contact_vel = 0.05f;
    p->noise_encoder     = 0.0175f;

    p->init_orientation  = 0.5236f;   /* 30 deg */
    p->init_velocity     = 1.0f;
    p->init_position     = 0.1f;
    p->init_contact      = 0.1f;
    p->init_gyro_bias    = 0.005f;
    p->init_accel_bias   = 0.05f;

    p->gravity[0] = 0.0f;
    p->gravity[1] = 0.0f;
    p->gravity[2] = -9.81f;
}

void inekf_reset(inekf_t *f)
{
    lg_mat3_identity(f->R);
    memset(f->v, 0, sizeof(f->v));
    memset(f->p, 0, sizeof(f->p));
    memset(f->d, 0, sizeof(f->d));
    memset(f->active, 0, sizeof(f->active));
    memset(f->bg, 0, sizeof(f->bg));
    memset(f->ba, 0, sizeof(f->ba));

    lg_matn_zero(f->P, NDIM);

    const inekf_params_t *q = &f->params;
    for (int i = 0; i < 3; i++)
    {
        f->P[IDX(INEKF_IDX_PHI + i, INEKF_IDX_PHI + i)] = q->init_orientation * q->init_orientation;
        f->P[IDX(INEKF_IDX_V   + i, INEKF_IDX_V   + i)] = q->init_velocity    * q->init_velocity;
        f->P[IDX(INEKF_IDX_P   + i, INEKF_IDX_P   + i)] = q->init_position    * q->init_position;
        f->P[IDX(INEKF_IDX_BG  + i, INEKF_IDX_BG  + i)] = q->init_gyro_bias   * q->init_gyro_bias;
        f->P[IDX(INEKF_IDX_BA  + i, INEKF_IDX_BA  + i)] = q->init_accel_bias  * q->init_accel_bias;
    }
    /* Inactive contact blocks stay at zero until the foot lands. */
}

void inekf_init(inekf_t *f, const inekf_params_t *params)
{
    if (params != NULL)
    {
        f->params = *params;
    }
    else
    {
        inekf_default_params(&f->params);
    }
    inekf_reset(f);
}

/* --------------------------------------------------------------------- */
/*  Prediction - equations 24, 50, 51, 58, 61                             */
/* --------------------------------------------------------------------- */

void inekf_predict(inekf_t *f, const inekf_real_t *omega,
                   const inekf_real_t *accel, inekf_real_t dt)
{
    if ((dt <= 0.0f) || (dt > 0.5f))
    {
        return;
    }

    const inekf_real_t *g = f->params.gravity;

    /* Bias-corrected inputs, equation 24. */
    inekf_real_t w[3], a[3];
    for (int i = 0; i < 3; i++)
    {
        w[i] = omega[i] - f->bg[i];
        a[i] = accel[i] - f->ba[i];
    }

    inekf_real_t phi[3] = { w[0] * dt, w[1] * dt, w[2] * dt };
    inekf_real_t G0[9], G1[9], G2[9];
    lg_gamma0(G0, phi);
    lg_gamma1(G1, phi);
    lg_gamma2(G2, phi);

    /*
     * Keep the PRE-propagation state for building Phi.
     *
     * The Python takes numpy views (R = self._X[0:3,0:3]) and then writes the
     * propagated values back into self._X - which silently updates those same
     * views. state_transition_right therefore receives the ALREADY-PROPAGATED
     * state and then propagates it a second time internally. Phi linearises
     * about the state at t_k, so the pre-propagation values are used here.
     */
    inekf_real_t R_old[9], v_old[3], p_old[3];
    lg_mat3_copy(R_old, f->R);
    memcpy(v_old, f->v, sizeof(v_old));
    memcpy(p_old, f->p, sizeof(p_old));

    /* --- propagate the state, equation 50 --- */
    inekf_real_t RG1[9], RG2[9], acc[3], tmp3[9];

    lg_mat3_mul(tmp3, R_old, G0);
    lg_mat3_copy(f->R, tmp3);

    lg_mat3_mul(RG1, R_old, G1);
    lg_mat3_vec(acc, RG1, a);
    for (int i = 0; i < 3; i++)
    {
        f->v[i] = v_old[i] + acc[i] * dt + g[i] * dt;
    }

    lg_mat3_mul(RG2, R_old, G2);
    lg_mat3_vec(acc, RG2, a);
    for (int i = 0; i < 3; i++)
    {
        f->p[i] = p_old[i] + v_old[i] * dt + acc[i] * dt * dt + 0.5f * g[i] * dt * dt;
    }
    /* Contact positions are assumed stationary and do not move. */

    /* --- state transition matrix Phi, equation 58 --- */
    inekf_real_t *Phi = f->Phi;
    lg_matn_identity(Phi, NDIM);

    inekf_real_t Sg[9], I3[9], blk[9];

    /*
     * Phi(phi,phi) stays IDENTITY - see equation 58.
     *
     * This is the whole point of the right-invariant formulation: the rotation
     * error does not rotate, so the linearised error dynamics do not depend on
     * the state estimate. Gamma0^T belongs in the LEFT-invariant Phi^l
     * (equation 55), where the error is expressed in the body frame.
     *
     * The Python puts Gamma0^T here, which quietly turns this back into the
     * state-dependent linearisation the InEKF exists to avoid - exactly the
     * QEKF behaviour the paper compares against and beats.
     */

    lg_skew(Sg, g);
    lg_matn_set_block3_scaled(Phi, INEKF_IDX_V, INEKF_IDX_PHI, Sg, dt);
    lg_matn_set_block3_scaled(Phi, INEKF_IDX_P, INEKF_IDX_PHI, Sg, 0.5f * dt * dt);

    lg_mat3_identity(I3);
    lg_matn_set_block3_scaled(Phi, INEKF_IDX_P, INEKF_IDX_V, I3, dt);

    /* Bias coupling. RG1/RG2 already hold R_old*G1 and R_old*G2. */
    lg_matn_set_block3_scaled(Phi, INEKF_IDX_PHI, INEKF_IDX_BG, RG1, -dt);
    lg_matn_set_block3_scaled(Phi, INEKF_IDX_V,   INEKF_IDX_BA, RG1, -dt);
    lg_matn_set_block3_scaled(Phi, INEKF_IDX_P,   INEKF_IDX_BA, RG2, -dt * dt);

    /* v_next / p_next enter the gyro-bias coupling terms. */
    inekf_real_t v_next[3], p_next[3];
    lg_mat3_vec(acc, RG1, a);
    for (int i = 0; i < 3; i++)
    {
        v_next[i] = v_old[i] + acc[i] * dt;
    }
    lg_mat3_vec(acc, RG2, a);
    for (int i = 0; i < 3; i++)
    {
        p_next[i] = p_old[i] + v_old[i] * dt + acc[i] * dt * dt;
    }

    /* Psi1 = (a)_x Gamma2(-w dt) dt^2 ,  Psi2 = (a)_x Gamma3(-w dt) dt^3 */
    inekf_real_t nphi[3] = { -phi[0], -phi[1], -phi[2] };
    inekf_real_t Sa[9], Gn[9], Psi[9], RPsi[9], Sv[9], SvRG1[9];
    lg_skew(Sa, a);

    /* velocity <- gyro bias */
    lg_gamma2(Gn, nphi);
    lg_mat3_mul(Psi, Sa, Gn);
    lg_mat3_scale(Psi, dt * dt);
    lg_mat3_mul(RPsi, R_old, Psi);
    lg_skew(Sv, v_next);
    lg_mat3_mul(SvRG1, Sv, RG1);
    for (int i = 0; i < 9; i++)
    {
        blk[i] = -SvRG1[i] * dt + RPsi[i];
    }
    lg_matn_set_block3(Phi, INEKF_IDX_V, INEKF_IDX_BG, blk);

    /* position <- gyro bias */
    lg_gamma3(Gn, nphi);
    lg_mat3_mul(Psi, Sa, Gn);
    lg_mat3_scale(Psi, dt * dt * dt);
    lg_mat3_mul(RPsi, R_old, Psi);
    lg_skew(Sv, p_next);
    lg_mat3_mul(SvRG1, Sv, RG1);
    for (int i = 0; i < 9; i++)
    {
        blk[i] = -SvRG1[i] * dt + RPsi[i];
    }
    lg_matn_set_block3(Phi, INEKF_IDX_P, INEKF_IDX_BG, blk);

    /* each active contact <- gyro bias */
    for (int k = 0; k < INEKF_MAX_CONTACTS; k++)
    {
        if (!f->active[k])
        {
            continue;
        }
        lg_skew(Sv, f->d[k]);
        lg_mat3_mul(SvRG1, Sv, RG1);
        lg_matn_set_block3_scaled(Phi, INEKF_IDX_D(k), INEKF_IDX_BG, SvRG1, -dt);
    }

    /* --- covariance: P = Phi P Phi^T + Phi Qbar Phi^T dt, equation 61 --- */

    /* Build Qbar in tmpA. Noise on velocity and contacts is rotated into the
       world frame by R, following the right-invariant form of equation 28. */
    inekf_real_t *Qb = f->tmpA;
    lg_matn_zero(Qb, NDIM);

    const inekf_params_t *q = &f->params;

    /*
     * Q_bar = Ad_X * Cov(w) * Ad_X^T   (equation 28)
     *
     * Ad (equation 63) is NOT block diagonal: its first block-column is
     *     [ R ; (v)_x R ; (p)_x R ; (d_k)_x R ]
     * so GYRO noise leaks into velocity, position and every contact, with
     * cross-covariances between them. Both the Python and the first version of
     * this port dropped all of that and used a diagonal Q_bar. The omitted
     * terms scale with |v|, |p| and |d| - negligible near the origin, but they
     * grow as the robot walks away from where it started, which is precisely
     * when the covariance most needs to be honest.
     *
     * Forming the full 21x21 adjoint and doing two more products would cost
     * ~18k multiply-accumulates. It is not needed: for isotropic noise
     * R*Sigma*R^T = sigma^2*I, so every block collapses to
     *
     *     Q[a][b] = sigma_g^2 * Pa * Pb^T  (+ the direct noise on the diagonal)
     *
     * where Pa is I for phi, (v)_x for v, (p)_x for p and (d_k)_x for contacts.
     * That is a handful of 3x3 products.
     */
    inekf_real_t pre[2 + INEKF_MAX_CONTACTS][9];   /* prefix per block row */
    int          row[2 + INEKF_MAX_CONTACTS];
    int          nrow = 0;

    lg_mat3_identity(pre[nrow]); row[nrow] = INEKF_IDX_PHI; nrow++;
    lg_skew(pre[nrow], f->v);    row[nrow] = INEKF_IDX_V;   nrow++;
    lg_skew(pre[nrow], f->p);    row[nrow] = INEKF_IDX_P;   nrow++;
    for (int k = 0; k < INEKF_MAX_CONTACTS; k++)
    {
        if (f->active[k])
        {
            lg_skew(pre[nrow], f->d[k]);
            row[nrow] = INEKF_IDX_D(k);
            nrow++;
        }
    }

    const inekf_real_t sg2 = q->noise_gyro * q->noise_gyro;

    for (int a = 0; a < nrow; a++)
    {
        for (int b = 0; b < nrow; b++)
        {
            inekf_real_t blk3[9];
            lg_mat3_mul_bt(blk3, pre[a], pre[b]);      /* Pa * Pb^T */
            lg_matn_set_block3_scaled(Qb, row[a], row[b], blk3, sg2);
        }
    }

    /* Direct noise that does not come through the adjoint's first column:
       accelerometer on velocity, foot slip on each contact, bias random walk.
       R*Sigma*R^T = sigma^2*I for isotropic Sigma, so these are plain adds. */
    for (int i = 0; i < 3; i++)
    {
        Qb[IDX(INEKF_IDX_V + i, INEKF_IDX_V + i)] += q->noise_accel * q->noise_accel;
        Qb[IDX(INEKF_IDX_BG + i, INEKF_IDX_BG + i)] = q->noise_gyro_bias * q->noise_gyro_bias;
        Qb[IDX(INEKF_IDX_BA + i, INEKF_IDX_BA + i)] = q->noise_accel_bias * q->noise_accel_bias;
    }
    for (int k = 0; k < INEKF_MAX_CONTACTS; k++)
    {
        if (!f->active[k])
        {
            continue;
        }
        for (int i = 0; i < 3; i++)
        {
            Qb[IDX(INEKF_IDX_D(k) + i, INEKF_IDX_D(k) + i)] +=
                q->noise_contact_vel * q->noise_contact_vel;
        }
    }

    /* tmpB = Phi * P ; P = tmpB * Phi^T */
    lg_matn_mul(f->tmpB, Phi, f->P, NDIM);
    lg_matn_mul_bt(f->P, f->tmpB, Phi, NDIM);

    /* tmpB = Phi * Qbar ; tmpA = tmpB * Phi^T  (tmpA is free again) */
    lg_matn_mul(f->tmpB, Phi, Qb, NDIM);
    lg_matn_mul_bt(f->tmpA, f->tmpB, Phi, NDIM);

    for (int i = 0; i < NDIM; i++)
    {
        for (int j = 0; j < NDIM; j++)
        {
            f->P[IDX(i, j)] += f->tmpA[IDX(i, j)] * dt;
        }
    }

    lg_matn_symmetrise(f->P, NDIM);
}

/* --------------------------------------------------------------------- */
/*  Contact management                                                     */
/* --------------------------------------------------------------------- */

void inekf_add_contact(inekf_t *f, int slot,
                       const inekf_real_t *B_p_BC, const inekf_real_t *J_p)
{
    if ((slot < 0) || (slot >= INEKF_MAX_CONTACTS) || f->active[slot])
    {
        return;
    }

    /* World position of the new contact from FK, equation 31. */
    inekf_real_t Rb[3];
    lg_mat3_vec(Rb, f->R, B_p_BC);
    for (int i = 0; i < 3; i++)
    {
        f->d[slot][i] = f->p[i] + Rb[i];
    }
    f->active[slot] = 1;

    /*
     * Covariance augmentation, equation 32.
     *
     * The new contact's error starts equal to the position error, so its block
     * inherits the position block and its cross-covariances - then the encoder
     * noise mapped through R*J_p is added.
     *
     * The Python does this with an explicit F matrix; here the same result is
     * written directly, since F is only a copy of the position rows.
     */
    const int dr = INEKF_IDX_D(slot);

    for (int i = 0; i < NDIM; i++)
    {
        for (int c = 0; c < 3; c++)
        {
            f->P[IDX(i, dr + c)] = f->P[IDX(i, INEKF_IDX_P + c)];
        }
    }
    for (int j = 0; j < NDIM; j++)
    {
        for (int r = 0; r < 3; r++)
        {
            f->P[IDX(dr + r, j)] = f->P[IDX(INEKF_IDX_P + r, j)];
        }
    }
    /* The corner block must come from the position block, not from the row
       copy above, which has already been partly overwritten. */
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            f->P[IDX(dr + r, dr + c)] = f->P[IDX(INEKF_IDX_P + r, INEKF_IDX_P + c)];
        }
    }

    /* Add R J_p Sigma_enc J_p^T R^T to the new block. */
    inekf_real_t RJ[3 * KIN_LEG_JOINTS];
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < KIN_LEG_JOINTS; c++)
        {
            inekf_real_t s = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                s += f->R[r * 3 + k] * J_p[k * KIN_LEG_JOINTS + c];
            }
            RJ[r * KIN_LEG_JOINTS + c] = s;
        }
    }
    const inekf_real_t se = f->params.noise_encoder * f->params.noise_encoder;
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            inekf_real_t s = 0.0f;
            for (int k = 0; k < KIN_LEG_JOINTS; k++)
            {
                s += RJ[r * KIN_LEG_JOINTS + k] * RJ[c * KIN_LEG_JOINTS + k];
            }
            f->P[IDX(dr + r, dr + c)] += se * s;
        }
    }

    lg_matn_symmetrise(f->P, NDIM);
}

void inekf_remove_contact(inekf_t *f, int slot)
{
    if ((slot < 0) || (slot >= INEKF_MAX_CONTACTS) || !f->active[slot])
    {
        return;
    }
    f->active[slot] = 0;

    /*
     * Marginalisation, equation 30. The Python builds a selection matrix that
     * drops the rows and columns; with fixed slots, zeroing them is the same
     * operation and leaves every other index untouched.
     */
    const int dr = INEKF_IDX_D(slot);
    for (int i = 0; i < NDIM; i++)
    {
        for (int c = 0; c < 3; c++)
        {
            f->P[IDX(i, dr + c)] = 0.0f;
            f->P[IDX(dr + c, i)] = 0.0f;
        }
    }
    memset(f->d[slot], 0, sizeof(f->d[slot]));
}

/* --------------------------------------------------------------------- */
/*  Measurement update - equations 19, 20, 29                             */
/* --------------------------------------------------------------------- */

void inekf_update_contact(inekf_t *f, int slot,
                          const inekf_real_t *B_p_BC, const inekf_real_t *J_p)
{
    if ((slot < 0) || (slot >= INEKF_MAX_CONTACTS) || !f->active[slot])
    {
        return;
    }

    const int dr = INEKF_IDX_D(slot);

    /*
     * Innovation.
     *
     * The observation is Y = [B_p_BC; 0; 1; -1], and the right-invariant
     * innovation is the first three rows of X*Y:
     *
     *     z = R * B_p_BC + p - d_k
     *
     * which is exactly zero when the state agrees with forward kinematics.
     *
     * NOTE: the Python computes Xinv @ b and never uses B_p_BC at all, so its
     * innovation is R^T(d_k - p) - i.e. the prediction on its own, with the
     * measurement missing. That makes the update drive the state towards a
     * fixed point rather than towards the encoders. The measurement is
     * included here.
     */
    inekf_real_t Rb[3], z[3];
    lg_mat3_vec(Rb, f->R, B_p_BC);
    for (int i = 0; i < 3; i++)
    {
        z[i] = Rb[i] + f->p[i] - f->d[slot][i];
    }

    /*
     * H is [ ... +I at p ... -I at d_k ... ], matching the sign of z above:
     * perturbing p by dp moves z by +dp, perturbing d_k by dd moves it by -dd.
     * Only six columns are non-zero, so H is applied by picking columns of P
     * rather than forming a 3xN matrix and multiplying.
     *
     *   (P H^T)[:, c] = P[:, p+c] - P[:, d+c]
     */
    inekf_real_t PHt[NDIM][3];
    for (int i = 0; i < NDIM; i++)
    {
        for (int c = 0; c < 3; c++)
        {
            PHt[i][c] = f->P[IDX(i, INEKF_IDX_P + c)] - f->P[IDX(i, dr + c)];
        }
    }

    /* S = H P H^T + N,  a 3x3.  (H P H^T)[r][c] = PHt[p+r][c] - PHt[d+r][c] */
    inekf_real_t S[9];
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            S[r * 3 + c] = PHt[INEKF_IDX_P + r][c] - PHt[dr + r][c];
        }
    }

    /* N = R J_p Sigma_enc J_p^T R^T - encoder noise pushed through the leg. */
    inekf_real_t RJ[3 * KIN_LEG_JOINTS];
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < KIN_LEG_JOINTS; c++)
        {
            inekf_real_t s = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                s += f->R[r * 3 + k] * J_p[k * KIN_LEG_JOINTS + c];
            }
            RJ[r * KIN_LEG_JOINTS + c] = s;
        }
    }
    const inekf_real_t se = f->params.noise_encoder * f->params.noise_encoder;
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            inekf_real_t s = 0.0f;
            for (int k = 0; k < KIN_LEG_JOINTS; k++)
            {
                s += RJ[r * KIN_LEG_JOINTS + k] * RJ[c * KIN_LEG_JOINTS + k];
            }
            S[r * 3 + c] += se * s;
        }
    }

    /* Invert the 3x3 S by cofactors. */
    inekf_real_t det =
        S[0] * (S[4] * S[8] - S[5] * S[7]) -
        S[1] * (S[3] * S[8] - S[5] * S[6]) +
        S[2] * (S[3] * S[7] - S[4] * S[6]);

    if (fabsf(det) < 1e-20f)
    {
        return;   /* singular - skip rather than produce garbage */
    }
    inekf_real_t invdet = 1.0f / det;
    inekf_real_t Si[9];
    Si[0] =  (S[4] * S[8] - S[5] * S[7]) * invdet;
    Si[1] = -(S[1] * S[8] - S[2] * S[7]) * invdet;
    Si[2] =  (S[1] * S[5] - S[2] * S[4]) * invdet;
    Si[3] = -(S[3] * S[8] - S[5] * S[6]) * invdet;
    Si[4] =  (S[0] * S[8] - S[2] * S[6]) * invdet;
    Si[5] = -(S[0] * S[5] - S[2] * S[3]) * invdet;
    Si[6] =  (S[3] * S[7] - S[4] * S[6]) * invdet;
    Si[7] = -(S[0] * S[7] - S[1] * S[6]) * invdet;
    Si[8] =  (S[0] * S[4] - S[1] * S[3]) * invdet;

    /* K = P H^T S^-1,  NDIM x 3 */
    inekf_real_t K[NDIM][3];
    for (int i = 0; i < NDIM; i++)
    {
        for (int c = 0; c < 3; c++)
        {
            K[i][c] = PHt[i][0] * Si[0 * 3 + c] +
                      PHt[i][1] * Si[1 * 3 + c] +
                      PHt[i][2] * Si[2 * 3 + c];
        }
    }

    /* Tangent-space correction xi = -K z. The sign is negative because z is
       the residual of the prediction against the measurement, and the
       correction must remove it. */
    inekf_real_t xi[NDIM];
    for (int i = 0; i < NDIM; i++)
    {
        xi[i] = -(K[i][0] * z[0] + K[i][1] * z[1] + K[i][2] * z[2]);
    }

    /* --- apply to the state: X <- exp(xi) X, equation 29 --- */
    inekf_real_t dR[9], G1x[9], tmp3[9], t3[3];
    lg_gamma0(dR, &xi[INEKF_IDX_PHI]);
    lg_gamma1(G1x, &xi[INEKF_IDX_PHI]);

    /* v <- dR v + G1 xi_v , same for p and every active contact */
    lg_mat3_vec(t3, dR, f->v);
    inekf_real_t gv[3];
    lg_mat3_vec(gv, G1x, &xi[INEKF_IDX_V]);
    for (int i = 0; i < 3; i++)
    {
        f->v[i] = t3[i] + gv[i];
    }

    lg_mat3_vec(t3, dR, f->p);
    lg_mat3_vec(gv, G1x, &xi[INEKF_IDX_P]);
    for (int i = 0; i < 3; i++)
    {
        f->p[i] = t3[i] + gv[i];
    }

    for (int k = 0; k < INEKF_MAX_CONTACTS; k++)
    {
        if (!f->active[k])
        {
            continue;
        }
        lg_mat3_vec(t3, dR, f->d[k]);
        lg_mat3_vec(gv, G1x, &xi[INEKF_IDX_D(k)]);
        for (int i = 0; i < 3; i++)
        {
            f->d[k][i] = t3[i] + gv[i];
        }
    }

    lg_mat3_mul(tmp3, dR, f->R);
    lg_mat3_copy(f->R, tmp3);

    /* Biases live in a plain vector space, so they just add. */
    for (int i = 0; i < 3; i++)
    {
        f->bg[i] += xi[INEKF_IDX_BG + i];
        f->ba[i] += xi[INEKF_IDX_BA + i];
    }

    /*
     * Covariance, Joseph form:  P <- (I-KH) P (I-KH)^T + K N K^T
     *
     * The simpler P <- (I-KH)P is equivalent in exact arithmetic but loses
     * symmetry and positive-definiteness quickly in single precision. Joseph
     * form costs one more product and is what keeps this stable at float.
     */
    inekf_real_t *A = f->tmpA;    /* A = I - K H */
    lg_matn_identity(A, NDIM);
    for (int i = 0; i < NDIM; i++)
    {
        for (int c = 0; c < 3; c++)
        {
            A[IDX(i, INEKF_IDX_P + c)] -= K[i][c];
            A[IDX(i, dr + c)]          += K[i][c];
        }
    }

    lg_matn_mul(f->tmpB, A, f->P, NDIM);
    lg_matn_mul_bt(f->P, f->tmpB, A, NDIM);

    /* + K N K^T.  N is the encoder term already folded into S, so recompute
       it here from RJ rather than keeping another copy. */
    inekf_real_t N3[9];
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            inekf_real_t s = 0.0f;
            for (int k = 0; k < KIN_LEG_JOINTS; k++)
            {
                s += RJ[r * KIN_LEG_JOINTS + k] * RJ[c * KIN_LEG_JOINTS + k];
            }
            N3[r * 3 + c] = se * s;
        }
    }
    for (int i = 0; i < NDIM; i++)
    {
        inekf_real_t KN[3];
        for (int c = 0; c < 3; c++)
        {
            KN[c] = K[i][0] * N3[0 * 3 + c] +
                    K[i][1] * N3[1 * 3 + c] +
                    K[i][2] * N3[2 * 3 + c];
        }
        for (int j = 0; j < NDIM; j++)
        {
            f->P[IDX(i, j)] += KN[0] * K[j][0] + KN[1] * K[j][1] + KN[2] * K[j][2];
        }
    }

    lg_matn_symmetrise(f->P, NDIM);
}

/* --------------------------------------------------------------------- */
/*  Accessors                                                              */
/* --------------------------------------------------------------------- */

inekf_real_t inekf_height(const inekf_t *f)
{
    return f->p[2];
}

void inekf_velocity_world(const inekf_t *f, inekf_real_t *v3)
{
    memcpy(v3, f->v, sizeof(f->v));
}

void inekf_velocity_body(const inekf_t *f, inekf_real_t *v3)
{
    lg_mat3_vec_t(v3, f->R, f->v);
}

void inekf_quaternion(const inekf_t *f, inekf_real_t *q4)
{
    /* Shepperd's method: pick the branch with the largest denominator so the
       square root never operates on a near-zero value. */
    const inekf_real_t *R = f->R;
    inekf_real_t tr = R[0] + R[4] + R[8];

    if (tr > 0.0f)
    {
        inekf_real_t s = sqrtf(tr + 1.0f) * 2.0f;
        q4[0] = 0.25f * s;
        q4[1] = (R[7] - R[5]) / s;
        q4[2] = (R[2] - R[6]) / s;
        q4[3] = (R[3] - R[1]) / s;
    }
    else if ((R[0] > R[4]) && (R[0] > R[8]))
    {
        inekf_real_t s = sqrtf(1.0f + R[0] - R[4] - R[8]) * 2.0f;
        q4[0] = (R[7] - R[5]) / s;
        q4[1] = 0.25f * s;
        q4[2] = (R[1] + R[3]) / s;
        q4[3] = (R[2] + R[6]) / s;
    }
    else if (R[4] > R[8])
    {
        inekf_real_t s = sqrtf(1.0f + R[4] - R[0] - R[8]) * 2.0f;
        q4[0] = (R[2] - R[6]) / s;
        q4[1] = (R[1] + R[3]) / s;
        q4[2] = 0.25f * s;
        q4[3] = (R[5] + R[7]) / s;
    }
    else
    {
        inekf_real_t s = sqrtf(1.0f + R[8] - R[0] - R[4]) * 2.0f;
        q4[0] = (R[3] - R[1]) / s;
        q4[1] = (R[2] + R[6]) / s;
        q4[2] = (R[5] + R[7]) / s;
        q4[3] = 0.25f * s;
    }
}

int inekf_num_contacts(const inekf_t *f)
{
    int n = 0;
    for (int k = 0; k < INEKF_MAX_CONTACTS; k++)
    {
        n += f->active[k] ? 1 : 0;
    }
    return n;
}
