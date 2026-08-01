#include "kinematics.h"
#include <math.h>
#include <string.h>

void kin_defaults(kin_params_t *p)
{
    p->thigh_length = 0.30f;
    p->shank_length = 0.30f;
    p->foot_height  = 0.05f;

    p->left_hip_offset[0]  =  0.0f;
    p->left_hip_offset[1]  =  0.05f;
    p->left_hip_offset[2]  =  0.0f;

    p->right_hip_offset[0] =  0.0f;
    p->right_hip_offset[1] = -0.05f;
    p->right_hip_offset[2] =  0.0f;
}

/*
 * The Python builds 4x4 homogeneous transforms and multiplies them. Doing that
 * here would mean eight 4x4 products per foot per call, and the Jacobian calls
 * FK five times - so ~40 matrix products per foot, per filter step.
 *
 * The chain is only rotations about Y and X plus translations along -Z, so it
 * collapses into a running (R, p) pair updated in place. Same result, a small
 * fraction of the work, and it makes the structure of the chain readable.
 */
static void fk_position(const kin_params_t *params,
                        const inekf_real_t *hip_offset,
                        const inekf_real_t *q,
                        inekf_real_t *p_out)
{
    /* Running transform: world-of-body -> current link. */
    inekf_real_t R[9];
    inekf_real_t p[3];

    lg_mat3_identity(R);
    p[0] = hip_offset[0];
    p[1] = hip_offset[1];
    p[2] = hip_offset[2];

    /* Rotate R in place by a joint, then translate along the new -Z. */
    inekf_real_t Rj[9], Rtmp[9], d[3], dw[3];

    /* --- hip pitch, about +Y --- */
    {
        inekf_real_t c = cosf(q[0]), s = sinf(q[0]);
        Rj[0] =  c;   Rj[1] = 0.0f; Rj[2] =  s;
        Rj[3] = 0.0f; Rj[4] = 1.0f; Rj[5] = 0.0f;
        Rj[6] = -s;   Rj[7] = 0.0f; Rj[8] =  c;
        lg_mat3_mul(Rtmp, R, Rj);
        lg_mat3_copy(R, Rtmp);
    }

    /* --- hip roll, about +X --- */
    {
        inekf_real_t c = cosf(q[1]), s = sinf(q[1]);
        Rj[0] = 1.0f; Rj[1] = 0.0f; Rj[2] = 0.0f;
        Rj[3] = 0.0f; Rj[4] =  c;   Rj[5] = -s;
        Rj[6] = 0.0f; Rj[7] =  s;   Rj[8] =  c;
        lg_mat3_mul(Rtmp, R, Rj);
        lg_mat3_copy(R, Rtmp);
    }

    /* --- thigh: translate -thigh along local Z --- */
    d[0] = 0.0f; d[1] = 0.0f; d[2] = -params->thigh_length;
    lg_mat3_vec(dw, R, d);
    p[0] += dw[0]; p[1] += dw[1]; p[2] += dw[2];

    /* --- knee pitch, about +Y --- */
    {
        inekf_real_t c = cosf(q[2]), s = sinf(q[2]);
        Rj[0] =  c;   Rj[1] = 0.0f; Rj[2] =  s;
        Rj[3] = 0.0f; Rj[4] = 1.0f; Rj[5] = 0.0f;
        Rj[6] = -s;   Rj[7] = 0.0f; Rj[8] =  c;
        lg_mat3_mul(Rtmp, R, Rj);
        lg_mat3_copy(R, Rtmp);
    }

    /* --- shank --- */
    d[2] = -params->shank_length;
    lg_mat3_vec(dw, R, d);
    p[0] += dw[0]; p[1] += dw[1]; p[2] += dw[2];

    /* --- ankle pitch, about +Y --- */
    {
        inekf_real_t c = cosf(q[3]), s = sinf(q[3]);
        Rj[0] =  c;   Rj[1] = 0.0f; Rj[2] =  s;
        Rj[3] = 0.0f; Rj[4] = 1.0f; Rj[5] = 0.0f;
        Rj[6] = -s;   Rj[7] = 0.0f; Rj[8] =  c;
        lg_mat3_mul(Rtmp, R, Rj);
        lg_mat3_copy(R, Rtmp);
    }

    /* --- foot: ankle down to the contact point --- */
    d[2] = -params->foot_height;
    lg_mat3_vec(dw, R, d);
    p[0] += dw[0]; p[1] += dw[1]; p[2] += dw[2];

    p_out[0] = p[0];
    p_out[1] = p[1];
    p_out[2] = p[2];
}

void kin_foot(const kin_params_t *params,
              const inekf_real_t *hip_offset,
              const inekf_real_t *q,
              inekf_real_t *p_out,
              inekf_real_t *J_out)
{
    fk_position(params, hip_offset, q, p_out);

    /*
     * Jacobian by central differences.
     *
     * The Python uses a FORWARD difference with eps = 1e-6. That is a poor
     * choice in single precision: the FK output is order 0.5 m, so 1e-6 rad
     * moves it by ~1e-7 m, which is at the edge of float resolution (~6e-8
     * relative) - the difference would be mostly rounding noise.
     *
     * Central differences with a larger step give O(h^2) accuracy instead of
     * O(h), so a step big enough to stay well clear of the noise floor is
     * still more accurate than the original. Costs 8 FK evaluations per foot
     * rather than 5, which is cheap given how light fk_position is.
     */
    const inekf_real_t eps = 1e-3f;

    for (int i = 0; i < KIN_LEG_JOINTS; i++)
    {
        inekf_real_t qp[KIN_LEG_JOINTS], qm[KIN_LEG_JOINTS];
        inekf_real_t pp[3], pm[3];

        memcpy(qp, q, sizeof(qp));
        memcpy(qm, q, sizeof(qm));
        qp[i] += eps;
        qm[i] -= eps;

        fk_position(params, hip_offset, qp, pp);
        fk_position(params, hip_offset, qm, pm);

        for (int r = 0; r < 3; r++)
        {
            J_out[r * KIN_LEG_JOINTS + i] = (pp[r] - pm[r]) / (2.0f * eps);
        }
    }
}
