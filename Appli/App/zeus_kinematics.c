#include "zeus_kinematics.h"

#include <math.h>
#include <stddef.h>

/*
 * A leg, as the generator hands it over, is a straight walk from the IMU to
 * the foot through ZK_STEPS joints:
 *
 *     T = pre_0 * Rot(axis_0, sign_0 * q[qi_0]) * pre_1 * Rot(...) * ... ,
 *     contact = T * point
 *
 * `pre` is everything rigid between one joint and the next, already multiplied
 * together. `sign` is -1 where the walk crosses a joint from child to parent -
 * the waist, since the IMU sits above it and the legs hang below.
 *
 * The Jacobian needs no extra FK. A revolute joint turning by dq moves any
 * point downstream of it by (axis x (point - joint origin)) dq, so recording
 * each joint's axis and origin on the way down is enough.
 */
#include "zeus_kinematics_model.h"

static void mat3_mul(float out[9], const float a[9], const float b[9])
{
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            out[3 * r + c] = a[3 * r] * b[c] + a[3 * r + 1] * b[3 + c] + a[3 * r + 2] * b[6 + c];
        }
    }
}

static void mat3_vec(float out[3], const float m[9], const float v[3])
{
    for (int r = 0; r < 3; r++)
    {
        out[r] = m[3 * r] * v[0] + m[3 * r + 1] * v[1] + m[3 * r + 2] * v[2];
    }
}

/* Rotation by `angle` about the unit vector `a` (Rodrigues). */
static void axis_angle(float R[9], const float a[3], float angle)
{
    float c = cosf(angle);
    float s = sinf(angle);
    float t = 1.0f - c;

    R[0] = t * a[0] * a[0] + c;       R[1] = t * a[0] * a[1] - s * a[2]; R[2] = t * a[0] * a[2] + s * a[1];
    R[3] = t * a[0] * a[1] + s * a[2]; R[4] = t * a[1] * a[1] + c;       R[5] = t * a[1] * a[2] - s * a[0];
    R[6] = t * a[0] * a[2] - s * a[1]; R[7] = t * a[1] * a[2] + s * a[0]; R[8] = t * a[2] * a[2] + c;
}

int zeus_kin_foot(zeus_kin_side_t side,
                  const float q[ZEUS_KIN_NQ],
                  float p[ZEUS_KIN_POINTS][3],
                  float J[ZEUS_KIN_POINTS][3 * ZEUS_KIN_NQ])
{
    if ((side != ZEUS_KIN_LEFT) && (side != ZEUS_KIN_RIGHT))
    {
        return -1;
    }
    const zk_leg_t *leg = &zk_legs[side];

    /* Running transform, IMU frame -> current link. */
    float R[9] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    float t[3] = { 0.0f, 0.0f, 0.0f };

    float origin[ZK_STEPS][3];      /* joint i's origin, IMU frame             */
    float axis[ZK_STEPS][3];        /* joint i's axis, IMU frame, times sign   */
    float tmp[9], Rj[9], v[3];

    for (int i = 0; i < ZK_STEPS; i++)
    {
        const zk_step_t *st = &leg->step[i];

        mat3_vec(v, R, st->pre_p);
        t[0] += v[0];
        t[1] += v[1];
        t[2] += v[2];
        mat3_mul(tmp, R, st->pre_R);

        origin[i][0] = t[0];
        origin[i][1] = t[1];
        origin[i][2] = t[2];
        mat3_vec(axis[i], tmp, st->axis);
        axis[i][0] *= st->sign;
        axis[i][1] *= st->sign;
        axis[i][2] *= st->sign;

        axis_angle(Rj, st->axis, st->sign * q[st->q]);
        mat3_mul(R, tmp, Rj);
    }

    for (int k = 0; k < ZEUS_KIN_POINTS; k++)
    {
        mat3_vec(v, R, leg->point[k]);
        p[k][0] = t[0] + v[0];
        p[k][1] = t[1] + v[1];
        p[k][2] = t[2] + v[2];

        if (J == NULL)
        {
            continue;
        }
        for (int i = 0; i < 3 * ZEUS_KIN_NQ; i++)
        {
            J[k][i] = 0.0f;
        }
        for (int i = 0; i < ZK_STEPS; i++)
        {
            const float *w = axis[i];
            float d[3] = { p[k][0] - origin[i][0], p[k][1] - origin[i][1], p[k][2] - origin[i][2] };
            int c = leg->step[i].q;

            J[k][0 * ZEUS_KIN_NQ + c] += w[1] * d[2] - w[2] * d[1];
            J[k][1 * ZEUS_KIN_NQ + c] += w[2] * d[0] - w[0] * d[2];
            J[k][2 * ZEUS_KIN_NQ + c] += w[0] * d[1] - w[1] * d[0];
        }
    }
    return 0;
}
