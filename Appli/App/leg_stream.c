#include "leg_stream.h"

#include <math.h>
#include <string.h>

#define TWO_PI                 6.28318530718f
#define ODRV_CLOSED_LOOP       8u
#define NODES_PER_BUS          4u

static float turns_to_output_rad(float turns, float scale)
{
    return (scale > 0.0f) ? (turns / scale) * TWO_PI : 0.0f;
}

static uint16_t sat16(uint32_t v)
{
    return (v > 0xFFFFu) ? 0xFFFFu : (uint16_t)v;
}

static uint8_t sat8(uint32_t v)
{
    return (v > 0xFFu) ? 0xFFu : (uint8_t)v;
}

void leg_stream_fill(nexus_state_t *st,
                     const leg_stream_joint_t *joints, int count,
                     const leg_stream_status_t *status)
{
    memset(st, 0, sizeof(*st));

    st->seq          = status->seq;
    st->timestamp_us = status->timestamp_us;

    /* Absent, and saying so: identity rather than an all-zero quaternion,
       which is not a rotation at all. */
    st->quat[0]     = 1.0f;
    st->imu_quat[0] = 1.0f;
    st->foot_z[0]   = (float)NAN;
    st->foot_z[1]   = (float)NAN;

    /* The leg test drives the joints itself, without the interpolator or the
       safety envelope, so there is no "what the drive was told" to report.
       NaN rather than 0, which would plot as a command of zero. */
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        st->act_target[j] = (float)NAN;
    }
    st->fk_valid    = 0u;
    st->fused_valid = NEXUS_FUSION_INVALID;

    uint8_t any_closed_loop = 0u;

    for (int k = 0; k < count; k++)
    {
        const leg_stream_joint_t *jt = &joints[k];

        if (!jt->live || (jt->node < 1u) || (jt->node > NODES_PER_BUS))
        {
            continue;
        }

        /* The leg runs on FDCAN1, bus 0: NEXUS_J_* index = node - 1. */
        uint8_t i = (uint8_t)(jt->node - 1u);

        st->joint_pos[i]  = turns_to_output_rad(jt->pos_turns, jt->scale);
        st->joint_vel[i]  = turns_to_output_rad(jt->vel_turns_s, jt->scale);
        st->ref_angle[i]  = turns_to_output_rad(jt->cmd_turns, jt->scale);
        st->act_torque[i] = jt->torque;
        st->act_error[i]  = jt->axis_error;
        st->act_state[i]  = jt->axis_state;
        st->act_flags[i]  = jt->fresh ? (uint8_t)(NEXUS_ACT_TELEM_FRESH | NEXUS_ACT_HB_FRESH)
                                      : 0u;

        if (jt->axis_state == ODRV_CLOSED_LOOP)
        {
            any_closed_loop = 1u;
        }
    }

    float p = status->gait_phase;
    if (!(p >= 0.0f))
    {
        p = 0.0f;                              /* also catches NaN */
    }
    st->phase = p - (float)(uint32_t)p;

    st->stream_flags = (uint8_t)(NEXUS_STREAM_LEG_TEST |
                                 (status->gait_running ? NEXUS_STREAM_GAIT_LIVE : 0u));

    st->safety_state = status->stopped ? NEXUS_SAFETY_FAULT
                     : any_closed_loop ? NEXUS_SAFETY_ARMED
                                       : NEXUS_SAFETY_IDLE;

    st->can_dropped[0] = sat16(status->can_dropped);
    st->can_bus_off[0] = sat8(status->can_bus_off);
}
