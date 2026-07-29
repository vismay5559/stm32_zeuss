#include "app.h"
#include "main.h"
#include "link_proto.h"
#include "link_usb.h"
#include "imu_bno085.h"
#include "enc_as5048a.h"
#include "act_odrive.h"
#include "contact.h"
#include <string.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

static volatile uint32_t s_tick_pending;
static uint32_t          s_seq;
static nexus_state_t     s_state;

void app_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_tick_pending = 0;
    s_seq          = 0;

    contact_init();
    enc_init();
    act_init();
    link_usb_init();
    imu_init();

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start_IT(&htim6);
}

void app_on_tick(void)
{
    s_tick_pending++;
}

static void build_and_send_state(void)
{
    imu_sample_t    imu;
    act_telemetry_t act;

    imu_get(&imu);
    act_get(&act);

    s_state.seq          = s_seq++;
    s_state.timestamp_us = __HAL_TIM_GET_COUNTER(&htim2);

    memcpy(s_state.imu_quat,  imu.quat,  sizeof(s_state.imu_quat));
    memcpy(s_state.imu_accel, imu.accel, sizeof(s_state.imu_accel));
    memcpy(s_state.imu_gyro,  imu.gyro,  sizeof(s_state.imu_gyro));
    s_state.imu_seq = imu.seq;

    uint16_t enc_angle[NEXUS_NUM_ENCODERS];
    uint8_t  enc_valid;

    enc_get(enc_angle, &enc_valid);
    memcpy(s_state.enc_angle, enc_angle, sizeof(s_state.enc_angle));
    s_state.enc_valid = enc_valid;

    s_state.contacts         = (uint8_t)(contact_switches() | contact_feet());
    s_state.contact_ticks[0] = contact_stable_ticks(0);
    s_state.contact_ticks[1] = contact_stable_ticks(1);

    memcpy(s_state.act_pos,    act.pos,        sizeof(s_state.act_pos));
    memcpy(s_state.act_vel,    act.vel,        sizeof(s_state.act_vel));
    memcpy(s_state.act_torque, act.torque,     sizeof(s_state.act_torque));
    memcpy(s_state.act_error,  act.axis_error, sizeof(s_state.act_error));
    memcpy(s_state.act_state,  act.axis_state, sizeof(s_state.act_state));
    memcpy(s_state.act_flags,  act.flags,      sizeof(s_state.act_flags));

    link_usb_send_state(&s_state);
}

void app_run(void)
{
    nexus_cmd_t cmd;

    for (;;)
    {
        imu_service();

        if (link_usb_take_command(&cmd))
        {
            float targets[NEXUS_NUM_JOINTS];

            memcpy(targets, cmd.target_pos, sizeof(targets));
            act_set_targets(targets);
        }

        if (s_tick_pending == 0u)
        {
            continue;
        }

        /* Drop backlog rather than replaying stale ticks if a cycle ever overruns. */
        s_tick_pending = 0;

        enc_start_read();
        contact_poll();
        act_tick_1khz();
        build_and_send_state();
    }
}
