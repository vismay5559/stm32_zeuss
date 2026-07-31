#include "app.h"
#include "main.h"
#include "link_proto.h"
#include "link_usb.h"
#include "imu_bno085.h"
#include "enc_as5048a.h"
#include "act_odrive.h"
#include "contact.h"
#include "critical.h"
#include <string.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

static volatile uint32_t s_tick_pending;
static uint32_t          s_seq;
static uint32_t          s_overruns;
static nexus_state_t     s_state;

uint32_t app_overruns(void)
{
    return s_overruns;
}

/*
 * Every DMA buffer in this project lives in the "noncacheable_buffer" linker
 * section, but a section name alone changes nothing - what actually makes it
 * non-cacheable is MPU Region 2, configured in the *Boot* project
 * (CORTEX_M7_BOOT in CubeMX). If those two ever disagree, the CPU reads stale
 * cached copies while DMA writes real RAM, and sensor data goes intermittently
 * corrupt in a way that looks exactly like bad wiring.
 *
 * CubeMX will happily regenerate Region 2 back to its default, so verify the
 * agreement at boot rather than trusting it. A hang here means Region 2 no
 * longer covers the buffers - go fix it in CubeMX, not here.
 */
#define NONCACHEABLE_MPU_BASE  0x24070000u
#define NONCACHEABLE_MPU_SIZE  0x2000u

extern uint32_t __NONCACHEABLEBUFFER_BEGIN;
extern uint32_t __NONCACHEABLEBUFFER_END;

static void check_noncacheable_region(void)
{
    uint32_t begin = (uint32_t)&__NONCACHEABLEBUFFER_BEGIN;
    uint32_t end   = (uint32_t)&__NONCACHEABLEBUFFER_END;

    if ((begin < NONCACHEABLE_MPU_BASE) ||
        (end > (NONCACHEABLE_MPU_BASE + NONCACHEABLE_MPU_SIZE)))
    {
        /* main() has not reached its BSP_LED_Init calls yet, and Error_Handler
           spins forever with interrupts off - so light the LED here or this
           failure is completely silent. */
        BSP_LED_Init(LED_RED);
        BSP_LED_On(LED_RED);
        Error_Handler();
    }
}

/*
 * Health LEDs - the only outward sign the firmware is alive, since everything
 * else goes out over USB to the Pi.
 *
 *   LD1 green : 1 Hz heartbeat, driven from the 1 kHz tick. A steady, even
 *               blink is direct proof the control loop is running at the rate
 *               it is supposed to. If it stops, stutters, or visibly changes
 *               rate, the loop is in trouble - watch this during bring-up.
 *   LD3 red   : latched fault. Comes on and stays on the first time a tick is
 *               missed or a CAN frame is dropped, so a fault that happened
 *               seconds ago is still visible when you look up.
 */
#define HEARTBEAT_TICKS  500u   /* toggle every 500 ms -> 1 Hz full cycle */

static void update_health_leds(void)
{
    static uint32_t beat;
    static uint8_t  fault_latched;

    if (++beat >= HEARTBEAT_TICKS)
    {
        beat = 0;
        BSP_LED_Toggle(LED_GREEN);
    }

    if (!fault_latched &&
        ((s_overruns != 0u) || (act_tx_dropped(0) != 0u) || (act_tx_dropped(1) != 0u)))
    {
        fault_latched = 1;
        BSP_LED_On(LED_RED);
    }
}

void app_init(void)
{
    check_noncacheable_region();

    memset(&s_state, 0, sizeof(s_state));
    s_tick_pending = 0;
    s_seq          = 0;
    s_overruns     = 0;

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

        /* Keep feeding the 3-deep hardware CAN FIFO from the software queue.
           This runs far more often than once per tick, so the five frames
           queued each tick reach the wire well inside that tick. */
        act_tx_pump();

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

        /* Drop backlog rather than replaying stale ticks if a cycle ever
           overruns, but count what was dropped - a rising s_overruns is the
           first sign the 1 kHz budget is being exceeded. Claiming the counter
           has to be atomic against the TIM6 ISR that increments it. */
        uint32_t primask = critical_enter();
        uint32_t pending = s_tick_pending;
        s_tick_pending = 0;
        critical_exit(primask);

        if (pending > 1u)
        {
            s_overruns += (pending - 1u);
        }

        enc_start_read();
        contact_poll();
        act_tick_1khz();
        build_and_send_state();
        update_health_leds();
    }
}
