/*
 * Feed recorded or simulated sensor data through the REAL estimator.
 *
 * This is fusion.c, inekf.c, lie_group.c, zeus_kinematics.c and robot_config.c
 * exactly as they are flashed, compiled for the workstation instead of the
 * STM32, and driven tick by tick from a file instead of from interrupts. What
 * comes out is what the robot would have put in its USB packet.
 *
 *     replay input.bin estimate.bin
 *
 * input.bin   one record per 1 kHz tick, every field a little-endian double
 *             (the layout is INPUT_FIELDS in tools/sim/gen_walk.py):
 *               t_us, imu_new, gyro[3], accel[3], pos[10] (turns),
 *               pos_age[10], enc[4] (raw counts), enc_valid, contacts
 * estimate.bin one record per tick, every field a double:
 *               t_us, status, contacts_used, quat[4] (w,x,y,z),
 *               fused_pos[3], fused_vel[3] (world), gyro_bias[3],
 *               accel_bias[3], foot_z[2] ([0] right, [1] left), fk_valid,
 *               vel_hdg[3] (lateral, forward, vertical)
 *
 * Built by tools/sim/run.sh with the host-test stubs, the same way
 * tools/hosttest/run.sh builds test_fusion.
 */

#include "fusion.h"
#include "robot_config.h"

#include <stdio.h>
#include <string.h>

#define IN_FIELDS   (1 + 1 + 3 + 3 + NEXUS_NUM_JOINTS + NEXUS_NUM_JOINTS + NEXUS_NUM_ENCODERS + 1 + 1)
#define OUT_FIELDS  25

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        fprintf(stderr, "usage: %s input.bin estimate.bin\n", argv[0]);
        return 2;
    }

    FILE *in  = fopen(argv[1], "rb");
    FILE *out = fopen(argv[2], "wb");
    if ((in == NULL) || (out == NULL))
    {
        fprintf(stderr, "cannot open %s or %s\n", argv[1], argv[2]);
        return 2;
    }

    imu_sample_t    imu;
    act_telemetry_t act;
    memset(&imu, 0, sizeof(imu));
    memset(&act, 0, sizeof(act));
    imu.quat[0] = 1.0f;

    fusion_init();

    double r[IN_FIELDS];
    long   ticks = 0;

    while (fread(r, sizeof(double), IN_FIELDS, in) == IN_FIELDS)
    {
        int i = 0;
        uint32_t t_us = (uint32_t)r[i++];

        if (r[i++] != 0.0)
        {
            for (int k = 0; k < 3; k++) { imu.gyro[k]  = (float)r[i + k]; }
            for (int k = 0; k < 3; k++) { imu.accel[k] = (float)r[i + 3 + k]; }
            imu.gyro_seq++;
            imu.accel_seq++;
            imu.seq++;
        }
        i += 6;

        for (int j = 0; j < NEXUS_NUM_JOINTS; j++) { act.pos[j]     = (float)r[i++]; }
        for (int j = 0; j < NEXUS_NUM_JOINTS; j++) { act.pos_age[j] = (uint16_t)r[i++]; }

        float spring[NEXUS_NUM_ENCODERS];
        for (int e = 0; e < NEXUS_NUM_ENCODERS; e++)
        {
            spring[e] = robot_spring_deflection((uint8_t)e, (uint16_t)r[i++]);
        }
        uint8_t enc_valid = (uint8_t)r[i++];
        uint8_t contacts  = (uint8_t)r[i++];

        fusion_tick(&imu, &act, spring, enc_valid, contacts, t_us);

        nexus_state_t st;
        memset(&st, 0, sizeof(st));
        fusion_fill_state(&st);

        double o[OUT_FIELDS];
        int n = 0;
        o[n++] = t_us;
        o[n++] = st.fused_valid;
        o[n++] = fusion_num_contacts();
        for (int k = 0; k < 4; k++) { o[n++] = st.quat[k]; }
        for (int k = 0; k < 3; k++) { o[n++] = st.fused_pos[k]; }
        for (int k = 0; k < 3; k++) { o[n++] = st.fused_vel[k]; }
        for (int k = 0; k < 3; k++) { o[n++] = st.fused_gyro_bias[k]; }
        for (int k = 0; k < 3; k++) { o[n++] = st.fused_accel_bias[k]; }
        for (int k = 0; k < 2; k++) { o[n++] = st.foot_z[k]; }
        o[n++] = st.fk_valid;
        for (int k = 0; k < 3; k++) { o[n++] = st.vel_hdg[k]; }

        fwrite(o, sizeof(double), OUT_FIELDS, out);
        ticks++;
    }

    fclose(in);
    fclose(out);
    fprintf(stderr, "replayed %ld ticks through fusion.c\n", ticks);
    return (ticks > 0) ? 0 : 1;
}
