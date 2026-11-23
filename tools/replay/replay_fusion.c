/*
 * Replay a recording through the firmware's own estimator, on a laptop.
 *
 *   tools/replay/run.sh RECORDING [every_n_ticks]
 *
 * Built from Appli/App/fusion.c, inekf.c, lie_group.c, zeus_kinematics.c and
 * robot_config.c unchanged, so what it shows is what the board computes, fed
 * what the board's packets say it was fed. Two inputs are reconstructed, not
 * recorded: each drive's pos_age (from the per-tick fresh flag) and the IMU's
 * accel/gyro sequence numbers (a new sample is a changed value).
 *
 * Prints, every N ticks: the replayed estimate next to the one the board
 * reported. If the two agree, the replay is faithful and any change to the
 * filter can be tried here first - never by trial and error on the robot.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fusion.h"
#include "imu_bno085.h"
#include "act_odrive.h"
#include "link_proto.h"

#pragma pack(push, 1)
typedef struct
{
    uint32_t t_us;
    float    accel[3], gyro[3];
    float    pos[8];
    uint32_t err[8];
    uint8_t  fresh[8];
    float    spring[4];
    uint8_t  enc_valid, contacts;
    float    rec_z, rec_v[3];
    uint8_t  rec_status;
} tick_t;
#pragma pack(pop)

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: replay_fusion FILE.ticks [every]\n"); return 2; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 2; }
    int every = (argc > 2) ? atoi(argv[2]) : 1000;
    long start = (argc > 3) ? atol(argv[3]) : 0;   /* START_TICK: skip to here, then start fresh */

    imu_sample_t    imu;
    act_telemetry_t act;
    memset(&imu, 0, sizeof(imu));
    memset(&act, 0, sizeof(act));
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++) { act.pos_age[j] = 0xFFFFu; }

    fusion_init();

    tick_t r, prev;
    memset(&prev, 0, sizeof(prev));
    long n = 0;
    printf("     t  | replay: st  z        vx      vy      vz    | board: st  z        vx      vy      vz\n");
    while (fread(&r, sizeof(r), 1, f) == 1)
    {
        if (n < start) { prev = r; n++; continue; }
        if (memcmp(r.accel, prev.accel, sizeof(r.accel)) != 0) { imu.accel_seq++; imu.seq++; }
        if (memcmp(r.gyro,  prev.gyro,  sizeof(r.gyro))  != 0) { imu.gyro_seq++;  imu.seq++; }
        memcpy(imu.accel, r.accel, sizeof(imu.accel));
        memcpy(imu.gyro,  r.gyro,  sizeof(imu.gyro));

        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            act.pos[j]        = r.pos[j];
            act.axis_error[j] = r.err[j];
#ifdef SPARSE
            /* Emulate the board: a frame only reads as age 0 if it landed in
               the short window between act_tick_1khz() and act_get(). */
            if (r.fresh[j] && (rand() % SPARSE) == 0) { act.pos_age[j] = 0; }
            else if (act.pos_age[j] < 0xFFFFu) { act.pos_age[j] = (uint16_t)(r.fresh[j] ? 1 : act.pos_age[j] + 1); }
#else
            if (r.fresh[j]) { act.pos_age[j] = 0; }
            else if (act.pos_age[j] < 0xFFFFu) { act.pos_age[j]++; }
#endif
            act.pos_new[j] = r.fresh[j];   /* a frame this tick, as act_get() reports it */
        }

        fusion_tick(&imu, &act, r.spring, r.enc_valid, r.contacts, r.t_us);

        nexus_state_t st;
        memset(&st, 0, sizeof(st));
        fusion_fill_state(&st);
        if ((n % every) == 0)
        {
            printf("%7.3f | %u %+8.3f %+7.3f %+7.3f %+7.3f | %u %+8.3f %+7.3f %+7.3f %+7.3f\n",
                   (double)n / 1000.0, st.fused_valid, (double)st.pelvis_z,
                   (double)st.fused_vel[0], (double)st.fused_vel[1], (double)st.fused_vel[2],
                   r.rec_status, (double)r.rec_z,
                   (double)r.rec_v[0], (double)r.rec_v[1], (double)r.rec_v[2]);
        }
        prev = r;
        n++;
    }
    fclose(f);
    printf("IMU samples rejected as glitches: %u\n", fusion_imu_rejected());
    return 0;
}
