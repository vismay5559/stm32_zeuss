/*
 * Host test for leg_stream.c: the leg test's data in the robot's USB packet.
 *
 * What goes wrong in a packer is quiet: a joint written to the wrong index
 * plots as the wrong joint, a scale applied twice (or not at all) plots a 9:1
 * ankle nine times too big, and a zero where "not measured" belongs reads as a
 * real measurement. Each of those is checked here.
 */

#include "leg_stream.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_fail;

#define CHECK(cond, ...)                                                 \
    do {                                                                 \
        if (!(cond)) {                                                   \
            s_fail++;                                                    \
            printf("  FAIL  ");                                          \
            printf(__VA_ARGS__);                                         \
            printf("\n");                                                \
        }                                                                \
    } while (0)

#define NEAR(a, b)  (fabs((double)(a) - (double)(b)) < 1e-5)
#define TWO_PI      6.28318530718

/* The bench leg as it is wired: hip_pitch node 1 and knee node 3 on load-side
   encoders (scale 1), ankle node 4 on the motor side of a 9:1 (scale 9). */
static void bench_leg(leg_stream_joint_t j[3])
{
    memset(j, 0, 3 * sizeof(*j));

    j[0] = (leg_stream_joint_t){ .node = 1, .scale = 1.0f, .pos_turns = 0.05f,
        .vel_turns_s = 0.10f, .cmd_turns = 0.06f, .torque = 1.5f,
        .axis_error = 0, .axis_state = 8, .live = 1, .fresh = 1 };
    j[1] = (leg_stream_joint_t){ .node = 3, .scale = 1.0f, .pos_turns = -0.02f,
        .vel_turns_s = 0.0f, .cmd_turns = -0.025f, .torque = -0.4f,
        .axis_error = 0, .axis_state = 8, .live = 1, .fresh = 1 };
    j[2] = (leg_stream_joint_t){ .node = 4, .scale = 9.0f, .pos_turns = 0.90f,
        .vel_turns_s = -1.8f, .cmd_turns = 0.99f, .torque = 0.2f,
        .axis_error = 0x08000200u, .axis_state = 1, .live = 1, .fresh = 0 };
}

static leg_stream_status_t status(void)
{
    return (leg_stream_status_t){ .seq = 1234, .timestamp_us = 5678,
        .gait_phase = 2.25f, .gait_running = 1, .stopped = 0,
        .can_dropped = 70000u, .can_bus_off = 300u };
}

static void test_joints_land_at_their_joint_map_index(void)
{
    printf("each drive lands at index node - 1, in output radians\n");

    leg_stream_joint_t j[3];
    leg_stream_status_t s = status();
    nexus_state_t st;

    bench_leg(j);
    leg_stream_fill(&st, j, 3, &s);

    CHECK(NEAR(st.joint_pos[NEXUS_J_L_HIP_PITCH], 0.05 * TWO_PI),
          "hip_pitch pos %f", (double)st.joint_pos[NEXUS_J_L_HIP_PITCH]);
    CHECK(NEAR(st.ref_angle[NEXUS_J_L_HIP_PITCH], 0.06 * TWO_PI), "hip_pitch ref");
    CHECK(NEAR(st.joint_vel[NEXUS_J_L_HIP_PITCH], 0.10 * TWO_PI), "hip_pitch vel");
    CHECK(NEAR(st.act_torque[NEXUS_J_L_HIP_PITCH], 1.5), "hip_pitch torque");

    CHECK(NEAR(st.joint_pos[NEXUS_J_L_KNEE_PITCH], -0.02 * TWO_PI), "knee pos");
    CHECK(NEAR(st.ref_angle[NEXUS_J_L_KNEE_PITCH], -0.025 * TWO_PI), "knee ref");

    /* 0.9 motor turns through 9:1 is 0.1 output turn - not 0.9. */
    CHECK(NEAR(st.joint_pos[NEXUS_J_L_ANKLE_PITCH], 0.1 * TWO_PI),
          "ankle pos %f: the 9:1 scale was not applied once",
          (double)st.joint_pos[NEXUS_J_L_ANKLE_PITCH]);
    CHECK(NEAR(st.ref_angle[NEXUS_J_L_ANKLE_PITCH], 0.11 * TWO_PI), "ankle ref");
    CHECK(NEAR(st.joint_vel[NEXUS_J_L_ANKLE_PITCH], -0.2 * TWO_PI), "ankle vel");
    CHECK(st.act_error[NEXUS_J_L_ANKLE_PITCH] == 0x08000200u, "ankle error");
    CHECK(st.act_state[NEXUS_J_L_ANKLE_PITCH] == 1u, "ankle state");

    /* Node 2 (hip_roll) and 5 (waist) are not on this bench. */
    CHECK(st.joint_pos[NEXUS_J_L_HIP_ROLL] == 0.0f && st.act_state[NEXUS_J_L_HIP_ROLL] == 0u,
          "hip_roll written although no drive is node 2");
    for (int i = NEXUS_J_WAIST_ROLL; i < NEXUS_NUM_JOINTS; i++)
    {
        CHECK(st.joint_pos[i] == 0.0f && st.ref_angle[i] == 0.0f,
              "index %d written although the leg is on bus 0", i);
    }

    CHECK(st.act_flags[NEXUS_J_L_HIP_PITCH] == (NEXUS_ACT_TELEM_FRESH | NEXUS_ACT_HB_FRESH),
          "fresh joint not flagged fresh");
    CHECK(st.act_flags[NEXUS_J_L_ANKLE_PITCH] == 0u, "stale joint flagged fresh");
}

static void test_absent_data_reads_as_absent(void)
{
    printf("what the leg test does not measure is not reported as zero\n");

    leg_stream_joint_t j[3];
    leg_stream_status_t s = status();
    nexus_state_t st;

    bench_leg(j);
    memset(&st, 0xA5, sizeof(st));             /* catch any field left unset */
    leg_stream_fill(&st, j, 3, &s);

    CHECK(st.quat[0] == 1.0f && st.quat[1] == 0.0f && st.quat[2] == 0.0f && st.quat[3] == 0.0f,
          "fused quaternion is not identity");
    CHECK(st.imu_quat[0] == 1.0f && st.imu_quat[3] == 0.0f, "IMU quaternion is not identity");
    CHECK(isnan(st.foot_z[0]) && isnan(st.foot_z[1]), "foot_z is not NaN");
    CHECK(st.fk_valid == 0u, "fk_valid claims a valid foot");
    CHECK(st.fused_valid == NEXUS_FUSION_INVALID, "fused_valid is not INVALID");
    CHECK(st.pelvis_z == 0.0f && st.health == 0u && st.contacts == 0u && st.enc_valid == 0u,
          "estimator/sensor fields not cleared");
    CHECK(st.stream_flags & NEXUS_STREAM_LEG_TEST, "LEG_TEST flag missing");
    CHECK(st.seq == 1234u && st.timestamp_us == 5678u, "seq/timestamp");
}

static void test_status_fields(void)
{
    printf("phase, flags, safety state and counters\n");

    leg_stream_joint_t j[3];
    leg_stream_status_t s = status();
    nexus_state_t st;

    bench_leg(j);
    leg_stream_fill(&st, j, 3, &s);
    CHECK(NEAR(st.phase, 0.25), "phase %f: 2.25 cycles should send 0.25", (double)st.phase);
    CHECK(st.stream_flags == (NEXUS_STREAM_LEG_TEST | NEXUS_STREAM_GAIT_LIVE), "running flags");
    CHECK(st.safety_state == NEXUS_SAFETY_ARMED, "a joint in closed loop should read ARMED");
    CHECK(st.can_dropped[0] == 0xFFFFu && st.can_bus_off[0] == 0xFFu, "counters do not saturate");

    s.gait_running = 0;
    j[0].axis_state = 1;
    j[1].axis_state = 1;
    leg_stream_fill(&st, j, 3, &s);
    CHECK(st.stream_flags == NEXUS_STREAM_LEG_TEST, "GAIT_LIVE set with no gait running");
    CHECK(st.safety_state == NEXUS_SAFETY_IDLE, "no joint in closed loop should read IDLE");

    s.stopped = 1;
    leg_stream_fill(&st, j, 3, &s);
    CHECK(st.safety_state == NEXUS_SAFETY_FAULT, "a stopped test should read FAULT");

    s.gait_phase = (float)NAN;
    leg_stream_fill(&st, j, 3, &s);
    CHECK(st.phase == 0.0f, "NaN phase went out as %f", (double)st.phase);
}

static void test_joints_off_the_bus_are_left_out(void)
{
    printf("a drive that is not live, or has a bad node id, writes nothing\n");

    leg_stream_joint_t j[3];
    leg_stream_status_t s = status();
    nexus_state_t st;

    bench_leg(j);
    j[1].live = 0;                              /* knee unplugged */
    j[2].node = 7;                              /* impossible on a 5-node bus */
    j[2].scale = 0.0f;
    leg_stream_fill(&st, j, 3, &s);

    CHECK(st.joint_pos[NEXUS_J_L_KNEE_PITCH] == 0.0f &&
          st.act_state[NEXUS_J_L_KNEE_PITCH] == 0u, "unplugged knee written");
    for (int i = 0; i < NEXUS_NUM_JOINTS; i++)
    {
        CHECK(isfinite(st.joint_pos[i]) && isfinite(st.ref_angle[i]),
              "index %d is not finite", i);
    }
    CHECK(NEAR(st.joint_pos[NEXUS_J_L_HIP_PITCH], 0.05 * TWO_PI), "hip lost");
}

int main(void)
{
    printf("leg_stream.c host tests\n-----------------------\n");

    test_joints_land_at_their_joint_map_index();
    test_absent_data_reads_as_absent();
    test_status_fields();
    test_joints_off_the_bus_are_left_out();

    printf("-----------------------\n%s (%d failure%s)\n",
           s_fail ? "FAILED" : "PASSED", s_fail, s_fail == 1 ? "" : "s");
    return s_fail ? 1 : 0;
}
