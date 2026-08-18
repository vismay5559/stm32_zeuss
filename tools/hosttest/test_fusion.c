/*
 * Host test for fusion.c - the bridge between the sensors and the estimator.
 *
 * fusion.c, inekf.c, lie_group.c and kinematics.c touch no hardware at all:
 * they take structs in and produce a state estimate. That makes them the part
 * of this firmware most worth testing on a workstation, and until now the only
 * way to run a line of any of it was to flash a robot.
 *
 * These tests are about the RULES fusion.c enforces, not about whether the
 * filter's numbers are right - that needs recorded sensor traces and a
 * measured robot.
 */

#include "fusion.h"
#include "robot_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int s_fail;

#define CHECK(cond, fmt, ...)                                            \
    do {                                                                 \
        if (!(cond)) {                                                   \
            s_fail++;                                                    \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__);                  \
        }                                                                \
    } while (0)

/* ---- fixtures -------------------------------------------------------- */

static imu_sample_t    s_imu;
static act_telemetry_t s_act;
static uint32_t        s_now_us;

static void fixtures_reset(void)
{
    memset(&s_imu, 0, sizeof(s_imu));
    memset(&s_act, 0, sizeof(s_act));
    s_now_us = 0;

    s_imu.quat[0] = 1.0f;
    s_imu.accel[2] = 9.81f;     /* at rest: the accelerometer reads +g up */

    /* Every joint fresh and fault-free. */
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_act.pos_age[j] = 0u;
    }
}

/* One 1 kHz tick. `inertial` says whether a new gyro+accel pair arrived. */
static void tick(uint8_t inertial, uint8_t contacts)
{
    s_now_us += 1000u;

    if (inertial)
    {
        s_imu.accel_seq++;
        s_imu.gyro_seq++;
        s_imu.seq++;
    }

    fusion_tick(&s_imu, &s_act, contacts, s_now_us);
}

static void age_all_joints(uint16_t by)
{
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_act.pos_age[j] = by;
    }
}

/* ---- tests ----------------------------------------------------------- */

/*
 * The rotation vector runs at 100 Hz, independent of the gyro and
 * accelerometer at 400 Hz. Keying the prediction off the shared imu->seq
 * propagated on quaternion-only frames, integrating the same inertial sample
 * a second time over a fresh dt.
 */
static void test_quat_only_frames_do_not_propagate(void)
{
    printf("a quaternion-only IMU frame does not advance the estimate\n");

    fixtures_reset();
    fusion_init();

    /* Two real inertial samples, so the filter has a dt to work from. */
    s_imu.accel[0] = 2.0f;              /* a clear horizontal acceleration */
    tick(1, 0);
    tick(1, 0);

    nexus_state_t before;
    memset(&before, 0, sizeof(before));
    fusion_fill_state(&before);

    /* Now ten frames where ONLY the rotation vector updated. */
    for (int i = 0; i < 10; i++)
    {
        s_imu.quat_seq++;
        s_imu.seq++;                    /* the shared counter moves too */
        s_now_us += 1000u;
        fusion_tick(&s_imu, &s_act, 0u, s_now_us);
    }

    nexus_state_t after;
    memset(&after, 0, sizeof(after));
    fusion_fill_state(&after);

    for (int i = 0; i < 3; i++)
    {
        CHECK(before.fused_vel[i] == after.fused_vel[i],
              "velocity[%d] moved from %f to %f on quaternion-only frames",
              i, (double)before.fused_vel[i], (double)after.fused_vel[i]);
    }
}

static void test_inertial_frames_do_propagate(void)
{
    printf("a real gyro+accel sample does advance the estimate\n");

    fixtures_reset();
    fusion_init();

    s_imu.accel[0] = 2.0f;
    tick(1, 0);
    tick(1, 0);

    nexus_state_t before;
    memset(&before, 0, sizeof(before));
    fusion_fill_state(&before);

    for (int i = 0; i < 10; i++)
    {
        tick(1, 0);
    }

    nexus_state_t after;
    memset(&after, 0, sizeof(after));
    fusion_fill_state(&after);

    CHECK(before.fused_vel[0] != after.fused_vel[0],
          "velocity did not move under a sustained 2 m/s^2 acceleration");
}

/*
 * The estimator used to take hip_pitch and knee from the spring encoders,
 * which measure deflection rather than joint angle. It now takes all four
 * joint angles from the drives - so moving a drive must move the foot, and
 * the encoders must have no say in it at all.
 */
static void test_foot_follows_the_drives(void)
{
    printf("foot height follows the drives, and only the drives\n");

    fixtures_reset();
    fusion_init();
    tick(1, 0);

    nexus_state_t a;
    memset(&a, 0, sizeof(a));
    fusion_fill_state(&a);

    /* Bend every joint on the right leg by a quarter turn. */
    for (int j = 5; j < 9; j++)
    {
        s_act.pos[j] = 0.25f;
    }
    tick(1, 0);

    nexus_state_t b;
    memset(&b, 0, sizeof(b));
    fusion_fill_state(&b);

    CHECK(a.foot_z[0] != b.foot_z[0],
          "right foot height did not change when its drives moved (%f -> %f)",
          (double)a.foot_z[0], (double)b.foot_z[0]);
}

/*
 * A leg whose drives have gone quiet must not produce a foot height at all.
 * It used to be reported as 0.0, which to a gait policy reads as "this foot is
 * exactly on the ground".
 */
static void test_stale_leg_reports_nan_not_zero(void)
{
    printf("a leg with stale telemetry reports NaN, not 0.0\n");

    fixtures_reset();
    fusion_init();
    tick(1, 0);

    nexus_state_t good;
    memset(&good, 0, sizeof(good));
    fusion_fill_state(&good);

    CHECK(good.fk_valid == 0x3u,
          "fk_valid = 0x%02X with fresh telemetry, expected both feet valid",
          good.fk_valid);
    CHECK(!isnan(good.foot_z[0]) && !isnan(good.foot_z[1]),
          "a valid foot height came out NaN");

    /* Both buses go quiet. */
    age_all_joints((uint16_t)(ACT_POS_STALE_TICKS + 1u));
    tick(1, 0);

    nexus_state_t stale;
    memset(&stale, 0, sizeof(stale));
    fusion_fill_state(&stale);

    CHECK(stale.fk_valid == 0u,
          "fk_valid = 0x%02X with stale telemetry, expected 0", stale.fk_valid);
    CHECK(isnan(stale.foot_z[0]) && isnan(stale.foot_z[1]),
          "stale foot heights read %f / %f, expected NaN",
          (double)stale.foot_z[0], (double)stale.foot_z[1]);
    CHECK(stale.foot_z[0] != 0.0f,
          "a stale foot height read exactly 0.0 - indistinguishable from "
          "a foot resting on the ground");
}

/* An axis reporting a fault is not reporting a trustworthy angle either. */
static void test_faulted_axis_invalidates_its_leg(void)
{
    printf("an axis in a fault state invalidates its leg\n");

    fixtures_reset();
    fusion_init();
    tick(1, 0);

    s_act.axis_error[g_leg_joints[1][ROBOT_JOINT_KNEE].act_index] = 0x20u;
    tick(1, 0);

    nexus_state_t st;
    memset(&st, 0, sizeof(st));
    fusion_fill_state(&st);

    CHECK((st.fk_valid & NEXUS_FK_RIGHT_VALID) == 0u,
          "right leg still valid with a faulted knee axis");
    CHECK((st.fk_valid & NEXUS_FK_LEFT_VALID) != 0u,
          "left leg was invalidated by a fault on the right");
}

/*
 * The whole point of the calibration gate: a filter can converge beautifully
 * onto geometry that does not match the robot, and the Pi cannot tell the
 * difference from the covariance alone.
 */
static void test_never_reports_ok_while_uncalibrated(void)
{
    printf("status never reaches OK while the robot is uncalibrated\n");

    fixtures_reset();
    fusion_init();

    /* Stand on both feet and run for well past the convergence hold time. */
    uint8_t both = (uint8_t)(NEXUS_CONTACT_L_FOOT | NEXUS_CONTACT_R_FOOT);

    for (int i = 0; i < 3000; i++)
    {
        /* Refresh telemetry so the legs stay usable and each tick counts as a
           new measurement. */
        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            s_act.pos_age[j] = 0u;
        }
        tick(1, both);
    }

    uint8_t st = fusion_status();

    if (robot_config_is_calibrated())
    {
        printf("  note: robot_config.h is marked calibrated, so OK is allowed\n");
        return;
    }

    CHECK(st != NEXUS_FUSION_OK,
          "reported FUSION_OK after %u converged ticks while robot_config.h "
          "still says the robot has not been measured",
          fusion_converged_ticks());
    CHECK(st == NEXUS_FUSION_CONVERGING,
          "expected CONVERGING, got %u", st);
}

/* No NaN may ever reach the packet from a healthy run. */
static void test_no_nan_in_a_healthy_run(void)
{
    printf("a healthy run produces no NaN in the estimate\n");

    fixtures_reset();
    fusion_init();

    uint8_t both = (uint8_t)(NEXUS_CONTACT_L_FOOT | NEXUS_CONTACT_R_FOOT);

    for (int i = 0; i < 1000; i++)
    {
        s_imu.gyro[2] = 0.1f;
        for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
        {
            s_act.pos_age[j] = 0u;
        }
        tick(1, both);
    }

    nexus_state_t st;
    memset(&st, 0, sizeof(st));
    fusion_fill_state(&st);

    CHECK(!isnan(st.pelvis_z), "pelvis_z is NaN");
    for (int i = 0; i < 3; i++)
    {
        CHECK(!isnan(st.fused_vel[i]), "fused_vel[%d] is NaN", i);
        CHECK(!isnan(st.vel_hdg[i]), "vel_hdg[%d] is NaN", i);
    }
    for (int i = 0; i < 4; i++)
    {
        CHECK(!isnan(st.quat[i]), "quat[%d] is NaN", i);
    }
}

int main(void)
{
    printf("fusion.c host tests\n-------------------\n");

    test_quat_only_frames_do_not_propagate();
    test_inertial_frames_do_propagate();
    test_foot_follows_the_drives();
    test_stale_leg_reports_nan_not_zero();
    test_faulted_axis_invalidates_its_leg();
    test_never_reports_ok_while_uncalibrated();
    test_no_nan_in_a_healthy_run();

    printf("-------------------\n%s (%d failure%s)\n",
           s_fail ? "FAILED" : "PASSED", s_fail, s_fail == 1 ? "" : "s");

    return s_fail ? 1 : 0;
}
