/*
 * Host test for safety.c - the state machine that decides whether a command
 * from the Pi is allowed to reach ten actuators.
 *
 * Everything this file asserts is a rule that, if it silently stopped
 * holding, would not show up as a crash or a log line. It would show up as a
 * robot that kept moving after the thing controlling it stopped existing.
 */

#include "safety.h"
#include "act_odrive.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ---- stubs for the actuator driver ---------------------------------- */

static int     s_disarm_calls;
static int     s_arm_requests;
static uint8_t s_closed_loop = 1;   /* drives are ready unless a test says not */

void act_disarm(void)             { s_disarm_calls++; }
uint8_t act_is_armed(void)        { return 0; }
void act_request_arm(void)        { s_arm_requests++; }
uint8_t act_all_closed_loop(void) { return s_closed_loop; }

/* ---- harness --------------------------------------------------------- */

static int s_fail;

#define CHECK(cond, fmt, ...)                                            \
    do {                                                                 \
        if (!(cond)) {                                                   \
            s_fail++;                                                    \
            printf("  FAIL  " fmt "\n", ##__VA_ARGS__);                  \
        }                                                                \
    } while (0)

static uint32_t s_seq;

static nexus_cmd_t make_cmd(float pos, uint16_t flags)
{
    nexus_cmd_t c;

    memset(&c, 0, sizeof(c));
    c.sync    = NEXUS_SYNC;
    c.msg_id  = NEXUS_MSG_COMMAND;
    c.version = NEXUS_PROTO_VERSION;
    c.seq     = ++s_seq;
    c.flags   = flags;

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        c.target_pos[j] = pos;
    }
    return c;
}

/* Drive the machine to ARMED from a cold start. */
static void arm(float pos)
{
    safety_init();
    s_seq = 0;
    s_disarm_calls = 0;
    s_arm_requests = 0;
    s_closed_loop  = 1;

    safety_tick(0);                       /* BOOT -> IDLE */

    float t[NEXUS_NUM_JOINTS];
    nexus_cmd_t c = make_cmd(pos, NEXUS_CMD_ENABLE);
    (void)safety_accept_command(&c, t);
}

/* ---- tests ----------------------------------------------------------- */

static void test_boot_refuses_commands(void)
{
    printf("BOOT refuses commands until the watched subsystems are healthy\n");

    safety_init();
    s_seq = 0;
    s_closed_loop = 1;

    float t[NEXUS_NUM_JOINTS];
    nexus_cmd_t c = make_cmd(0.0f, NEXUS_CMD_ENABLE);

    CHECK(safety_state() == SAFETY_BOOT, "did not start in BOOT");
    CHECK(safety_accept_command(&c, t) == 0, "accepted a command while in BOOT");

    /*
     * A subsystem that is still coming up holds BOOT. HEALTH_IMU rather than
     * HEALTH_LINK on purpose: a fatal fault would latch FAULT and pull the
     * re-arm handshake into a test that is not about recovery.
     */
    safety_tick(HEALTH_IMU);
    CHECK(safety_state() == SAFETY_BOOT, "left BOOT while a subsystem was faulted");

    c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    CHECK(safety_accept_command(&c, t) == 0,
          "accepted a command before the subsystems were healthy");

    safety_tick(0);
    CHECK(safety_state() == SAFETY_IDLE, "did not reach IDLE once healthy");

    c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    CHECK(safety_accept_command(&c, t) == 1, "refused a good command from IDLE");
    CHECK(safety_state() == SAFETY_ARMED, "did not arm on a good command");
}

/* The whole reason this file exists. */
static void test_link_loss_disarms(void)
{
    printf("a lost link takes the actuators away\n");

    arm(0.0f);
    CHECK(safety_state() == SAFETY_ARMED, "setup: not armed");
    s_disarm_calls = 0;

    safety_tick(HEALTH_LINK);

    CHECK(safety_state() == SAFETY_FAULT, "did not fault on HEALTH_LINK");
    CHECK(s_disarm_calls == 1, "act_disarm() called %d times, expected 1",
          s_disarm_calls);

    /* Repeated ticks in fault must not re-issue the disarm every millisecond. */
    safety_tick(HEALTH_LINK);
    safety_tick(HEALTH_LINK);
    CHECK(s_disarm_calls == 1, "re-disarmed while already faulted (%d calls)",
          s_disarm_calls);
}

static void test_missed_deadline_disarms(void)
{
    printf("a missed control deadline takes the actuators away\n");

    arm(0.0f);
    s_disarm_calls = 0;

    safety_tick(HEALTH_TIMING);
    CHECK(safety_state() == SAFETY_FAULT, "did not fault on HEALTH_TIMING");
    CHECK(s_disarm_calls == 1, "did not disarm on HEALTH_TIMING");
}

/* A degraded estimate is the Pi's problem to handle; it must not idle the
   robot on its own, or a single dropped IMU sample stops the machine. */
static void test_sensor_faults_do_not_disarm(void)
{
    printf("a degraded sensor does not idle the robot by itself\n");

    arm(0.0f);
    s_disarm_calls = 0;

    safety_tick(HEALTH_IMU | HEALTH_ENC | HEALTH_CAN1);

    CHECK(safety_state() == SAFETY_ARMED, "faulted on a non-fatal health bit");
    CHECK(s_disarm_calls == 0, "disarmed on a non-fatal health bit");
}

static void test_fault_requires_explicit_rearm(void)
{
    printf("recovery needs an ENABLE-off handshake, not just a clean link\n");

    arm(0.0f);
    safety_tick(HEALTH_LINK);
    CHECK(safety_state() == SAFETY_FAULT, "setup: not faulted");

    safety_tick(0);
    CHECK(safety_state() == SAFETY_IDLE, "did not return to IDLE when clean");

    float t[NEXUS_NUM_JOINTS];

    /* Still enabled: must NOT come straight back under the Pi's control. */
    nexus_cmd_t c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    CHECK(safety_accept_command(&c, t) == 0, "re-armed without a handshake");
    CHECK(safety_state() != SAFETY_ARMED, "re-armed without a handshake");

    /* The Pi drops ENABLE, acknowledging it lost control... */
    c = make_cmd(0.0f, 0);
    (void)safety_accept_command(&c, t);

    /* ...and only now may it take the robot back. */
    c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    CHECK(safety_accept_command(&c, t) == 1, "refused a proper re-arm");
    CHECK(safety_state() == SAFETY_ARMED, "did not arm after the handshake");
}

static void test_enable_off_stands_down(void)
{
    printf("ENABLE off stands the actuators down\n");

    arm(0.0f);
    s_disarm_calls = 0;

    float t[NEXUS_NUM_JOINTS];
    nexus_cmd_t c = make_cmd(0.0f, 0);

    CHECK(safety_accept_command(&c, t) == 0, "acted on a disabled command");
    CHECK(safety_state() == SAFETY_IDLE, "did not stand down on ENABLE off");
    CHECK(s_disarm_calls == 1, "did not disarm on ENABLE off");
}

static void test_rejects_nan_and_out_of_range(void)
{
    printf("NaN and out-of-range targets are refused\n");

    float t[NEXUS_NUM_JOINTS];

    arm(0.0f);
    uint32_t before = safety_rejected();

    nexus_cmd_t c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    c.target_pos[3] = NAN;
    CHECK(safety_accept_command(&c, t) == 0, "accepted a NaN target");

    c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    c.target_pos[7] = INFINITY;
    CHECK(safety_accept_command(&c, t) == 0, "accepted an infinite target");

    c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    c.target_pos[0] = SAFETY_POS_MAX_TURNS + 1.0f;
    CHECK(safety_accept_command(&c, t) == 0, "accepted an over-range target");

    c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    c.target_pos[0] = SAFETY_POS_MIN_TURNS - 1.0f;
    CHECK(safety_accept_command(&c, t) == 0, "accepted an under-range target");

    CHECK(safety_rejected() == before + 4,
          "rejection counter = %u, expected %u",
          safety_rejected(), before + 4);
}

static void test_slew_limit(void)
{
    printf("a target that jumps too far in one command is refused\n");

    float t[NEXUS_NUM_JOINTS];

    arm(0.0f);

    nexus_cmd_t c = make_cmd(SAFETY_MAX_STEP_TURNS * 0.5f, NEXUS_CMD_ENABLE);
    CHECK(safety_accept_command(&c, t) == 1, "refused a legal step");

    c = make_cmd(SAFETY_MAX_STEP_TURNS * 0.5f + SAFETY_MAX_STEP_TURNS + 0.1f,
                 NEXUS_CMD_ENABLE);
    CHECK(safety_accept_command(&c, t) == 0, "accepted an oversized step");

    /* The refused command must not have moved the reference. A rejected
       frame that still updates s_last_pos would let a policy walk the robot
       anywhere one rejected step at a time. */
    c = make_cmd(SAFETY_MAX_STEP_TURNS * 0.5f, NEXUS_CMD_ENABLE);
    CHECK(safety_accept_command(&c, t) == 1,
          "a rejected command moved the slew reference");
}

static void test_replayed_and_stale_seq(void)
{
    printf("replayed and out-of-order frames are refused\n");

    float t[NEXUS_NUM_JOINTS];

    arm(0.0f);

    nexus_cmd_t good = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    CHECK(safety_accept_command(&good, t) == 1, "refused a fresh command");

    /* Exact replay. */
    CHECK(safety_accept_command(&good, t) == 0, "accepted a replayed frame");

    /* Older sequence number. */
    nexus_cmd_t old = good;
    old.seq = good.seq - 5u;
    CHECK(safety_accept_command(&old, t) == 0, "accepted a stale frame");

    /* Sequence wrap must still read as forward motion, not as stale. */
    safety_init();
    s_seq = 0;
    s_closed_loop = 1;
    safety_tick(0);

    nexus_cmd_t a = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    a.seq = 0xFFFFFFFEu;
    CHECK(safety_accept_command(&a, t) == 1, "setup: refused first command");

    nexus_cmd_t b = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    b.seq = 1u;                    /* wrapped past 0xFFFFFFFF */
    CHECK(safety_accept_command(&b, t) == 1,
          "treated a wrapped sequence number as stale");
}

static void test_sustained_garbage_faults(void)
{
    printf("a sustained run of bad commands faults the link\n");

    float t[NEXUS_NUM_JOINTS];

    arm(0.0f);
    s_disarm_calls = 0;

    for (unsigned i = 0; i < SAFETY_MAX_REJECTS; i++)
    {
        nexus_cmd_t c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
        c.target_pos[0] = NAN;
        (void)safety_accept_command(&c, t);
    }

    CHECK(safety_state() == SAFETY_FAULT,
          "still %s after %u rejected commands",
          safety_state_name(), SAFETY_MAX_REJECTS);
    CHECK(s_disarm_calls == 1, "did not disarm after sustained garbage");
}

static void test_good_command_passes_through_intact(void)
{
    printf("an accepted command reaches the caller unmodified\n");

    float t[NEXUS_NUM_JOINTS];

    safety_init();
    s_seq = 0;
    s_closed_loop = 1;
    safety_tick(0);

    nexus_cmd_t c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        c.target_pos[j] = 0.01f * (float)j;
    }

    CHECK(safety_accept_command(&c, t) == 1, "refused a good command");

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        CHECK(t[j] == 0.01f * (float)j,
              "joint %d came through as %f, expected %f",
              j, (double)t[j], (double)(0.01f * (float)j));
    }
}

/*
 * The firmware used to send position commands to drives that might be sitting
 * in IDLE, and never notice. Nothing moved, and nothing said why.
 */
static void test_arms_the_drives_before_commanding(void)
{
    printf("drives are put into closed loop before any command is acted on\n");

    safety_init();
    s_seq = 0;
    s_arm_requests = 0;
    s_disarm_calls = 0;
    s_closed_loop  = 0;              /* drives are idle */
    safety_tick(0);

    float t[NEXUS_NUM_JOINTS];
    nexus_cmd_t c = make_cmd(0.0f, NEXUS_CMD_ENABLE);

    CHECK(safety_accept_command(&c, t) == 0,
          "commanded actuators that were not in closed loop");
    CHECK(s_arm_requests == 1, "did not ask the drives to arm");
    CHECK(safety_state() != SAFETY_ARMED, "reported ARMED with idle drives");

    /* Drives arrive in closed loop; the next command goes through. */
    s_closed_loop = 1;
    c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
    CHECK(safety_accept_command(&c, t) == 1, "refused a command once armed");
    CHECK(safety_state() == SAFETY_ARMED, "did not reach ARMED");
}

static void test_drives_that_never_arm_fault(void)
{
    printf("drives that never reach closed loop fault rather than hang\n");

    safety_init();
    s_seq = 0;
    s_arm_requests = 0;
    s_disarm_calls = 0;
    s_closed_loop  = 0;
    safety_tick(0);

    float t[NEXUS_NUM_JOINTS];

    for (unsigned i = 0; i < SAFETY_ARM_TIMEOUT_CMDS; i++)
    {
        nexus_cmd_t c = make_cmd(0.0f, NEXUS_CMD_ENABLE);
        (void)safety_accept_command(&c, t);
    }

    CHECK(safety_state() == SAFETY_FAULT,
          "still %s after %u commands with the drives refusing to arm",
          safety_state_name(), SAFETY_ARM_TIMEOUT_CMDS);
    CHECK(s_disarm_calls == 1, "did not disarm after failing to arm");
}

int main(void)
{
    printf("safety.c host tests\n-------------------\n");

    test_boot_refuses_commands();
    test_link_loss_disarms();
    test_missed_deadline_disarms();
    test_sensor_faults_do_not_disarm();
    test_fault_requires_explicit_rearm();
    test_enable_off_stands_down();
    test_rejects_nan_and_out_of_range();
    test_slew_limit();
    test_replayed_and_stale_seq();
    test_sustained_garbage_faults();
    test_good_command_passes_through_intact();
    test_arms_the_drives_before_commanding();
    test_drives_that_never_arm_fault();

    printf("-------------------\n%s (%d failure%s)\n",
           s_fail ? "FAILED" : "PASSED", s_fail, s_fail == 1 ? "" : "s");

    return s_fail ? 1 : 0;
}
