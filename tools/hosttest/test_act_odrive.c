/*
 * Host test for act_odrive.c - the code that actually moves the robot.
 *
 * Everything else in the App layer decides what the legs should do. This is
 * the file that tells them, so a mistake here is a mistake that reaches the
 * motors: a joint commanded to jump to a new position instead of easing into
 * it, a speed hint that wraps from "fast forwards" to "fast backwards", a
 * stop request that is sent once and then forgotten, or a command quietly
 * dropped because a queue filled up and nobody noticed.
 *
 * None of that is visible by reading. It is visible here because the two CAN
 * wires are simulated (tools/hosttest/stub/fdcan.h), so a test can look at
 * every single message the firmware puts on the wire, and can make the wire
 * misbehave on demand - block it, refuse a message, report errors - without
 * owning a robot or a bench.
 *
 * Vocabulary, since the CAN wire has its own:
 *
 *   bus       one of the two wires. Joints 0-4 are on bus 0, joints 5-9 on
 *             bus 1.
 *   node      which motor on that wire, 1 to 5.
 *   frame     one message. Its address ("identifier") packs the node and the
 *             kind of message together: (node << 5) | command.
 *   turns     the unit ODrive counts position in. One turn is one revolution.
 *
 * A note on ORDER: the tests below run in the order main() lists them, and
 * the first one has to be first. It checks what happens BEFORE the motors
 * have been set up, and there is no way to un-set-up them afterwards.
 */

#include "act_odrive.h"
#include "fdcan.h"
#include "link_proto.h"

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

#define NEAR(a, b, tol)  (fabsf((a) - (b)) <= (tol))

/* The command numbers, spelled the same way the ODrive manual does. */
#define CMD_HEARTBEAT     0x001u
#define CMD_SET_STATE     0x007u
#define CMD_GET_ENCODER   0x009u
#define CMD_SET_INPUT_POS 0x00Cu
#define CMD_CLEAR_ERRORS  0x018u
#define CMD_GET_TORQUES   0x01Cu

#define NODES_PER_BUS  5

/* ---- reading and writing the eight bytes of a frame ------------------ */

static void put_f32(uint8_t *p, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    p[0] = (uint8_t)(bits & 0xFFu);
    p[1] = (uint8_t)((bits >> 8) & 0xFFu);
    p[2] = (uint8_t)((bits >> 16) & 0xFFu);
    p[3] = (uint8_t)((bits >> 24) & 0xFFu);
}

static float get_f32(const uint8_t *p)
{
    uint32_t bits = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

static int16_t get_i16(const uint8_t *p)
{
    uint16_t bits = (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
    int16_t  v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

/* ---- where a joint lives, and what its frames are addressed to ------- */

static uint8_t bus_of(int joint)
{
    return (uint8_t)(joint / NODES_PER_BUS);
}

static uint32_t id_of(int joint, uint32_t cmd)
{
    uint32_t node = (uint32_t)(joint % NODES_PER_BUS) + 1u;
    return (node << 5) | cmd;
}

/* ---- looking at what the firmware sent ------------------------------- */

static const host_can_frame_t *sent_to(int joint, uint32_t cmd)
{
    uint8_t  bus = bus_of(joint);
    uint32_t id  = id_of(joint, cmd);

    for (uint32_t i = 0; i < host_can_sent_count(bus); i++)
    {
        const host_can_frame_t *f = host_can_sent(bus, i);
        if (f->identifier == id)
        {
            return f;
        }
    }
    return NULL;
}

static uint32_t count_of_cmd(uint32_t cmd)
{
    uint32_t n = 0;

    for (uint8_t bus = 0; bus < 2u; bus++)
    {
        for (uint32_t i = 0; i < host_can_sent_count(bus); i++)
        {
            if ((host_can_sent(bus, i)->identifier & 0x1Fu) == cmd)
            {
                n++;
            }
        }
    }
    return n;
}

static uint32_t total_sent(void)
{
    return host_can_sent_count(0) + host_can_sent_count(1);
}

/* ---- pretending to be a motor drive ---------------------------------- */

static void drive_reports_position(int joint, float pos, float vel)
{
    uint8_t d[8];

    put_f32(&d[0], pos);
    put_f32(&d[4], vel);
    host_can_deliver(bus_of(joint), id_of(joint, CMD_GET_ENCODER), d);
    act_on_rx(bus_of(joint));
}

static void drive_reports_state(int joint, uint32_t error, uint8_t state)
{
    uint8_t d[8] = { 0 };

    d[0] = (uint8_t)(error & 0xFFu);
    d[1] = (uint8_t)((error >> 8) & 0xFFu);
    d[2] = (uint8_t)((error >> 16) & 0xFFu);
    d[3] = (uint8_t)((error >> 24) & 0xFFu);
    d[4] = state;
    host_can_deliver(bus_of(joint), id_of(joint, CMD_HEARTBEAT), d);
    act_on_rx(bus_of(joint));
}

static void all_drives_report_running(void)
{
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        drive_reports_state(j, 0u, (uint8_t)ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
}

/* ---- setup shared by most tests -------------------------------------- */

static void fresh_board(void)
{
    host_can_reset();
    act_init();
    host_can_forget_sent();
}

static void set_all_targets(float v)
{
    float t[NEXUS_NUM_JOINTS];

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        t[j] = v;
    }
    act_set_targets(t);
}

static void ticks(int n)
{
    for (int i = 0; i < n; i++)
    {
        act_tick_1khz();
    }
}

/* =====================================================================
 * MUST RUN FIRST - see the note at the top of the file.
 * ===================================================================== */

static void test_the_emergency_stop_is_safe_before_startup(void)
{
    printf("emergency stop before the motors are set up\n");

    host_can_reset();

    /*
     * A failure early in startup - a clock that would not configure, a board
     * check that failed - reaches the emergency handler while the CAN
     * hardware is still untouched. Talking to it then would turn a reported
     * problem into a crash inside the very code meant to handle it.
     */
    act_emergency_idle();

    CHECK(total_sent() == 0u,
          "it put %u frames on the wire before the hardware existed",
          total_sent());
}

static void test_startup_wakes_both_wires_and_commands_nothing(void)
{
    printf("starting up\n");

    host_can_reset();
    act_init();

    CHECK(host_can_start_count(0) == 1u && host_can_start_count(1) == 1u,
          "wires started %u and %u times, expected 1 each",
          host_can_start_count(0), host_can_start_count(1));

    /* Setting up the wires must not, by itself, ask a motor to do anything. */
    CHECK(total_sent() == 0u,
          "startup sent %u frames; it should send none", total_sent());

    CHECK(act_is_armed() == 0u, "it came up already switched on for movement");

    /* Nothing has been heard from any joint yet, and "unknown" has to read as
       old rather than as a fresh zero - the estimator trusts fresh readings. */
    act_telemetry_t t;
    act_get(&t);
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        CHECK(t.pos_age[j] == 0xFFFFu,
              "joint %d's reading came up looking %u ticks old, expected the "
              "maximum", j, t.pos_age[j]);
    }
}

static void test_the_first_move_starts_from_where_the_leg_is(void)
{
    printf("the first move after a stop\n");

    fresh_board();

    /* Joint 0 is where it is; joint 1 has not reported at all. */
    drive_reports_position(0, 1.0f, 0.0f);

    set_all_targets(2.0f);
    act_tick_1khz();

    /*
     * Joint 0 knows where it is, so the first millisecond of the move should
     * be a quarter of the way from THERE (1.0) to the target (2.0). Starting
     * from the target instead would tell the drive "be somewhere else, now",
     * and it would go there as fast as it physically can.
     */
    const host_can_frame_t *f0 = sent_to(0, CMD_SET_INPUT_POS);
    CHECK(f0 != NULL, "joint 0 was sent no position command at all");
    if (f0)
    {
        CHECK(NEAR(get_f32(f0->data), 1.25f, 1e-6f),
              "joint 0's first command was %.4f turns, expected 1.25 - a "
              "quarter of the way from where it is",
              (double)get_f32(f0->data));
    }

    /*
     * Joint 1 has never reported. There is nothing to start from, so it
     * starts from the target - which is the abrupt move again, but from the
     * best guess available rather than from a stale or zero reading.
     */
    const host_can_frame_t *f1 = sent_to(1, CMD_SET_INPUT_POS);
    CHECK(f1 != NULL, "joint 1 was sent no position command at all");
    if (f1)
    {
        CHECK(NEAR(get_f32(f1->data), 2.0f, 1e-6f),
              "joint 1, with no reading of its own, was commanded to %.4f; "
              "expected the target itself, 2.0",
              (double)get_f32(f1->data));
    }
}

static void test_an_old_or_nonsense_reading_is_not_used_as_a_start(void)
{
    printf("what counts as knowing where a leg is\n");

    /* Exactly at the limit still counts. */
    fresh_board();
    drive_reports_position(0, 1.0f, 0.0f);
    ticks((int)ACT_POS_STALE_TICKS);          /* reading is now 100 ticks old */
    host_can_forget_sent();
    set_all_targets(2.0f);
    act_tick_1khz();

    const host_can_frame_t *f = sent_to(0, CMD_SET_INPUT_POS);
    CHECK(f && NEAR(get_f32(f->data), 1.25f, 1e-6f),
          "a reading exactly %u ticks old was refused; it should still count",
          ACT_POS_STALE_TICKS);

    /* One tick older does not. */
    fresh_board();
    drive_reports_position(0, 1.0f, 0.0f);
    ticks((int)ACT_POS_STALE_TICKS + 1);
    host_can_forget_sent();
    set_all_targets(2.0f);
    act_tick_1khz();

    f = sent_to(0, CMD_SET_INPUT_POS);
    CHECK(f && NEAR(get_f32(f->data), 2.0f, 1e-6f),
          "a reading %u ticks old was still used as a starting point",
          ACT_POS_STALE_TICKS + 1u);

    /*
     * And a reading that is fresh but not a number. A drive with a broken
     * encoder can report one, and it would poison every position derived
     * from it - so it is refused the same way an old one is.
     */
    fresh_board();
    drive_reports_position(0, NAN, 0.0f);
    host_can_forget_sent();
    set_all_targets(2.0f);
    act_tick_1khz();

    f = sent_to(0, CMD_SET_INPUT_POS);
    CHECK(f != NULL, "joint 0 was sent no position command at all");
    if (f)
    {
        CHECK(isfinite(get_f32(f->data)),
              "a nonsense reading was used as a starting point and the "
              "command sent to the drive was nonsense too");
        CHECK(NEAR(get_f32(f->data), 2.0f, 1e-6f),
              "expected the target, 2.0; got %.4f",
              (double)get_f32(f->data));
    }
}

static void test_a_move_is_spread_over_four_ticks_and_then_stops(void)
{
    printf("easing into a new target\n");

    fresh_board();
    drive_reports_position(0, 0.0f, 0.0f);
    host_can_forget_sent();

    set_all_targets(1.0f);

    /*
     * Commands arrive from the Pi at 250 Hz and the motors are addressed at
     * 1 kHz, so each command is spread across the four ticks that follow it.
     * A quarter, a half, three quarters, then there.
     */
    const float expected[4] = { 0.25f, 0.5f, 0.75f, 1.0f };

    for (int i = 0; i < 4; i++)
    {
        host_can_forget_sent();
        act_tick_1khz();

        const host_can_frame_t *f = sent_to(0, CMD_SET_INPUT_POS);
        CHECK(f != NULL, "no command on tick %d", i + 1);
        if (f)
        {
            CHECK(NEAR(get_f32(f->data), expected[i], 1e-6f),
                  "tick %d commanded %.4f turns, expected %.4f",
                  i + 1, (double)get_f32(f->data), (double)expected[i]);
        }
    }

    /*
     * With no new command the leg holds where it got to. It does NOT keep
     * extrapolating past the target, which is what a ramp that ran off the
     * end of its segment would do.
     */
    for (int i = 0; i < 20; i++)
    {
        host_can_forget_sent();
        act_tick_1khz();

        const host_can_frame_t *f = sent_to(0, CMD_SET_INPUT_POS);
        CHECK(f && NEAR(get_f32(f->data), 1.0f, 1e-6f),
              "%d ticks after arriving, the leg was commanded to %.4f instead "
              "of holding at 1.0", i + 1, f ? (double)get_f32(f->data) : 0.0);
        CHECK(f && get_i16(&f->data[4]) == 0,
              "a leg that has arrived was still being told it is moving");
    }
}

static void test_a_fast_move_reports_a_capped_speed_not_a_reversed_one(void)
{
    printf("the speed hint on a large move\n");

    /*
     * Alongside each position the firmware sends a speed hint, packed by
     * ODrive into a small whole number of thousandths of a turn per second.
     * It cannot hold more than about 32.7 turns/s.
     *
     * A large move easily exceeds that, and simply casting the number into
     * the small field would not clamp it - it would WRAP, turning a fast
     * forward move into a fast backward one. The drive would then be told to
     * go the wrong way at speed while the position it is given says
     * otherwise. This is the check that the clamp exists.
     */
    fresh_board();
    drive_reports_position(0, 0.0f, 0.0f);
    host_can_forget_sent();

    set_all_targets(1.0f);          /* a quarter turn per tick = 250 turns/s */
    act_tick_1khz();

    const host_can_frame_t *f = sent_to(0, CMD_SET_INPUT_POS);
    CHECK(f != NULL, "no command sent");
    if (f)
    {
        CHECK(get_i16(&f->data[4]) == 32767,
              "a fast forward move sent a speed hint of %d; expected the "
              "maximum, 32767", get_i16(&f->data[4]));
    }

    /* And the same in the other direction. */
    fresh_board();
    drive_reports_position(0, 0.0f, 0.0f);
    host_can_forget_sent();

    set_all_targets(-1.0f);
    act_tick_1khz();

    f = sent_to(0, CMD_SET_INPUT_POS);
    CHECK(f != NULL, "no command sent");
    if (f)
    {
        CHECK(get_i16(&f->data[4]) == -32768,
              "a fast backward move sent a speed hint of %d; expected the "
              "minimum, -32768", get_i16(&f->data[4]));
    }
}

static void test_stopping_goes_limp_and_stays_limp(void)
{
    printf("stopping\n");

    fresh_board();
    set_all_targets(1.0f);
    act_tick_1khz();
    CHECK(act_is_armed() != 0u, "it did not report itself as switched on");

    host_can_forget_sent();
    act_disarm();

    CHECK(act_is_armed() == 0u, "it still reports itself as switched on");

    /* Every one of the ten joints is asked to go limp. */
    CHECK(count_of_cmd(CMD_SET_STATE) == (uint32_t)NEXUS_NUM_JOINTS,
          "%u joints were told to stop, expected %d",
          count_of_cmd(CMD_SET_STATE), NEXUS_NUM_JOINTS);

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        const host_can_frame_t *f = sent_to(j, CMD_SET_STATE);
        CHECK(f != NULL, "joint %d was not told to stop", j);
        if (f)
        {
            CHECK(f->data[0] == (uint8_t)ODRV_AXIS_STATE_IDLE,
                  "joint %d was told to go to state %u, expected %u (limp)",
                  j, f->data[0], ODRV_AXIS_STATE_IDLE);
        }
    }

    /* No position commands afterwards. This is the part that matters: the
       request to go limp is advice, but silence is what actually stops it. */
    host_can_forget_sent();
    ticks(99);
    CHECK(total_sent() == 0u,
          "after stopping, %u frames were still being sent", total_sent());

    /*
     * A single request can be lost, and a drive that reboots comes back in
     * whatever state it was configured for - so a standing stop is repeated,
     * slowly. Every 100 ticks, not every tick: a wire that is already in
     * trouble does not need ten more frames a millisecond.
     */
    act_tick_1khz();
    CHECK(count_of_cmd(CMD_SET_STATE) == (uint32_t)NEXUS_NUM_JOINTS,
          "the standing stop repeated %u requests on tick 100, expected %d",
          count_of_cmd(CMD_SET_STATE), NEXUS_NUM_JOINTS);
}

static void test_switching_on_clears_only_real_errors(void)
{
    printf("switching the motors on\n");

    fresh_board();

    /* Joint 3 has tripped. Joint 7 is already running. Everyone else is
       simply sitting there. */
    drive_reports_state(3, 0x20u, (uint8_t)ODRV_AXIS_STATE_IDLE);
    drive_reports_state(7, 0u, (uint8_t)ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL);
    host_can_forget_sent();

    act_request_arm();

    /*
     * Only the joint that actually tripped is asked to forget its error. An
     * error that clears itself everywhere, every time, is an error nobody
     * ever investigates.
     */
    CHECK(count_of_cmd(CMD_CLEAR_ERRORS) == 1u,
          "%u joints were asked to clear errors, expected only the one that "
          "tripped", count_of_cmd(CMD_CLEAR_ERRORS));
    CHECK(sent_to(3, CMD_CLEAR_ERRORS) != NULL,
          "the joint that tripped was not asked to clear its error");

    /*
     * Joint 7 already reported itself running, so it is left alone.
     * Re-commanding it is not free, and it would fight a drive that is
     * halfway through starting.
     */
    CHECK(count_of_cmd(CMD_SET_STATE) == (uint32_t)(NEXUS_NUM_JOINTS - 1),
          "%u joints were told to start, expected %d - the one already "
          "running should have been skipped",
          count_of_cmd(CMD_SET_STATE), NEXUS_NUM_JOINTS - 1);
    CHECK(sent_to(7, CMD_SET_STATE) == NULL,
          "a joint that was already running was told to start again");

    const host_can_frame_t *f = sent_to(0, CMD_SET_STATE);
    CHECK(f != NULL, "joint 0 was not told to start");
    if (f)
    {
        CHECK(f->data[0] == (uint8_t)ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL,
              "joint 0 was told to go to state %u, expected %u (running)",
              f->data[0], ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
}

static void test_it_keeps_asking_until_every_motor_confirms(void)
{
    printf("waiting for the motors to confirm\n");

    fresh_board();
    act_request_arm();

    CHECK(act_all_closed_loop() == 0u,
          "it reported every motor running before any had said so");
    CHECK(act_not_closed_loop_mask() == 0x3FFu,
          "the list of joints not yet running was 0x%03X, expected all ten "
          "(0x3FF)", act_not_closed_loop_mask());

    /* Nothing has confirmed, so the request is repeated - slowly. */
    host_can_forget_sent();
    ticks(99);
    CHECK(total_sent() == 0u,
          "the request was repeated too eagerly: %u frames in 99 ticks",
          total_sent());

    act_tick_1khz();
    CHECK(count_of_cmd(CMD_SET_STATE) == (uint32_t)NEXUS_NUM_JOINTS,
          "tick 100 repeated %u start requests, expected %d",
          count_of_cmd(CMD_SET_STATE), NEXUS_NUM_JOINTS);

    /* Now they all confirm. */
    all_drives_report_running();
    CHECK(act_all_closed_loop() == 1u,
          "every motor reported itself running but it still says otherwise");
    CHECK(act_not_closed_loop_mask() == 0u,
          "the list of joints not yet running was 0x%03X, expected empty",
          act_not_closed_loop_mask());

    act_tick_1khz();     /* this is the tick that notices and stops asking */

    /*
     * And it really has stopped. If a motor drops out later it is NOT quietly
     * put back - which is deliberate. A joint that fell out of control on its
     * own has a reason, and starting it again without anyone deciding to
     * would hide that reason. Recovering it takes a fresh, explicit request.
     */
    drive_reports_state(2, 0u, (uint8_t)ODRV_AXIS_STATE_IDLE);
    host_can_forget_sent();
    ticks(300);
    CHECK(total_sent() == 0u,
          "a motor that dropped out was restarted without anyone asking "
          "(%u frames)", total_sent());

    /* Asking explicitly does start it - and only it. */
    act_request_arm();
    CHECK(count_of_cmd(CMD_SET_STATE) == 1u,
          "a fresh request sent %u start requests, expected only the joint "
          "that had dropped out", count_of_cmd(CMD_SET_STATE));
    CHECK(sent_to(2, CMD_SET_STATE) != NULL,
          "the joint that dropped out was not the one restarted");
}

static void test_a_blocked_wire_loses_commands_and_says_so(void)
{
    printf("a wire that stops carrying anything\n");

    fresh_board();

    /*
     * The hardware itself holds only three messages at a time, so the
     * firmware keeps its own queue of fifteen behind that and feeds them in
     * as room appears. Block the wire and both fill up: three in the
     * hardware, fifteen waiting, and everything after that is lost.
     *
     * That is the right thing to do - waiting instead would stall the
     * heartbeat that keeps the whole robot alive - but it must be COUNTED,
     * because a number above zero here is the only sign that instructions are
     * not reaching the motors.
     */
    host_can_wire_blocked(0, 1);

    set_all_targets(1.0f);
    ticks(4);                        /* 4 ticks x 5 joints = 20 commands */

    CHECK(act_tx_dropped(0) == 2u,
          "the blocked wire lost %u commands; expected 20 sent minus the 18 "
          "it can hold", act_tx_dropped(0));
    CHECK(act_tx_dropped(1) == 0u,
          "the healthy wire lost %u commands and should have lost none",
          act_tx_dropped(1));

    /* When the wire recovers, everything still queued goes out. Nothing that
       was accepted is quietly forgotten. */
    host_can_wire_blocked(0, 0);
    host_can_forget_sent();
    act_tx_pump();

    CHECK(host_can_sent_count(0) == 15u,
          "%u queued commands went out after the wire recovered, expected the "
          "15 that were waiting", host_can_sent_count(0));
    CHECK(act_tx_dropped(0) == 2u,
          "the lost-command count changed after recovery, to %u",
          act_tx_dropped(0));
}

static void test_a_refused_message_is_kept_not_thrown_away(void)
{
    printf("a message the hardware refuses\n");

    fresh_board();
    set_all_targets(1.0f);

    /* The hardware refuses the very first message of this tick. */
    host_can_refuse_tx(0, 1u);
    act_tick_1khz();

    /* It must still be there. Dropping it would leave one joint holding a
       position one command out of date, with nothing reporting anything. */
    act_tx_pump();

    CHECK(host_can_sent_count(0) == (uint32_t)NODES_PER_BUS,
          "%u of %d commands survived a refusal", host_can_sent_count(0),
          NODES_PER_BUS);
    CHECK(act_tx_dropped(0) == 0u,
          "a refused message was counted as lost (%u)", act_tx_dropped(0));

    /* And in the original order - joint 0 first. */
    const host_can_frame_t *first = host_can_sent(0, 0);
    CHECK(first && first->identifier == id_of(0, CMD_SET_INPUT_POS),
          "the commands came out in a different order after the refusal");
}

static void test_replies_land_on_the_right_joint(void)
{
    printf("reading replies from the motors\n");

    fresh_board();

    uint32_t before = act_rx_count(1);

    /* Joint 7 is the third motor on the second wire. */
    drive_reports_position(7, 7.5f, -1.25f);

    act_telemetry_t t;
    act_get(&t);

    CHECK(NEAR(t.pos[7], 7.5f, 1e-6f),
          "joint 7's position came back as %.4f, expected 7.5",
          (double)t.pos[7]);
    CHECK(NEAR(t.vel[7], -1.25f, 1e-6f),
          "joint 7's speed came back as %.4f, expected -1.25",
          (double)t.vel[7]);
    CHECK(t.pos_age[7] == 0u,
          "joint 7's brand new reading looked %u ticks old", t.pos_age[7]);
    CHECK(act_rx_count(1) == before + 1u,
          "the reply count for that wire went from %u to %u, expected one more",
          before, act_rx_count(1));

    /* No other joint moved. */
    CHECK(NEAR(t.pos[2], 0.0f, 1e-6f) && NEAR(t.pos[8], 0.0f, 1e-6f),
          "a reply for joint 7 changed another joint's reading");

    /*
     * A frame addressed to a motor that does not exist is ignored, and is not
     * counted either - counting it would let noise on the wire pass for a
     * healthy motor, and "replies are still arriving" is exactly what the
     * health check uses to decide a wire is alive.
     */
    before = act_rx_count(1);
    uint8_t d[8] = { 0 };

    host_can_deliver(1, (0u << 5) | CMD_GET_ENCODER, d);   /* node 0 */
    host_can_deliver(1, (6u << 5) | CMD_GET_ENCODER, d);   /* node 6 */
    act_on_rx(1);

    CHECK(act_rx_count(1) == before,
          "%u frames from motors that do not exist were counted as real",
          act_rx_count(1) - before);

    /* A trip report, and the effort reading, which ODrive puts in the second
       half of the message - the first half is what it was ASKED for. */
    drive_reports_state(4, 0x11u, (uint8_t)ODRV_AXIS_STATE_IDLE);

    uint8_t tq[8];
    put_f32(&tq[0], 9.0f);      /* asked for */
    put_f32(&tq[4], 3.5f);      /* actually doing */
    host_can_deliver(bus_of(4), id_of(4, CMD_GET_TORQUES), tq);
    act_on_rx(bus_of(4));

    act_get(&t);
    CHECK(t.axis_error[4] == 0x11u,
          "joint 4's reported problem came back as 0x%X, expected 0x11",
          t.axis_error[4]);
    CHECK(t.axis_state[4] == (uint8_t)ODRV_AXIS_STATE_IDLE,
          "joint 4's reported state came back as %u", t.axis_state[4]);
    CHECK(NEAR(t.torque[4], 3.5f, 1e-6f),
          "joint 4's effort came back as %.4f, expected 3.5 - the amount it "
          "is actually applying, not the amount asked for",
          (double)t.torque[4]);
}

static void test_a_frame_from_a_wire_that_does_not_exist_is_bounded(void)
{
    printf("a frame handed in against a wire number that cannot exist\n");

    /*
     * There are two wires. The routine that reads incoming frames is called
     * by the two interrupt handlers, which pass 0 and 1, so a larger number
     * should never arrive - but it used to work out WHICH JOINT a frame
     * belonged to by multiplying that number out, without checking it first.
     * A 2 would have filed the frame as joint 10, 11 or 12, and there are
     * only ten. The write would have gone past the end of the table and
     * landed on whichever variable happened to sit next to it in memory.
     *
     * Nothing on the board does that today. The check is here because the
     * cost of being wrong is a variable changing by itself, which is close to
     * the worst thing to have to debug on a robot, and because the same
     * routine already guarded the frame counter the same way - so half of it
     * was careful and half was not.
     *
     * Two things are being checked, by two different mechanisms. The
     * assertions below catch a bogus wire number being read from the wrong
     * place. The sanitizer job in CI catches the write that follows going
     * past the end of the table - which is why the same frame is handed in on
     * BOTH wires: without the frame on wire 1 there is nothing for the
     * unbounded version to read, and so nothing for it to misfile.
     */
    fresh_board();

    /* The reply count is deliberately never reset - it is a total since the
       board powered on, which is how the health check spots a wire going
       quiet - so compare against where it stood, not against zero. */
    uint32_t before = act_rx_count(0);

    uint8_t d[8];
    put_f32(&d[0], 4.25f);
    put_f32(&d[4], 0.0f);

    host_can_deliver(0, (1u << 5) | CMD_GET_ENCODER, d);
    host_can_deliver(1, (1u << 5) | CMD_GET_ENCODER, d);
    act_on_rx(2);            /* there is no wire 2 */

    act_telemetry_t t;
    act_get(&t);

    CHECK(NEAR(t.pos[0], 4.25f, 1e-6f),
          "a frame handed in against wire 2 was filed as joint 0 with "
          "position %.4f, expected 4.25 - it should be treated as one of the "
          "two real wires", (double)t.pos[0]);
    CHECK(act_rx_count(0) == before + 1u,
          "the reply count for wire 0 went from %u to %u, expected one more",
          before, act_rx_count(0));
}

static void test_a_reading_is_handed_over_once_and_then_ages(void)
{
    printf("freshness of readings\n");

    fresh_board();
    drive_reports_position(0, 1.0f, 0.0f);

    act_telemetry_t a, b;
    act_get(&a);
    CHECK((a.flags[0] & NEXUS_ACT_TELEM_FRESH) != 0u,
          "a reading that just arrived was not marked as new");

    /*
     * The "this is new" mark is consumed by reading it. Otherwise every
     * reader after the first would see a stale mark and believe a value that
     * had not been refreshed since.
     */
    act_get(&b);
    CHECK((b.flags[0] & NEXUS_ACT_TELEM_FRESH) == 0u,
          "the same reading was handed over as new twice");
    CHECK(NEAR(b.pos[0], 1.0f, 1e-6f),
          "the reading itself disappeared along with its mark");

    /* Age, on the other hand, is a number and keeps counting. */
    ticks(5);
    act_get(&b);
    CHECK(b.pos_age[0] == 5u,
          "after 5 ticks the reading looked %u ticks old", b.pos_age[0]);

    /* A joint never heard from sits at the top of the scale rather than
       rolling over and pretending to be brand new. */
    ticks(20);
    act_get(&b);
    CHECK(b.pos_age[9] == 0xFFFFu,
          "a joint that has never reported now looks %u ticks old",
          b.pos_age[9]);
}

static void test_a_wire_that_shuts_itself_down_is_restarted(void)
{
    printf("a wire in trouble\n");

    fresh_board();

    /* The hardware keeps a running unhappiness score for each wire. */
    host_can_set_tx_errors(0, 42u);
    act_bus_service();
    CHECK(act_tx_error_count(0) == 42u,
          "the unhappiness score read back as %u, expected 42",
          act_tx_error_count(0));

    /*
     * If that score cannot be read this time round, the last known value
     * stays. Reporting a fictitious zero would look exactly like a wire that
     * had just recovered.
     */
    host_can_break_counter_read(0, 1);
    host_can_set_tx_errors(0, 0u);
    act_bus_service();
    CHECK(act_tx_error_count(0) == 42u,
          "a failed read reported %u; it should have kept the last known 42",
          act_tx_error_count(0));
    host_can_break_counter_read(0, 0);

    /*
     * Too much unhappiness and the wire switches itself off entirely. It does
     * not come back on its own, and the only symptom is that replies quietly
     * stop - so this is the code that notices and restarts it.
     */
    uint32_t starts_before = host_can_start_count(0);
    host_can_set_bus_off(0, 1);
    act_bus_service();

    CHECK(act_bus_off_count(0) == 1u,
          "the wire shut down but the count says %u", act_bus_off_count(0));
    CHECK(host_can_start_count(0) == starts_before + 1u,
          "the wire was not restarted");

    /* A wire whose state cannot be read at all is left alone rather than
       guessed at. */
    starts_before = host_can_start_count(1);
    host_can_break_status_read(1, 1);
    host_can_set_bus_off(1, 1);
    act_bus_service();

    CHECK(act_bus_off_count(1) == 0u,
          "a wire that could not be asked was recorded as shut down anyway");
    CHECK(host_can_start_count(1) == starts_before,
          "a wire that could not be asked was restarted anyway");
    host_can_break_status_read(1, 0);

    /* Asking about a wire that does not exist answers zero rather than
       reading past the end of the two it has. */
    CHECK(act_bus_off_count(9) == 0u && act_tx_error_count(9) == 0u &&
              act_tx_dropped(9) == 0u && act_rx_count(9) == 0u,
          "asking about a third wire returned something");
}

static void test_the_emergency_stop_bypasses_everything(void)
{
    printf("the emergency stop\n");

    fresh_board();
    set_all_targets(1.0f);

    /* Bus 0 is blocked, so its queues are full and a backlog is waiting. */
    host_can_wire_blocked(0, 1);
    ticks(2);
    host_can_forget_sent();

    uint32_t dropped_before = act_tx_dropped(0);

    /*
     * This runs from a crash handler, where the ordinary queue may be exactly
     * what is broken. So it goes straight to the hardware, and it must finish
     * even when a wire will never accept anything again - hanging here would
     * strand the other wire too, and with it the other five joints.
     */
    act_emergency_idle();

    CHECK(host_can_sent_count(1) == (uint32_t)NODES_PER_BUS,
          "the healthy wire carried %u stop requests, expected %d",
          host_can_sent_count(1), NODES_PER_BUS);

    for (int j = NODES_PER_BUS; j < NEXUS_NUM_JOINTS; j++)
    {
        const host_can_frame_t *f = sent_to(j, CMD_SET_STATE);
        CHECK(f != NULL, "joint %d got no emergency stop", j);
        if (f)
        {
            CHECK(f->data[0] == (uint8_t)ODRV_AXIS_STATE_IDLE,
                  "joint %d's emergency stop asked for state %u, expected %u",
                  j, f->data[0], ODRV_AXIS_STATE_IDLE);
        }
    }

    /* It touched neither the ordinary queue nor its counters. */
    CHECK(act_tx_dropped(0) == dropped_before,
          "the emergency stop disturbed the ordinary queue's counters");

    host_can_wire_blocked(0, 0);
    host_can_forget_sent();
    act_tx_pump();
    CHECK(host_can_sent_count(0) > 0u,
          "the backlog waiting on the ordinary queue was lost");
}

int main(void)
{
    printf("act_odrive.c host tests\n");
    printf("-----------------------\n");

    /* First, and it has to be: there is no way back to "not set up yet". */
    test_the_emergency_stop_is_safe_before_startup();

    test_startup_wakes_both_wires_and_commands_nothing();
    test_the_first_move_starts_from_where_the_leg_is();
    test_an_old_or_nonsense_reading_is_not_used_as_a_start();
    test_a_move_is_spread_over_four_ticks_and_then_stops();
    test_a_fast_move_reports_a_capped_speed_not_a_reversed_one();
    test_stopping_goes_limp_and_stays_limp();
    test_switching_on_clears_only_real_errors();
    test_it_keeps_asking_until_every_motor_confirms();
    test_a_blocked_wire_loses_commands_and_says_so();
    test_a_refused_message_is_kept_not_thrown_away();
    test_replies_land_on_the_right_joint();
    test_a_frame_from_a_wire_that_does_not_exist_is_bounded();
    test_a_reading_is_handed_over_once_and_then_ages();
    test_a_wire_that_shuts_itself_down_is_restarted();
    test_the_emergency_stop_bypasses_everything();

    printf("-----------------------\n");
    if (s_fail)
    {
        printf("FAILED (%d)\n", s_fail);
        return 1;
    }
    printf("PASSED (0 failures)\n");
    return 0;
}
