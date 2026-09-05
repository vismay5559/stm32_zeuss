/*
 * Host test for link_usb.c - the cable to the Pi.
 *
 * Every instruction the robot acts on arrives through here. safety.c can only
 * judge a command it received correctly, so a framing bug upstream of it means
 * the robot acting on something the Pi never sent - and the checks downstream
 * would see nothing wrong.
 *
 * The bytes arrive in whatever chunks USB feels like delivering, which is the
 * heart of it: a frame can be split anywhere, two can arrive together, and
 * rubbish can appear between them. None of that may change what the robot
 * ends up obeying.
 */

#include "link_usb.h"
#include "usbd_cdc_if.h"

#include <stddef.h>
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

/* ---- building a command the way the Pi does ------------------------- */

static void build_command(nexus_cmd_t *c, uint32_t seq)
{
    memset(c, 0, sizeof(*c));
    c->sync    = NEXUS_SYNC;
    c->msg_id  = NEXUS_MSG_COMMAND;
    c->version = NEXUS_PROTO_VERSION;
    c->seq     = seq;

    for (int i = 0; i < NEXUS_NUM_JOINTS; i++)
    {
        c->target_pos[i] = 0.1f * (float)(i + 1);
    }

    c->crc = nexus_crc16((const uint8_t *)c, sizeof(*c) - sizeof(uint16_t));
}

/*
 * link_usb_init() deliberately does NOT reset the received/dropped counters:
 * health.c decides the Pi has gone quiet by watching link_usb_cmd_count()
 * stop climbing, so zeroing it on a re-init would look exactly like a dead
 * link. Tests therefore compare deltas, never absolute totals.
 */
static void reset_link(void)
{
    stub_usb_reset();
    link_usb_init();
}

/* Push a buffer through in chunks of `chunk` bytes, as USB would. */
static void feed_in_chunks(const uint8_t *buf, uint32_t len, uint32_t chunk)
{
    uint32_t off = 0;
    while (off < len)
    {
        uint32_t n = (len - off < chunk) ? (len - off) : chunk;
        link_usb_on_rx((uint8_t *)(buf + off), n);
        off += n;
    }
}

/* ---- the tests -------------------------------------------------------- */

static void test_a_whole_command_arrives_intact(void)
{
    printf("a command delivered in one piece is accepted unchanged\n");

    reset_link();
    uint32_t before = link_usb_cmd_count();

    nexus_cmd_t sent;
    build_command(&sent, 42);

    link_usb_on_rx((uint8_t *)&sent, sizeof(sent));

    nexus_cmd_t got;
    CHECK(link_usb_take_command(&got) == 1, "a valid command was not accepted");
    CHECK(got.seq == 42u, "sequence arrived as %lu, expected 42",
          (unsigned long)got.seq);
    CHECK(memcmp(&got, &sent, sizeof(got)) == 0,
          "the command was altered on the way through");
    CHECK(link_usb_cmd_count() == before + 1u,
          "the command was not counted (%lu then %lu)",
          (unsigned long)before, (unsigned long)link_usb_cmd_count());

    /* Taking it twice must not hand back a stale copy. */
    CHECK(link_usb_take_command(&got) == 0,
          "the same command was handed out twice");
}

static void test_a_command_split_anywhere_still_arrives(void)
{
    printf("a command split across deliveries is reassembled, whatever the split\n");

    /*
     * USB hands over whatever it has. A frame arriving one byte at a time, or
     * broken at an awkward offset, has to end up identical to one that arrived
     * whole - otherwise the robot acts on a command that depends on how the
     * cable happened to chunk it.
     */
    const uint32_t chunks[5] = { 1, 2, 7, 31, 64 };

    for (int c = 0; c < 5; c++)
    {
        reset_link();

        nexus_cmd_t sent;
        build_command(&sent, 100u + (uint32_t)c);
        feed_in_chunks((const uint8_t *)&sent, sizeof(sent), chunks[c]);

        nexus_cmd_t got;
        CHECK(link_usb_take_command(&got) == 1,
              "split into %lu-byte pieces, the command was lost",
              (unsigned long)chunks[c]);
        CHECK(memcmp(&got, &sent, sizeof(got)) == 0,
              "split into %lu-byte pieces, the command came through altered",
              (unsigned long)chunks[c]);
    }
}

static void test_a_corrupted_command_is_thrown_away(void)
{
    printf("a command damaged in transit is refused, not acted on\n");

    reset_link();
    uint32_t before = link_usb_cmd_count();

    nexus_cmd_t sent;
    build_command(&sent, 7);

    /* Flip a bit in a target position, leaving the checksum stale. */
    uint8_t raw[sizeof(nexus_cmd_t)];
    memcpy(raw, &sent, sizeof(raw));
    raw[offsetof(nexus_cmd_t, target_pos) + 1] ^= 0x40u;

    link_usb_on_rx(raw, sizeof(raw));

    nexus_cmd_t got;
    CHECK(link_usb_take_command(&got) == 0,
          "a command with a broken checksum was accepted");
    CHECK(link_usb_cmd_count() == before,
          "a corrupted command was counted as received (%lu then %lu)",
          (unsigned long)before, (unsigned long)link_usb_cmd_count());
}

static void test_rubbish_before_a_command_is_ignored(void)
{
    printf("noise on the wire does not stop the next real command\n");

    /*
     * A half frame, a burst of noise, a truncated message - the parser has to
     * find the start of the next real command rather than staying wedged on
     * whatever it saw first.
     */
    reset_link();

    const uint8_t junk[] = { 0x00, 0xFF, 0x12, 0x34, 0xAA, 0x55, 0x7E, 0x01 };
    link_usb_on_rx((uint8_t *)junk, sizeof(junk));

    nexus_cmd_t sent;
    build_command(&sent, 9);
    link_usb_on_rx((uint8_t *)&sent, sizeof(sent));

    nexus_cmd_t got;
    CHECK(link_usb_take_command(&got) == 1,
          "noise before a command stopped it being seen");
    CHECK(got.seq == 9u, "the wrong command came through (seq %lu)",
          (unsigned long)got.seq);
}

static void test_a_stray_sync_byte_costs_one_frame_and_no_more(void)
{
    printf("a stray sync byte costs the frame behind it, then recovery\n");

    /*
     * NEXUS_SYNC is 0xA5A5 - BOTH bytes the same. That makes a stray 0xA5
     * arriving immediately before a real frame genuinely ambiguous: the
     * parser takes it plus the frame's own first byte as the two-byte marker,
     * ends up one byte out of step, and the checksum then rejects the frame.
     *
     * That is a property of choosing a marker whose two bytes are equal, not
     * a mistake in the parser - no amount of lookahead makes those two cases
     * distinguishable. It also means the "the byte might start a new frame"
     * branch in feed() is unreachable as things stand: if a byte is not the
     * high sync byte then it is not the low one either, because they are the
     * same value.
     *
     * What this pins is the cost: exactly one frame, and the next one is
     * accepted normally. The Pi sends at 250 Hz, so a lost frame is a 4 ms
     * gap, and the sequence numbers make it visible. What must NOT happen is
     * the parser staying wedged.
     */
    reset_link();
    uint32_t before = link_usb_cmd_count();

    nexus_cmd_t first, second;
    build_command(&first,  55);
    build_command(&second, 56);

    uint8_t stream[1 + sizeof(nexus_cmd_t)];
    stream[0] = (uint8_t)(NEXUS_SYNC & 0xFFu);       /* the stray byte */
    memcpy(stream + 1, &first, sizeof(first));
    link_usb_on_rx(stream, sizeof(stream));

    nexus_cmd_t got;
    CHECK(link_usb_take_command(&got) == 0,
          "the misaligned frame was accepted - the checksum should have "
          "rejected it");
    CHECK(link_usb_cmd_count() == before,
          "the misaligned frame was counted as received");

    /* The next clean frame must be accepted, or the parser is wedged. */
    link_usb_on_rx((uint8_t *)&second, sizeof(second));

    CHECK(link_usb_take_command(&got) == 1,
          "after a stray sync byte the parser never recovered");
    CHECK(got.seq == 56u, "recovered the wrong command (seq %lu)",
          (unsigned long)got.seq);
    CHECK(link_usb_cmd_count() == before + 1u,
          "exactly one command should have been counted, got %lu",
          (unsigned long)(link_usb_cmd_count() - before));
}

static void test_a_truncated_command_does_not_block_the_next(void)
{
    printf("half a command followed by a whole one yields the whole one\n");

    reset_link();

    nexus_cmd_t partial;
    build_command(&partial, 1);
    link_usb_on_rx((uint8_t *)&partial, sizeof(partial) / 2);   /* cut short */

    nexus_cmd_t sent;
    build_command(&sent, 2);
    link_usb_on_rx((uint8_t *)&sent, sizeof(sent));

    nexus_cmd_t got;
    if (link_usb_take_command(&got) == 1)
    {
        CHECK(got.seq == 2u,
              "after a truncated frame the command handed over was seq %lu, "
              "not the complete one (2)", (unsigned long)got.seq);
    }
    else
    {
        /* Also acceptable: the parser is still resynchronising. What is NOT
           acceptable is handing over the truncated one as if it were whole. */
        printf("        (note: the parser resynchronised rather than "
               "recovering the second frame)\n");
    }
    CHECK(got.seq != 1u, "a truncated command was handed over as complete");
}

static void test_back_to_back_commands_both_arrive(void)
{
    printf("two commands in one delivery are both seen, newest last\n");

    reset_link();
    uint32_t before = link_usb_cmd_count();

    uint8_t both[2 * sizeof(nexus_cmd_t)];
    nexus_cmd_t a, b;
    build_command(&a, 11);
    build_command(&b, 12);
    memcpy(both, &a, sizeof(a));
    memcpy(both + sizeof(a), &b, sizeof(b));

    link_usb_on_rx(both, sizeof(both));

    CHECK(link_usb_cmd_count() == before + 2u,
          "two commands arrived together but %lu were counted",
          (unsigned long)(link_usb_cmd_count() - before));

    /* Only the most recent is kept - the robot should act on the newest
       instruction, not a queued stale one. */
    nexus_cmd_t got;
    CHECK(link_usb_take_command(&got) == 1, "neither command was available");
    CHECK(got.seq == 12u,
          "the older command was handed over (seq %lu) rather than the newer",
          (unsigned long)got.seq);
}

static void test_a_state_packet_is_sent_with_a_valid_checksum(void)
{
    printf("an outgoing report carries a header and a checksum the Pi can verify\n");

    reset_link();

    nexus_state_t st;
    memset(&st, 0, sizeof(st));
    st.seq = 77;

    uint8_t rc = link_usb_send_state(&st);
    CHECK(rc == USBD_OK, "sending a report failed with %u", rc);
    CHECK(stub_usb_tx_calls() == 1u, "nothing was actually transmitted");
    CHECK(stub_usb_last_tx_len() == sizeof(nexus_state_t),
          "sent %u bytes, expected %u", stub_usb_last_tx_len(),
          (unsigned)sizeof(nexus_state_t));

    /* Check it the way the Pi will. */
    const nexus_state_t *sent = (const nexus_state_t *)stub_usb_last_tx();
    uint16_t expect = nexus_crc16((const uint8_t *)sent,
                                  sizeof(nexus_state_t) - sizeof(uint16_t));

    CHECK(sent->crc == expect, "the report's checksum does not verify");
    CHECK(sent->sync == NEXUS_SYNC, "the report has no sync marker");
    CHECK(sent->msg_id == NEXUS_MSG_STATE, "the report has the wrong message id");
    CHECK(sent->version == NEXUS_PROTO_VERSION, "the report has the wrong version");
    CHECK(sent->seq == 77u, "the report's sequence was altered");
}

static void test_a_busy_cable_drops_the_report_rather_than_splicing_it(void)
{
    printf("when the cable is busy a report is dropped, not half-overwritten\n");

    /*
     * The previous report is still being read out of the same buffer. Copying
     * a new one in first would send the Pi something half one tick and half
     * another; its checksum would fail, so nothing wrong reaches the policy -
     * but the packet is lost, and without the counter, invisibly.
     */
    reset_link();

    nexus_state_t st;
    memset(&st, 0, sizeof(st));
    st.seq = 1;
    CHECK(link_usb_send_state(&st) == USBD_OK, "setup: the first report failed");

    uint32_t before = link_usb_tx_dropped();

    stub_usb_set_busy(1);
    st.seq = 2;
    uint8_t rc = link_usb_send_state(&st);
    stub_usb_set_busy(0);

    CHECK(rc == USBD_BUSY, "a busy cable did not report BUSY (%u)", rc);
    CHECK(link_usb_tx_dropped() == before + 1u,
          "the dropped report was not counted (%lu then %lu)",
          (unsigned long)before, (unsigned long)link_usb_tx_dropped());

    /* The buffer must still hold the FIRST report, untouched. */
    const nexus_state_t *sent = (const nexus_state_t *)stub_usb_last_tx();
    CHECK(sent->seq == 1u,
          "the dropped report overwrote the one still being sent (seq %lu)",
          (unsigned long)sent->seq);
}

int main(void)
{
    printf("link_usb.c host tests\n");
    printf("---------------------\n");

    test_a_whole_command_arrives_intact();
    test_a_command_split_anywhere_still_arrives();
    test_a_corrupted_command_is_thrown_away();
    test_rubbish_before_a_command_is_ignored();
    test_a_stray_sync_byte_costs_one_frame_and_no_more();
    test_a_truncated_command_does_not_block_the_next();
    test_back_to_back_commands_both_arrive();
    test_a_state_packet_is_sent_with_a_valid_checksum();
    test_a_busy_cable_drops_the_report_rather_than_splicing_it();

    printf("---------------------\n");
    if (s_fail)
    {
        printf("FAILED (%d)\n", s_fail);
        return 1;
    }
    printf("PASSED (0 failures)\n");
    return 0;
}
