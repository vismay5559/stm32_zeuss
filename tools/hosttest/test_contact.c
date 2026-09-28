/*
 * Host test for contact.c.
 *
 * The foot switches are the estimator's only direct evidence that the robot is
 * touching the ground, and fusion.c keys inekf_add_contact() straight off the
 * bits this file produces. A wrong bit here does not look like a bug - it
 * looks like a robot that cannot estimate its own velocity.
 *
 * Runs on a workstation against tools/hosttest/stub/main.h. No board needed.
 */

#include "contact.h"
#include "link_proto.h"
#include "main.h"

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

/* contact.c's own debounce constants, which are static to that file. */
#define MAKE_TICKS   3
#define BREAK_TICKS  8

static void poll_n(int n)
{
    for (int i = 0; i < n; i++) { contact_poll(); }
}

static void settle_released(void)
{
    host_release_all();
    poll_n(BREAK_TICKS + 2);
}

typedef struct
{
    const char   *name;
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       expect_bit;    /* the ONE bit this switch must set   */
} sw_t;

/*
 * One switch per foot, at the centre of the sole. The pins keep their CubeMX
 * names: L_TOE and R_TOE are the two that are still wired, and L_HEEL/R_HEEL
 * are no longer connected to anything - see contact_init().
 */
static const sw_t SWITCHES[NEXUS_NUM_CONTACTS] = {
    { "LEFT",  L_TOE_GPIO_Port, L_TOE_Pin, NEXUS_CONTACT_L_BIT },
    { "RIGHT", R_TOE_GPIO_Port, R_TOE_Pin, NEXUS_CONTACT_R_BIT },
};

/*
 * Each switch, pressed on its own, must set exactly its own bit and light
 * exactly its own foot. This is the test that catches a mask table built from
 * the INDEX macros (0,1) rather than the _BIT macros - which would make the
 * left foot invisible and the right foot report as the left.
 */
static void test_one_switch_one_bit(void)
{
    printf("one switch -> one bit\n");

    for (int i = 0; i < NEXUS_NUM_CONTACTS; i++)
    {
        const sw_t *s = &SWITCHES[i];

        settle_released();
        host_press(s->port, s->pin, 1);
        poll_n(MAKE_TICKS);

        uint8_t sw   = contact_switches();
        uint8_t feet = contact_feet();

        CHECK(sw == s->expect_bit,
              "%s pressed: switches = 0x%02X, expected 0x%02X",
              s->name, sw, s->expect_bit);

        /* One switch per foot, so the foot mask is the switch mask - but go
           through contact_feet() rather than assuming it, since that is the
           thing the estimator actually asks. */
        CHECK(feet == s->expect_bit,
              "%s pressed: feet = 0x%02X, expected 0x%02X",
              s->name, feet, s->expect_bit);
    }
}

/*
 * The float array the policy consumes is built in app.c as
 *     contact[c] = (sw & (1u << c)) ? 1.0f : 0.0f
 * so the switch bits must sit at the bit positions the protocol names.
 */
static void test_bit_positions_match_protocol(void)
{
    printf("bit positions match the protocol's index order\n");

    const int idx[NEXUS_NUM_CONTACTS] = {
        NEXUS_CONTACT_LEFT, NEXUS_CONTACT_RIGHT,
    };

    for (int i = 0; i < NEXUS_NUM_CONTACTS; i++)
    {
        const sw_t *s = &SWITCHES[i];

        settle_released();
        host_press(s->port, s->pin, 1);
        poll_n(MAKE_TICKS);

        uint8_t sw = contact_switches();

        for (int c = 0; c < NEXUS_NUM_CONTACTS; c++)
        {
            int want = (c == idx[i]) ? 1 : 0;
            int got  = (sw & (1u << c)) ? 1 : 0;

            CHECK(got == want,
                  "%s pressed: contact[%d] = %d, expected %d",
                  s->name, c, got, want);
        }
    }
}

/*
 * One foot's switch must never move the other foot.
 *
 * This used to press both switches on a foot to prove they OR-ed together.
 * There is only one per foot now, so what is left to protect is the thing
 * that actually matters to the estimator: the two feet are independent, and
 * standing on one does not make the other look planted. Anchoring a foot that
 * is in the air is how the position estimate runs away.
 */
static void test_feet_are_independent(void)
{
    printf("one foot's switch never moves the other foot\n");

    settle_released();
    host_press(L_TOE_GPIO_Port, L_TOE_Pin, 1);
    poll_n(MAKE_TICKS);

    CHECK(contact_feet() == NEXUS_CONTACT_L_BIT,
          "left switch: feet = 0x%02X, expected 0x%02X",
          contact_feet(), NEXUS_CONTACT_L_BIT);

    settle_released();
    host_press(R_TOE_GPIO_Port, R_TOE_Pin, 1);
    poll_n(MAKE_TICKS);

    CHECK(contact_feet() == NEXUS_CONTACT_R_BIT,
          "right switch: feet = 0x%02X, expected 0x%02X",
          contact_feet(), NEXUS_CONTACT_R_BIT);

    settle_released();
    host_press(L_TOE_GPIO_Port, L_TOE_Pin, 1);
    host_press(R_TOE_GPIO_Port, R_TOE_Pin, 1);
    poll_n(MAKE_TICKS);

    CHECK(contact_feet() == (NEXUS_CONTACT_L_BIT | NEXUS_CONTACT_R_BIT),
          "both switches: feet = 0x%02X, expected 0x%02X",
          contact_feet(), NEXUS_CONTACT_L_BIT | NEXUS_CONTACT_R_BIT);
}

/*
 * Make and break use different confirmation times on purpose - a planted foot
 * that chatters must not read as lift-off. Lock that asymmetry down so nobody
 * "simplifies" it to a single constant later.
 */
static void test_debounce_asymmetry(void)
{
    printf("debounce: %d ticks to make, %d to break\n", MAKE_TICKS, BREAK_TICKS);

    settle_released();
    host_press(L_TOE_GPIO_Port, L_TOE_Pin, 1);

    poll_n(MAKE_TICKS - 1);
    CHECK(contact_switches() == 0,
          "made after only %d ticks (should need %d)", MAKE_TICKS - 1, MAKE_TICKS);

    poll_n(1);
    CHECK(contact_switches() == NEXUS_CONTACT_L_BIT,
          "not made after %d ticks", MAKE_TICKS);

    host_press(L_TOE_GPIO_Port, L_TOE_Pin, 0);

    poll_n(BREAK_TICKS - 1);
    CHECK(contact_switches() == NEXUS_CONTACT_L_BIT,
          "broke after only %d ticks (should need %d)", BREAK_TICKS - 1, BREAK_TICKS);

    poll_n(1);
    CHECK(contact_switches() == 0, "not broken after %d ticks", BREAK_TICKS);
}

/* contact_stable_ticks() is what lets the estimator ignore a foot that has
   only just landed, so it must reset on every transition. */
static void test_stable_ticks(void)
{
    printf("stable-tick counters reset on each transition\n");

    settle_released();
    CHECK(contact_stable_ticks(0) > 0, "left counter never advanced while released");

    host_press(L_TOE_GPIO_Port, L_TOE_Pin, 1);
    poll_n(MAKE_TICKS);
    CHECK(contact_stable_ticks(0) == 0, "left counter did not reset on touchdown");

    poll_n(10);
    CHECK(contact_stable_ticks(0) == 10,
          "left counter = %u after 10 ticks planted, expected 10",
          contact_stable_ticks(0));

    CHECK(contact_stable_ticks(2) == 0, "out-of-range foot index must read 0");
}

int main(void)
{
    printf("contact.c host tests\n--------------------\n");

    contact_init();

    test_one_switch_one_bit();
    test_bit_positions_match_protocol();
    test_feet_are_independent();
    test_debounce_asymmetry();
    test_stable_ticks();

    printf("--------------------\n%s (%d failure%s)\n",
           s_fail ? "FAILED" : "PASSED", s_fail, s_fail == 1 ? "" : "s");

    return s_fail ? 1 : 0;
}
