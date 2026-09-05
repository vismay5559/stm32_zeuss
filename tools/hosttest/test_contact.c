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
    uint8_t       expect_foot;   /* the foot bit it must derive        */
} sw_t;

static const sw_t SWITCHES[4] = {
    { "L_TOE",  L_TOE_GPIO_Port,  L_TOE_Pin,  NEXUS_CONTACT_L_TOE_BIT,  NEXUS_CONTACT_L_FOOT },
    { "L_HEEL", L_HEEL_GPIO_Port, L_HEEL_Pin, NEXUS_CONTACT_L_HEEL_BIT, NEXUS_CONTACT_L_FOOT },
    { "R_TOE",  R_TOE_GPIO_Port,  R_TOE_Pin,  NEXUS_CONTACT_R_TOE_BIT,  NEXUS_CONTACT_R_FOOT },
    { "R_HEEL", R_HEEL_GPIO_Port, R_HEEL_Pin, NEXUS_CONTACT_R_HEEL_BIT, NEXUS_CONTACT_R_FOOT },
};

/*
 * Each switch, pressed on its own, must set exactly its own bit and light
 * exactly its own foot. This is the test that C1 fails: the mask table was
 * built from the INDEX macros (0,1,2,3) rather than the _BIT macros, so
 * L_TOE set nothing at all and R_HEEL set two bits at once.
 */
static void test_one_switch_one_bit(void)
{
    printf("one switch -> one bit\n");

    for (int i = 0; i < 4; i++)
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

        CHECK(feet == s->expect_foot,
              "%s pressed: feet = 0x%02X, expected 0x%02X",
              s->name, feet, s->expect_foot);
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

    const int idx[4] = {
        NEXUS_CONTACT_L_TOE, NEXUS_CONTACT_L_HEEL,
        NEXUS_CONTACT_R_TOE, NEXUS_CONTACT_R_HEEL,
    };

    for (int i = 0; i < 4; i++)
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

/* Both switches on one foot, and nothing on the other. */
static void test_feet_are_independent(void)
{
    printf("one foot's switches never move the other foot\n");

    settle_released();
    host_press(L_TOE_GPIO_Port,  L_TOE_Pin,  1);
    host_press(L_HEEL_GPIO_Port, L_HEEL_Pin, 1);
    poll_n(MAKE_TICKS);

    CHECK(contact_feet() == NEXUS_CONTACT_L_FOOT,
          "both left switches: feet = 0x%02X, expected 0x%02X",
          contact_feet(), NEXUS_CONTACT_L_FOOT);

    settle_released();
    host_press(R_TOE_GPIO_Port,  R_TOE_Pin,  1);
    host_press(R_HEEL_GPIO_Port, R_HEEL_Pin, 1);
    poll_n(MAKE_TICKS);

    CHECK(contact_feet() == NEXUS_CONTACT_R_FOOT,
          "both right switches: feet = 0x%02X, expected 0x%02X",
          contact_feet(), NEXUS_CONTACT_R_FOOT);

    settle_released();
    host_press(L_TOE_GPIO_Port, L_TOE_Pin, 1);
    host_press(R_TOE_GPIO_Port, R_TOE_Pin, 1);
    poll_n(MAKE_TICKS);

    CHECK(contact_feet() == (NEXUS_CONTACT_L_FOOT | NEXUS_CONTACT_R_FOOT),
          "both toes: feet = 0x%02X, expected 0x%02X",
          contact_feet(), NEXUS_CONTACT_L_FOOT | NEXUS_CONTACT_R_FOOT);
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
    CHECK(contact_switches() == NEXUS_CONTACT_L_TOE_BIT,
          "not made after %d ticks", MAKE_TICKS);

    host_press(L_TOE_GPIO_Port, L_TOE_Pin, 0);

    poll_n(BREAK_TICKS - 1);
    CHECK(contact_switches() == NEXUS_CONTACT_L_TOE_BIT,
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
