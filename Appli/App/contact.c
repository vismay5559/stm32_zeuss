#include "contact.h"
#include "link_proto.h"
#include "main.h"

/*
 * Switches are wired to ground with internal pull-ups, so a closed switch reads 0.
 *
 * Make and break use different confirmation times on purpose. A foot that is
 * really planted may still chatter as it rolls, so a brief open is not treated
 * as lift-off; conversely a false "in contact" corrupts an InEKF velocity
 * estimate badly, so landing is only declared once the switch has settled.
 */
/*
 * Overridable at configure time. These are exactly the numbers you want to
 * trim against a real gait - a foot that chatters as it rolls, a heel strike
 * that bounces - and having to edit and reflash to try a value discourages
 * trying any.
 *
 *     CONTACT_MAKE_TICKS=4 CONTACT_BREAK_TICKS=12 cmake --preset Debug
 */
#ifndef CONTACT_MAKE_TICKS
#define CONTACT_MAKE_TICKS   3u
#endif
#ifndef CONTACT_BREAK_TICKS
#define CONTACT_BREAK_TICKS  8u
#endif

/* Break must outlast make, or a planted foot that chatters reads as lift-off -
   which is the failure this asymmetry exists to prevent. */
_Static_assert(CONTACT_BREAK_TICKS > CONTACT_MAKE_TICKS,
               "contact break confirmation must be longer than make");

#define STABLE_TICKS_MAX     0xFFFFu

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       mask;      /* a NEXUS_CONTACT_*_BIT, never an index */
    uint16_t      counter;
    uint8_t       stable;
} contact_ch_t;

static contact_ch_t s_ch[NEXUS_NUM_CONTACTS];
static uint8_t      s_switches;
static uint16_t     s_foot_ticks[NEXUS_NUM_CONTACTS];

/*
 * link_proto.h names each contact TWICE - once as an array index for
 * contact[] in the policy block, and once as a bit for the `contacts`
 * bitmask:
 *
 *     NEXUS_CONTACT_LEFT     0          <- index into contact[]
 *     NEXUS_CONTACT_L_BIT    (1u << 0)  <- bit in the mask
 *
 * The two families differ by a few characters and the index one is a perfectly
 * valid uint8_t, so writing an index into `mask` compiles silently. Back when
 * there were four switches that produced a mask table of {0,1,2,3}: the left
 * toe set nothing at all, the right toe reported as the left heel, and the
 * right heel set two bits. Every downstream consumer - the policy's contact[],
 * contact_feet(), and the InEKF's choice of which foot to anchor - then worked
 * from switches that were not the ones being pressed.
 *
 * With two switches the same mistake is quieter and no less wrong: the left
 * foot would vanish and the right would report as the left. The assertions
 * below make the substitution a build error rather than a robot that cannot
 * estimate its own velocity.
 */
_Static_assert(NEXUS_CONTACT_L_BIT == (1u << NEXUS_CONTACT_LEFT),
               "the left bit must sit at the left index");
_Static_assert(NEXUS_CONTACT_R_BIT == (1u << NEXUS_CONTACT_RIGHT),
               "the right bit must sit at the right index");
_Static_assert(NEXUS_NUM_CONTACTS == 2, "one switch per foot, two feet");

void contact_init(void)
{
    /*
     * The pins keep their CubeMX names. L_TOE/R_TOE were two of the four
     * switch inputs and are now the single switch on each foot; L_HEEL and
     * R_HEEL are unused. Renaming them means regenerating the .ioc, which is
     * a GUI action nobody can do from a diff - so the mapping is written out
     * here instead, where it is the first thing anyone wiring a foot reads.
     */
    s_ch[NEXUS_CONTACT_LEFT]  = (contact_ch_t){ L_TOE_GPIO_Port, L_TOE_Pin, NEXUS_CONTACT_L_BIT, 0, 0 };
    s_ch[NEXUS_CONTACT_RIGHT] = (contact_ch_t){ R_TOE_GPIO_Port, R_TOE_Pin, NEXUS_CONTACT_R_BIT, 0, 0 };

    s_switches = 0;
    for (int f = 0; f < NEXUS_NUM_CONTACTS; f++)
    {
        s_foot_ticks[f] = 0;
    }
}

void contact_poll(void)
{
    uint8_t sw = 0;

    for (int i = 0; i < NEXUS_NUM_CONTACTS; i++)
    {
        uint8_t raw = (HAL_GPIO_ReadPin(s_ch[i].port, s_ch[i].pin) == GPIO_PIN_RESET) ? 1u : 0u;

        if (raw != s_ch[i].stable)
        {
            uint16_t need = raw ? CONTACT_MAKE_TICKS : CONTACT_BREAK_TICKS;

            if (++s_ch[i].counter >= need)
            {
                s_ch[i].stable  = raw;
                s_ch[i].counter = 0;
            }
        }
        else
        {
            s_ch[i].counter = 0;
        }

        if (s_ch[i].stable)
        {
            sw |= s_ch[i].mask;
        }
    }

    /*
     * One switch per foot, so the switch mask IS the foot mask - there is no
     * longer a toe and a heel to OR together. contact_feet() is kept as its
     * own name anyway: the two meanings are only equal because of how the
     * robot is currently built, and callers that ask "is this foot down"
     * should not have to know that.
     */
    for (int f = 0; f < NEXUS_NUM_CONTACTS; f++)
    {
        uint8_t now  = (sw & (uint8_t)(1u << f)) ? 1u : 0u;
        uint8_t prev = (s_switches & (uint8_t)(1u << f)) ? 1u : 0u;

        if (now != prev)
        {
            s_foot_ticks[f] = 0;
        }
        else if (s_foot_ticks[f] < STABLE_TICKS_MAX)
        {
            s_foot_ticks[f]++;
        }
    }

    s_switches = sw;
}

uint8_t contact_switches(void)
{
    return s_switches;
}

uint8_t contact_feet(void)
{
    return s_switches;
}

uint16_t contact_stable_ticks(uint8_t foot)
{
    return (foot < (uint8_t)NEXUS_NUM_CONTACTS) ? s_foot_ticks[foot] : 0u;
}
