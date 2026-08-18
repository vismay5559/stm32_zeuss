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
#define CONTACT_MAKE_TICKS   3u
#define CONTACT_BREAK_TICKS  8u

#define STABLE_TICKS_MAX     0xFFFFu

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       mask;      /* a NEXUS_CONTACT_*_BIT, never an index */
    uint16_t      counter;
    uint8_t       stable;
} contact_ch_t;

static contact_ch_t s_ch[4];
static uint8_t      s_switches;
static uint8_t      s_feet;
static uint16_t     s_foot_ticks[2];

/*
 * link_proto.h names each contact TWICE - once as an array index for
 * contact[] in the policy block, and once as a bit for the `contacts`
 * bitmask:
 *
 *     NEXUS_CONTACT_L_TOE       0          <- index into contact[]
 *     NEXUS_CONTACT_L_TOE_BIT   (1u << 0)  <- bit in the mask
 *
 * The two families differ by four characters and the index one is a perfectly
 * valid uint8_t, so writing an index into `mask` compiles silently and yields
 * a mask table of {0,1,2,3}: the left toe sets nothing at all, the right toe
 * reports as the left heel, and the right heel sets two bits. Every downstream
 * consumer - the policy's contact[], contact_feet(), and the InEKF's choice of
 * which foot to anchor - then works from switches that are not the ones being
 * pressed.
 *
 * The assertions below make that substitution a build error rather than a
 * robot that cannot estimate its own velocity.
 */
_Static_assert(NEXUS_CONTACT_L_TOE_BIT  == (1u << NEXUS_CONTACT_L_TOE),
               "L_TOE bit must sit at the L_TOE index");
_Static_assert(NEXUS_CONTACT_L_HEEL_BIT == (1u << NEXUS_CONTACT_L_HEEL),
               "L_HEEL bit must sit at the L_HEEL index");
_Static_assert(NEXUS_CONTACT_R_TOE_BIT  == (1u << NEXUS_CONTACT_R_TOE),
               "R_TOE bit must sit at the R_TOE index");
_Static_assert(NEXUS_CONTACT_R_HEEL_BIT == (1u << NEXUS_CONTACT_R_HEEL),
               "R_HEEL bit must sit at the R_HEEL index");

void contact_init(void)
{
    s_ch[0] = (contact_ch_t){ L_TOE_GPIO_Port,  L_TOE_Pin,  NEXUS_CONTACT_L_TOE_BIT,  0, 0 };
    s_ch[1] = (contact_ch_t){ L_HEEL_GPIO_Port, L_HEEL_Pin, NEXUS_CONTACT_L_HEEL_BIT, 0, 0 };
    s_ch[2] = (contact_ch_t){ R_TOE_GPIO_Port,  R_TOE_Pin,  NEXUS_CONTACT_R_TOE_BIT,  0, 0 };
    s_ch[3] = (contact_ch_t){ R_HEEL_GPIO_Port, R_HEEL_Pin, NEXUS_CONTACT_R_HEEL_BIT, 0, 0 };

    /* Each channel must own exactly one bit, and no two may share one. */
    _Static_assert(NEXUS_NUM_CONTACTS == 4, "the channel table assumes four switches");

    s_switches      = 0;
    s_feet          = 0;
    s_foot_ticks[0] = 0;
    s_foot_ticks[1] = 0;
}

void contact_poll(void)
{
    uint8_t sw = 0;

    for (int i = 0; i < 4; i++)
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

    s_switches = sw;

    /* Either contact point planted means that foot is loaded. */
    uint8_t feet = 0;

    if (sw & (NEXUS_CONTACT_L_TOE_BIT | NEXUS_CONTACT_L_HEEL_BIT))
    {
        feet |= NEXUS_CONTACT_L_FOOT;
    }
    if (sw & (NEXUS_CONTACT_R_TOE_BIT | NEXUS_CONTACT_R_HEEL_BIT))
    {
        feet |= NEXUS_CONTACT_R_FOOT;
    }

    for (int f = 0; f < 2; f++)
    {
        uint8_t now  = (feet & (uint8_t)(NEXUS_CONTACT_L_FOOT << f)) ? 1u : 0u;
        uint8_t prev = (s_feet & (uint8_t)(NEXUS_CONTACT_L_FOOT << f)) ? 1u : 0u;

        if (now != prev)
        {
            s_foot_ticks[f] = 0;
        }
        else if (s_foot_ticks[f] < STABLE_TICKS_MAX)
        {
            s_foot_ticks[f]++;
        }
    }

    s_feet = feet;
}

uint8_t contact_switches(void)
{
    return s_switches;
}

uint8_t contact_feet(void)
{
    return s_feet;
}

uint16_t contact_stable_ticks(uint8_t foot)
{
    return (foot < 2u) ? s_foot_ticks[foot] : 0u;
}
