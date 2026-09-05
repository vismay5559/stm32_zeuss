#ifndef CONTACT_H
#define CONTACT_H

#include <stdint.h>

/*
 * THE FOOT SWITCHES
 *
 * Four switches tell the robot when it is standing on something: toe and heel
 * on each foot. This matters more than it sounds. A foot that is planted is
 * the one thing the robot can be certain is not moving, and the position
 * estimator leans on that to correct itself. Believe a foot is down when it
 * is not and the estimate drifts.
 *
 * A mechanical switch does not close cleanly - it chatters for a few
 * thousandths of a second as the foot lands. Nothing here reports a change
 * until the switch has settled. Landing is believed sooner than lifting,
 * because a foot arriving is more urgent news than a foot leaving.
 *
 * See README.md in this folder for tick, mask and debounce.
 */

/*
 * Get the switches ready. Call once at startup, before contact_poll().
 *
 * Each switch is tied to the slot it reports into. Those slots are numbered
 * differently from the switches' own positions in the list, and mixing the
 * two up once made the left toe invisible and the right heel report two
 * switches at once - so they are written out here one by one, deliberately.
 */
void    contact_init(void);

/*
 * Look at all four switches and update what they are saying. Call once per
 * tick.
 *
 * A change is only believed once the switch has held its new position long
 * enough. Anything briefer is treated as chatter and ignored.
 */
void    contact_poll(void);

/*
 * Which individual switches are pressed, as four yes/no answers:
 * left toe, left heel, right toe, right heel.
 *
 * These have already been through the chatter filtering, so they are what the
 * switches are really saying rather than what they said for an instant.
 */
uint8_t contact_switches(void);

/*
 * Which FEET are on the ground, as two yes/no answers: left and right.
 *
 * A foot counts as down if either of its switches is pressed - a robot on its
 * heel is still standing on that foot.
 */
uint8_t contact_feet(void);

/*
 * How long a foot has been in its current state, counted in ticks (so 1000
 * means one second). Pass 0 for the left foot, 1 for the right.
 *
 * The estimator uses this to hold off on a foot that has only just landed and
 * might still be settling. A foot down for a good while is solid ground worth
 * trusting; a foot down for two milliseconds might be a bounce.
 *
 * Stops climbing once it reaches its maximum rather than wrapping back to
 * zero, so a foot planted for a long time never briefly looks brand new.
 */
uint16_t contact_stable_ticks(uint8_t foot);

#endif /* CONTACT_H */
