#ifndef SOFT_START_H
#define SOFT_START_H

#include <stdint.h>
#include "link_proto.h"

/*
 * Getting from wherever the robot is standing into the gait, without a jump.
 *
 * WHY. Arming used to hand the drives ref + residual on the very first
 * command, with the interpolator reaching it within one 4 ms segment. The
 * gait never passes through the standing pose: at the phases the policy may
 * start from, the joints sit 20-35 degrees away from zero (hip pitch +-22,
 * knee 35, ankle -19 at phase 0). So the first armed tick was a step of up to
 * 35 degrees on all eight joints at once, at full stiffness, with the feet on
 * the ground. And rl_policy_node, which refuses to engage unless every joint
 * is already within 5 degrees of the reference, could never engage from a
 * standing robot at all.
 *
 * WHAT. Every arm now goes through a RAMP: the reference handed to the
 * drives blends from the pose measured at the moment of arming to the goal
 * over SOFT_START_RAMP_S, on a smoothstep (zero velocity at both ends). What
 * the goal is depends on NEXUS_CMD_HOLD on the arming command:
 *
 *   HOLD set    the gait clock is frozen at SOFT_START_PHASE, the goal is
 *               that phase's pose, and when the ramp ends the robot HOLDS it -
 *               standing, no feedforward torque - until a command arrives
 *               with HOLD clear. The clock then starts from SOFT_START_PHASE.
 *   HOLD clear  the clock keeps running and the goal is the live gait; the
 *               ramp blends into the stride already in progress, feedforward
 *               scaled in with it, then it is the plain gait (RUN).
 *
 * The residual is IGNORED until RUN. A policy is not driving anything while
 * the robot is being brought to its start pose, and rl_policy_node does not
 * engage until the hold is released.
 *
 * Disarming, for any reason, returns to IDLE, so the next arm ramps again
 * from wherever the robot is by then.
 *
 * BACK TO STANDING (v12, NEXUS_CMD_RETURN). Brought to the start pose and
 * then not wanted after all: the way back down used to be ENABLE off, which
 * drops every joint at once from a 35 degree knee bend. RETURN instead ramps,
 * over the same SOFT_START_RAMP_S smoothstep, from wherever the reference is
 * now to ZERO on every joint - the calibrated standing pose, ankles flat as
 * they were zeroed - and holds it there (HOME). Then ENABLE off idles the
 * drives from a straight stand. Zero, not the pose measured at arming: that
 * was the first version, and it brought the robot back to however it happened
 * to be slumped when START was pressed (a hip at 4, a knee at 9 degrees).
 * Only from a held arm - the held ramp or HOLD. Not from RUN: mid-stride with
 * a foot in the air there is no safe straight line back, so a walking robot
 * is stopped with ENABLE off. Once begun it is final until disarm; clearing
 * the bit does not turn the robot round again.
 *
 * HOME TRIM. Holding zero with the trained PD alone left the hips 4.5 deg
 * short, pushing a steady 12 Nm (150 Nm/rad x 4.5 deg) and not moving: the
 * gearboxes are not backdrivable and their breakaway friction is above what
 * the PD makes at that error (the training model had 0.05 Nm). So in HOME
 * only, each joint stuck more than SOFT_TRIM_DEADBAND away from zero has its
 * target nudged further past zero, at SOFT_TRIM_RATE x error, up to
 * SOFT_TRIM_MAX, until friction breaks; the instant it crosses zero the trim
 * is dropped, so it brakes there instead of hunting back and forth. Not the
 * ankles: they are backdrivable, and an ankle off zero in HOME is the floor
 * holding the foot, which more push would only tip the robot against. Not in
 * HOLD or RUN: the policy expects the PD it was trained with.
 */
#define SOFT_TRIM_DEADBAND  (0.5f / 360.0f)   /* turns */
#define SOFT_TRIM_RATE      1.0f              /* 1/s: trim grows err per second */
#define SOFT_TRIM_MAX       (8.0f / 360.0f)   /* turns */
/* Full trim for this long without half a degree of progress = jammed, not
   friction: the trim is dropped for that joint until the next arm. */
#define SOFT_TRIM_GIVEUP_TICKS  2000u          /* 2 s */

#define SOFT_START_RAMP_S   3.0f
/*
 * 0.95 - back to it, 2026-11-24 evening, after trying 0.15 and 0.10 the same
 * day. The start pose is reached by a ramp UNDER LOAD: stiff hips and knees
 * (KP 150) bend on schedule while the trained ankles (KP 30) cannot push the
 * robot's weight forward over its feet, so the shins stay upright, the hips go
 * back, and it sits onto its heels. How far depends on how much ankle bend the
 * pose asks for - measured on the robot:
 *
 *   phase   ankle bend L / R    lean back in the hold
 *   0.95     -8 / -6 deg          ~6 deg   (the first ground walk, 4 s)
 *   0.15    -22 / -13             12-19    (right ankle pushed 8 deg short)
 *   0.10    -28 / -16             13-16, toppled (right ankle 7-9 deg short)
 *
 * 0.95 asks least of the ankles, and it is the only start the robot has
 * walked from. Its known cost: feet 276 mm apart, and a 6 deg lean back lifts
 * the front (right) foot off its switch. Inside the policy's start window; the
 * closed-loop sim check walks 20 s from rest there (tilt p95 5.2 deg).
 */
#define SOFT_START_PHASE    0.95f

typedef enum
{
    SOFT_IDLE = 0,      /* not armed                                         */
    SOFT_RAMP = 1,      /* blending from the arming pose to the goal         */
    SOFT_HOLD = 2,      /* at the start pose, clock frozen, waiting          */
    SOFT_RUN  = 3,      /* the gait, unmodified                              */
    SOFT_RETURN = 4,    /* ramping to zero on every joint                    */
    SOFT_HOME = 5       /* holding zero on every joint                       */
} soft_mode_t;

/* Back to IDLE: call at boot and whenever the board is not armed. */
void soft_start_idle(void);

/*
 * The board has just armed. from[] is the measured pose (output turns) the
 * first target was built from; hold is NEXUS_CMD_HOLD on that command.
 */
void soft_start_armed(const float from[NEXUS_NUM_JOINTS], uint8_t hold);

/*
 * NEXUS_CMD_HOLD as it stands on each accepted command. Clearing it releases
 * a HOLD into RUN; clearing it during the ramp means "run when the ramp ends".
 * Setting it once running changes nothing: to stop, send ENABLE off.
 */
void soft_start_hold_flag(uint8_t hold);

/*
 * One 1 kHz tick while armed. Advances the ramp and fills:
 *
 *   out_ref    the reference to hand to safety_accept_command (output turns)
 *   out_vel    velocity feedforward (output turns/s)
 *   out_tau    torque feedforward (output Nm)
 *
 * from the gait's own values for this tick:
 *
 *   gait_ref   drive reference at the current phase, pre-deflected (turns)
 *   gait_vel   gait velocity (turns/s)
 *   gait_tau   gait torque (Nm)
 *   hold_pose  the drive reference at SOFT_START_PHASE, pre-deflected exactly
 *              as gait_ref is there (turns)
 *   hold_tau   the gait's torque at SOFT_START_PHASE (Nm)
 *
 * The hold carries SOFT_HOLD_TAU_SCALE * hold_tau - and the scale is 0.
 *
 * Tried, measured, and switched off the same day (2026-11-24). Feeding the
 * gait's own torque at the start phase was meant to stop the hold sagging
 * under the robot's weight. But those are the optimiser's torques for a robot
 * MID-STRIDE with a particular foot carrying it: at phase 0.10 that is -13.2 Nm
 * on the right hip roll (the right leg holding the body up). On the robot the
 * right foot was in the air in every hold, so the -13 Nm just shoved the free
 * leg out: commanded +3.8 deg, held at -5.0, drive torque -4 Nm = PD +9 plus
 * feed-forward -13. WALK could never be released. Zero: the hold is the PD on
 * the start pose, as on the first ground walk, and the gait's torque arrives
 * with the gait. Kept as a scale rather than deleted, so a gravity-only
 * feed-forward can be tried in its place.
 */
#define SOFT_HOLD_TAU_SCALE  0.0f

/*
 * 1 = in HOLD the ankles' target follows the measured ankle angle, so they
 * have no stiffness, only the drive's damping (KD 1.2 Nm s/rad): the feet
 * settle flat under the robot's weight and the builder can set them by hand.
 * WALK drives them to the gait again - the release check allows the ankles
 * 10 deg from the start pose. Asked for 2026-11-25: holding the start pose
 * with powered ankles, the robot sat back on its heels with the front (right)
 * foot half off the floor, and WALK could not release.
 */
/* 0 since 2026-11-25: back to the first ground walk's configuration (see
   g_friction_ff_nm); to be re-tested on its own. */
#define SOFT_HOLD_ANKLES_FREE  0

/*
 * How far a free ankle may wander from the start pose before the PD holds it.
 * Free without a limit, the right ankle settled 15.6 deg off (2026-11-25,
 * +9.9 against -5.7) and WALK - which allows ankles 10 deg - never released.
 * 8 deg keeps it inside that with margin.
 */
#define SOFT_HOLD_ANKLE_WINDOW  (8.0f / 360.0f)   /* turns */

void soft_start_tick(const float gait_ref[NEXUS_NUM_JOINTS],
                     const float gait_vel[NEXUS_NUM_JOINTS],
                     const float gait_tau[NEXUS_NUM_JOINTS],
                     const float hold_pose[NEXUS_NUM_JOINTS],
                     const float hold_tau[NEXUS_NUM_JOINTS],
                     float out_ref[NEXUS_NUM_JOINTS],
                     float out_vel[NEXUS_NUM_JOINTS],
                     float out_tau[NEXUS_NUM_JOINTS]);

/*
 * NEXUS_CMD_RETURN as it stands on each accepted command. Set while armed -
 * a ramp, the hold, or the running gait - it starts a 3 s ramp to zero on
 * every joint: from the last reference, or from the MEASURED pose out of RUN.
 * Already returning or home: nothing. (Out of IDLE the Pi arms with RETURN
 * set, and the arm's ramp turns into the return on the same command.)
 * Until 2026-11-24 it worked only from a held ramp or the hold.
 */
void soft_start_return_flag(uint8_t ret);

/*
 * The pose the drives report this tick (output turns), and whether all of it
 * is fresh. Used only by the HOME trim; with ok 0 the trim holds still.
 */
void soft_start_measured(const float meas[NEXUS_NUM_JOINTS], uint8_t ok);

soft_mode_t soft_start_mode(void);

/* 1 while the gait clock must stay frozen at SOFT_START_PHASE. */
uint8_t soft_start_clock_frozen(void);

/* 1 while the residual is to be ignored (everything but RUN). */
uint8_t soft_start_ignores_residual(void);

/* Ramp progress, 0..1 (1 outside RAMP). */
float soft_start_progress(void);

#endif /* SOFT_START_H */
