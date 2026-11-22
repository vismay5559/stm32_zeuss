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

#define SOFT_START_RAMP_S   3.0f
/*
 * 0.95: double support, both feet down, right foot ~27 cm ahead of the left
 * (the gait library's own sole positions and contact flags at 0.30 m/s).
 * Was 0.0, where the left foot is already lifting - a pose a robot cannot
 * stand in on the floor. Inside the policy's start window (0.95-0.30); the
 * closed-loop sim check walks 20 s from rest at 0.95 (tilt p95 5.2 deg),
 * against 4.7 at phase 0 (sim_check/RESULTS.md in zeus_8dof_deploy).
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
 *   hold_pose  the pure reference at SOFT_START_PHASE (turns, no pre-deflection,
 *              since nothing is pushing through the springs while standing)
 */
void soft_start_tick(const float gait_ref[NEXUS_NUM_JOINTS],
                     const float gait_vel[NEXUS_NUM_JOINTS],
                     const float gait_tau[NEXUS_NUM_JOINTS],
                     const float hold_pose[NEXUS_NUM_JOINTS],
                     float out_ref[NEXUS_NUM_JOINTS],
                     float out_vel[NEXUS_NUM_JOINTS],
                     float out_tau[NEXUS_NUM_JOINTS]);

/*
 * NEXUS_CMD_RETURN as it stands on each accepted command. Set during a held
 * ramp or HOLD: start ramping to zero on every joint. Anywhere else it does
 * nothing - in particular not in RUN.
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
