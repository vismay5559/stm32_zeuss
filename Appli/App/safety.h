#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include "link_proto.h"
#include "health.h"

/*
 * What the robot is allowed to do right now.
 *
 * health.c answers "is each subsystem alive". It does not act on the answer -
 * before this file existed, the only consumer of health_faults() was the red
 * LED, so a Pi that crashed mid-stride left ten actuators servoing to its last
 * command indefinitely while the board blinked code 5 at an empty room.
 *
 * This is the piece that acts. It owns two decisions:
 *
 *   1. whether a command from the Pi may reach the actuators at all, and
 *   2. when to take the actuators away from the Pi and idle them.
 *
 *          BOOT ---- subsystems healthy ------> IDLE
 *                                                | command with ENABLE set,
 *                                                | and a clean re-arm handshake
 *                                                v
 *          FAULT <--- any watched fault ----- ARMED
 *            |                                   ^
 *            +--- faults clear -> IDLE ----------+
 *
 * FAULT is latching by design: clearing the fault returns to IDLE, never
 * straight back to ARMED. Coming back requires the Pi to explicitly drop
 * NEXUS_CMD_ENABLE and raise it again, so a link that flickers cannot hand
 * control back to a policy that has no idea it ever lost it.
 */

typedef enum
{
    SAFETY_BOOT  = 0,
    SAFETY_IDLE  = 1,
    SAFETY_ARMED = 2,
    SAFETY_FAULT = 3
} safety_state_t;

/*
 * Faults that take the actuators away.
 *
 * A stale IMU or a bad encoder degrades the estimate, and the Pi can see that
 * in `health` and back off on its own terms. A dead link or a missed control
 * deadline are different in kind: nobody is flying the robot any more.
 */
#define SAFETY_FATAL_FAULTS  (HEALTH_LINK | HEALTH_TIMING)

/*
 * Command envelope.
 *
 * PLACEHOLDERS - these are deliberately loose bounds chosen to catch garbage
 * (a NaN, a corrupted-but-plausible frame, a policy that has diverged), NOT
 * to describe this robot's real joint travel. Set them from the machine's
 * actual limits, in ODrive turns on the output side, before anything walks.
 * A limit that is wrong in the tight direction stops a healthy robot; one
 * that is wrong in the loose direction only fails to stop a sick one, which
 * is why they start here.
 */
#define SAFETY_POS_MIN_TURNS   (-10.0f)
#define SAFETY_POS_MAX_TURNS   ( 10.0f)

/*
 * Largest jump accepted between two consecutive commands. The Pi commands at
 * 250 Hz, so 0.5 turns per command is 125 turns/s - far beyond any real gait
 * and still tight enough that a garbage value cannot become a full-speed
 * lunge on the next tick.
 */
#define SAFETY_MAX_STEP_TURNS  (0.5f)

/*
 * Largest residual the policy may add to the reference, per joint, in
 * output-shaft turns. 0.1 turns is 36 degrees at the joint - a large
 * correction for a gait whose whole hip swing is about 14 degrees, and still
 * far short of letting the policy drive the leg somewhere of its own choosing.
 *
 * This is checked on the residual ALONE, before it is added. The sum is then
 * checked again against the envelope and the slew limit, so a plausible
 * residual on top of a reference near a limit is still refused.
 */
#define SAFETY_MAX_RESIDUAL_TURNS  (0.1f)

/* Consecutive rejected commands before the link is treated as untrustworthy. */
#define SAFETY_MAX_REJECTS     10u

/*
 * How many enabled commands to wait for every axis to reach closed-loop
 * control before giving up. The Pi commands at 250 Hz, so this is about a
 * second - generous for a state transition the drives complete in
 * milliseconds, and short enough that a drive which is never going to arm is
 * reported rather than waited on forever.
 *
 * Failing to arm is a FAULT, not a quiet refusal: a robot that thinks it is
 * being controlled while half its axes are idle is worse than one that stops.
 */
#define SAFETY_ARM_TIMEOUT_CMDS  250u

/*
 * Get the safety system ready. Call once at startup. The robot begins unable
 * to move, and has to earn its way to being allowed.
 */
void safety_init(void);

/*
 * Run once per tick, after health_tick(). Drives the state machine and idles
 * the actuators on entry to FAULT.
 */
/*
 * Reconsider whether the robot should be allowed to move. Call once per
 * heartbeat, passing whatever health.c currently reports as broken.
 *
 * The robot moves through four states. It starts at BOOT and waits for
 * everything to be working. Once it is, it sits at IDLE - healthy, but still
 * refusing to move, until the Pi explicitly asks it to start. Then it is
 * ARMED and will obey instructions. If the Pi goes quiet, or the robot falls
 * behind its own heartbeat, it drops to FAULT and stops moving.
 *
 * Leaving FAULT is deliberate: someone has to clear the problem. The robot
 * never quietly decides it is fine again and starts moving on its own.
 */
void safety_tick(uint32_t faults);

/*
 * Validate one command from the Pi.
 *
 * Returns 1 and fills targets_out if the command may be acted on; 0 if it was
 * rejected, in which case the actuators keep their previous target and the
 * caller must not touch them.
 */
/*
 * Decide whether one instruction from the Pi may reach the motors.
 *
 * Returns 1 and fills in `targets_out` if the instruction is allowed, 0 if it
 * is refused - and nothing reaches a motor on a refusal.
 *
 * An instruction has to clear several hurdles. The robot must be ARMED. The
 * instruction must be newer than the last one, so a repeated or delayed
 * message cannot be acted on twice. Every target must be a real number - a
 * meaningless value getting through would poison the motion permanently. Each
 * target must be within the range the joint can physically reach, and must
 * not be a huge jump from where the joint is now, because a large jump is a
 * lurch rather than a movement.
 *
 * Too many refusals in a row is itself treated as a fault: it means the Pi is
 * sending something the robot cannot follow, and continuing to ignore it
 * quietly would be worse than stopping.
 */
uint8_t safety_accept_command(const nexus_cmd_t *cmd,
                              const float ref_turns[NEXUS_NUM_JOINTS],
                              float targets_out[NEXUS_NUM_JOINTS]);

/* Which of the four states the robot is in right now. */
safety_state_t safety_state(void);

/* Total commands refused since boot, for the console and the packet. */
/*
 * How many instructions have been refused in total. Climbing steadily means
 * the Pi and the robot disagree about what is reasonable - worth
 * investigating even while the robot still appears to be working.
 */
uint32_t safety_rejected(void);

/* "BOOT" / "IDLE" / "ARMED" / "FAULT". */
/* The current state as readable text - "BOOT", "IDLE", "ARMED", "FAULT" -
   for logs and status lines. */
const char *safety_state_name(void);

#endif /* SAFETY_H */
