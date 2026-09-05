#ifndef ACT_ODRIVE_H
#define ACT_ODRIVE_H

#include <stdint.h>
#include "link_proto.h"

/* ODrive axis states, from CANSimple's Heartbeat and Set_Axis_State. */
#define ODRV_AXIS_STATE_UNDEFINED            0u
#define ODRV_AXIS_STATE_IDLE                 1u
#define ODRV_AXIS_STATE_CLOSED_LOOP_CONTROL  8u

/*
 * How long a joint's position may go unheard before it stops counting as a
 * measurement, in 1 kHz ticks.
 *
 * This MUST be looser than the drives' configured encoder message rate
 * (ODrive's `axis.config.can.encoder_msg_rate_ms`) or every joint reads as
 * stale forever. It must also be far tighter than the 200 ms at which
 * health.c notices a bus has gone completely silent: the danger here is not a
 * dead bus, it is one joint quietly repeating its last value while the
 * estimator anchors a contact on it.
 */
#define ACT_POS_STALE_TICKS  100u

typedef struct
{
    float    pos[NEXUS_NUM_JOINTS];
    float    vel[NEXUS_NUM_JOINTS];
    float    torque[NEXUS_NUM_JOINTS];
    uint32_t axis_error[NEXUS_NUM_JOINTS];
    uint8_t  axis_state[NEXUS_NUM_JOINTS];
    uint8_t  flags[NEXUS_NUM_JOINTS];

    /*
     * Ticks since this joint last reported a position, saturating.
     *
     * flags[] carries NEXUS_ACT_TELEM_FRESH, but act_get() clears it every
     * tick - so it answers "did a frame arrive in the last millisecond",
     * which flaps on any drive that reports slower than 1 kHz. Anything
     * deciding whether to TRUST a position wants this instead.
     */
    uint16_t pos_age[NEXUS_NUM_JOINTS];
} act_telemetry_t;

/*
 * Get the motors ready to talk to. Call once at startup, before anything else
 * here. The motors are not switched on for movement at this point.
 */
void act_init(void);
/*
 * The hardware calls this by itself whenever a motor sends something back.
 * Nothing else should call it. `bus_index` says which of the two wires it
 * came in on.
 */
void act_on_rx(uint8_t bus_index);
/*
 * Tell every joint where to go, one target per joint.
 *
 * Do not call this directly with something the Pi sent - safety.c checks
 * instructions first, and this function trusts what it is given completely.
 */
void act_set_targets(const float target_pos[NEXUS_NUM_JOINTS]);
/*
 * The regular motor work: send out the current targets and ask for fresh
 * readings back. Call once per heartbeat.
 */
void act_tick_1khz(void);
/*
 * Copy out the newest readings from every motor - where each joint is, how
 * fast it is moving, how hard it is working, and whether it is reporting a
 * problem.
 *
 * Interrupts are held off for the moment it takes to copy, so the readings
 * all describe the same instant rather than being stitched together from two.
 */
void act_get(act_telemetry_t *out);

/*
 * Stop commanding position and ask every axis to go to IDLE.
 *
 * Idempotent and safe to call every tick while faulted; the request is only
 * re-sent periodically, not at 1 kHz. After this, act_tick_1khz() emits
 * nothing until a new target arrives.
 */
/*
 * Tell the motors to stop holding position and go limp. The robot will sag
 * under its own weight, so this is for a machine on a stand or already on the
 * ground, not one standing up.
 */
void act_disarm(void);

/* Non-zero while position commands are being sent. */
/* Whether the motors are currently switched on for movement. */
uint8_t act_is_armed(void);

/*
 * Last-resort disarm for fault handlers.
 *
 * Pushes Set_Axis_State(IDLE) straight into the hardware TX FIFOs with
 * bounded spins, bypassing the software queue and the main loop entirely, so
 * it works with interrupts disabled and a wedged application. Best-effort by
 * nature - it cannot confirm the drives heard it - but a CAN frame that
 * probably lands beats ten actuators holding torque for certain.
 */
/*
 * Stop the motors immediately, from anywhere, including the emergency
 * shutdown path.
 *
 * Safe to call before the motors have even been set up - it checks first,
 * because something failing very early in startup would otherwise crash
 * inside the very code meant to handle the failure.
 */
void act_emergency_idle(void);

/*
 * Ask every axis to enter closed-loop control, and clear any latched errors
 * first.
 *
 * The firmware used to do neither: it sent position commands and assumed ten
 * drives had been put into closed loop by hand before power-on. That is a
 * reasonable bench posture and an untenable one for a machine that is supposed
 * to come up on its own - and it meant an axis that tripped mid-run could only
 * be recovered by someone with a laptop.
 *
 * Idempotent; the request is repeated at a low rate until the axes report they
 * arrived. Errors are cleared only here, on an explicit arm - never
 * automatically, because an error that clears itself is an error nobody
 * investigates.
 */
/*
 * Ask the motors to switch on for movement.
 *
 * A motor that has previously tripped refuses to start until its recorded
 * error is cleared, so this clears them - once, here, on a deliberate
 * request. Errors are never cleared automatically, so "something went wrong"
 * stays visible until a person, or the Pi, actually asks to start again.
 */
void act_request_arm(void);

/* Non-zero once every axis reports CLOSED_LOOP_CONTROL on its heartbeat. */
/*
 * Whether every motor has confirmed it is actively holding its position.
 *
 * Asking a motor to switch on and it agreeing are two different things. The
 * robot should not be treated as ready to move until this says yes.
 */
uint8_t act_all_closed_loop(void);

/* Bitmask of joints NOT in closed loop, for diagnostics. */
/*
 * Which specific motors have NOT confirmed they are holding position - one
 * yes/no per joint. Useful when the robot refuses to start and you need to
 * know which joint is the hold-up.
 */
uint16_t act_not_closed_loop_mask(void);

/*
 * Poll both buses for protocol errors and drive bus-off recovery.
 *
 * Call once per tick. A bus that reaches bus-off stays there forever
 * otherwise: nothing in the driver noticed, and the only symptom was
 * act_rx_count() quietly ceasing to change.
 */
/*
 * Check on the health of the two wires the motors are on, and recover a wire
 * that has shut itself down. Call once per heartbeat.
 *
 * A wire that sees too many errors switches itself off completely and will
 * not come back on its own. Without this, the only visible symptom would be
 * motors that quietly stopped replying.
 */
void act_bus_service(void);

/* Times each bus has entered bus-off since boot. */
/*
 * How many times this wire has shut itself down from too many errors. Should
 * be zero. Anything else points at wiring, connectors or interference rather
 * than at the software.
 */
uint32_t act_bus_off_count(uint8_t bus);

/* Transmit error counter, the leading indicator that a bus is failing. */
/*
 * How unhappy this wire is right now, as a running score kept by the
 * hardware. It climbs on failures and falls on successes, so a number sitting
 * steadily above zero means messages are still getting through but the wire
 * is struggling - an early warning before it shuts down entirely.
 */
uint8_t act_tx_error_count(uint8_t bus);

/* Moves queued frames into the hardware TX FIFO. Must be called often from
   the main loop - see the queue comment in act_odrive.c. */
/*
 * Move queued messages out to the motors. Call often - much more than once
 * per heartbeat.
 *
 * The hardware can only hold a few messages at a time, so the rest wait in a
 * queue. Calling this frequently keeps that queue draining, which is what
 * gets each heartbeat's messages onto the wire inside that heartbeat.
 */
void act_tx_pump(void);

/* Non-zero means that bus is oversubscribed and frames are being lost. */
/*
 * How many messages were thrown away because the queue was full. Anything
 * other than zero means instructions are not reaching the motors.
 */
uint32_t act_tx_dropped(uint8_t bus);

/* Total frames accepted from a bus. Stops changing when the bus goes quiet. */
/*
 * How many replies have arrived on this wire in total. If this stops
 * climbing, the motors on that wire have gone silent.
 */
uint32_t act_rx_count(uint8_t bus);

#endif /* ACT_ODRIVE_H */
