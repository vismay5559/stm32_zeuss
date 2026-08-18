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

void act_init(void);
void act_on_rx(uint8_t bus_index);
void act_set_targets(const float target_pos[NEXUS_NUM_JOINTS]);
void act_tick_1khz(void);
void act_get(act_telemetry_t *out);

/*
 * Stop commanding position and ask every axis to go to IDLE.
 *
 * Idempotent and safe to call every tick while faulted; the request is only
 * re-sent periodically, not at 1 kHz. After this, act_tick_1khz() emits
 * nothing until a new target arrives.
 */
void act_disarm(void);

/* Non-zero while position commands are being sent. */
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
void act_request_arm(void);

/* Non-zero once every axis reports CLOSED_LOOP_CONTROL on its heartbeat. */
uint8_t act_all_closed_loop(void);

/* Bitmask of joints NOT in closed loop, for diagnostics. */
uint16_t act_not_closed_loop_mask(void);

/*
 * Poll both buses for protocol errors and drive bus-off recovery.
 *
 * Call once per tick. A bus that reaches bus-off stays there forever
 * otherwise: nothing in the driver noticed, and the only symptom was
 * act_rx_count() quietly ceasing to change.
 */
void act_bus_service(void);

/* Times each bus has entered bus-off since boot. */
uint32_t act_bus_off_count(uint8_t bus);

/* Transmit error counter, the leading indicator that a bus is failing. */
uint8_t act_tx_error_count(uint8_t bus);

/* Moves queued frames into the hardware TX FIFO. Must be called often from
   the main loop - see the queue comment in act_odrive.c. */
void act_tx_pump(void);

/* Non-zero means that bus is oversubscribed and frames are being lost. */
uint32_t act_tx_dropped(uint8_t bus);

/* Total frames accepted from a bus. Stops changing when the bus goes quiet. */
uint32_t act_rx_count(uint8_t bus);

#endif /* ACT_ODRIVE_H */
