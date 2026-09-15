#ifndef LEG_STREAM_H
#define LEG_STREAM_H

#include <stdint.h>
#include "link_proto.h"

/*
 * The single-leg test's data, packed into the robot's USB state packet.
 *
 * Robot mode is the only thing that sends nexus_state_t to the Pi. The leg test
 * used to send nothing, so on the bench the USB cable could enumerate but never
 * carry a byte - and there was no way to watch a leg run except by printing a
 * CSV over the ST-LINK console and plotting it afterwards.
 *
 * With this, the leg test sends the SAME 444-byte packet at 1 kHz. Everything
 * on the Pi side works unchanged: link_check, link_node, /zeus/state, Rerun.
 *
 * Only what the leg test actually has is filled in:
 *
 *   joint_pos, joint_vel   from the drive's encoder estimate, rad at the output
 *   ref_angle              the target last sent to the drive, rad at the output
 *   act_torque/error/state straight from the drive
 *   phase                  the gait's position in its cycle
 *
 * at the joint-map index of each drive (bus 0: index = node - 1). Everything
 * else reads as absent, not as a plausible zero: quaternions are identity,
 * foot_z is NaN with fk_valid clear, fused_valid is INVALID, and stream_flags
 * carries NEXUS_STREAM_LEG_TEST so a reader can tell this packet from the
 * robot's.
 *
 * Pure packing, no hardware: tools/hosttest/test_leg_stream.c checks it.
 */

typedef struct
{
    uint8_t  node;          /* CAN node id on bus 0 (FDCAN1), 1..5          */
    float    scale;         /* encoder turns per output turn (s_cmd_scale)  */
    float    pos_turns;     /* drive's position estimate, encoder turns     */
    float    vel_turns_s;   /* drive's velocity estimate, encoder turns/s   */
    float    cmd_turns;     /* last target sent to the drive, encoder turns */
    float    torque;        /* Nm, drive's estimate                         */
    uint32_t axis_error;
    uint8_t  axis_state;
    uint8_t  live;          /* 0 = not on the bus: left out of the packet   */
    uint8_t  fresh;         /* telemetry heard recently                     */
} leg_stream_joint_t;

typedef struct
{
    uint32_t seq;           /* one per processed control tick               */
    uint32_t timestamp_us;  /* TIM2, 1 MHz                                  */
    float    gait_phase;    /* cycles; the fraction goes out as `phase`     */
    uint8_t  gait_running;
    uint8_t  stopped;       /* hard stop, button, or fault: reads as FAULT  */
    uint32_t can_dropped;
    uint32_t can_bus_off;
} leg_stream_status_t;

/*
 * Fill every field of `st` except sync, msg_id, version and crc, which
 * link_usb_send_state() sets. `joints` has `count` entries.
 */
void leg_stream_fill(nexus_state_t *st,
                     const leg_stream_joint_t *joints, int count,
                     const leg_stream_status_t *status);

#endif /* LEG_STREAM_H */
