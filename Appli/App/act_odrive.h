#ifndef ACT_ODRIVE_H
#define ACT_ODRIVE_H

#include <stdint.h>
#include "link_proto.h"

typedef struct
{
    float    pos[NEXUS_NUM_JOINTS];
    float    vel[NEXUS_NUM_JOINTS];
    float    torque[NEXUS_NUM_JOINTS];
    uint32_t axis_error[NEXUS_NUM_JOINTS];
    uint8_t  axis_state[NEXUS_NUM_JOINTS];
    uint8_t  flags[NEXUS_NUM_JOINTS];
} act_telemetry_t;

void act_init(void);
void act_on_rx(uint8_t bus_index);
void act_set_targets(const float target_pos[NEXUS_NUM_JOINTS]);
void act_tick_1khz(void);
void act_get(act_telemetry_t *out);

/* Moves queued frames into the hardware TX FIFO. Must be called often from
   the main loop - see the queue comment in act_odrive.c. */
void act_tx_pump(void);

/* Non-zero means that bus is oversubscribed and frames are being lost. */
uint32_t act_tx_dropped(uint8_t bus);

/* Total frames accepted from a bus. Stops changing when the bus goes quiet. */
uint32_t act_rx_count(uint8_t bus);

#endif /* ACT_ODRIVE_H */
