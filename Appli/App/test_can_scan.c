#include "test_can_scan.h"
#include "critical.h"
#include <stdio.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* CANSimple: standard ID = node << 5 | command. See act_odrive.c. */
#define CANSCAN_HEARTBEAT  0x01u
#define CANSCAN_ENCODER    0x09u
#define CANSCAN_NODES      64u

typedef struct
{
    uint32_t frames;
    uint32_t heartbeats;
    uint32_t encoders;
    uint32_t error;
    float position;
    uint8_t state;
} node_sample_t;

/* Written by FDCAN interrupts, copied with interrupts masked below. */
static node_sample_t s_node[2][CANSCAN_NODES];
static uint32_t s_bus_frames[2];

static uint32_t le_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void start_bus(FDCAN_HandleTypeDef *h, uint8_t bus)
{
    FDCAN_FilterTypeDef f = {0};

    /* Include a drive still set to node 5 from the ten-joint detour. The
       robot driver discards it, making a wrong ID look like a missing drive. */
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = 0u;
    f.FilterType = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = 0u;
    f.FilterID2 = 0x7FFu;

    if (HAL_FDCAN_ConfigFilter(h, &f) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(h) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u)
            != HAL_OK)
    {
        printf("bus %u: FDCAN setup FAILED\r\n", (unsigned)bus);
    }
}

void canscan_init(void)
{
    memset(s_node, 0, sizeof(s_node));
    memset(s_bus_frames, 0, sizeof(s_bus_frames));
    start_bus(&hfdcan1, 0u);
    start_bus(&hfdcan2, 1u);
    printf("CAN scan: both buses, all node IDs, no drive commands sent\r\n");
}

void canscan_on_rx(FDCAN_HandleTypeDef *hfdcan)
{
    uint8_t bus = (hfdcan->Instance == FDCAN2) ? 1u : 0u;
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[64];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK)
        {
            return;
        }

        s_bus_frames[bus]++;
        if (hdr.IdType != FDCAN_STANDARD_ID)
        {
            continue;
        }

        uint32_t node = (hdr.Identifier >> 5) & 0x3Fu;
        uint32_t cmd = hdr.Identifier & 0x1Fu;
        node_sample_t *sample = &s_node[bus][node];
        sample->frames++;

        if (hdr.DataLength != FDCAN_DLC_BYTES_8)
        {
            continue;
        }
        if (cmd == CANSCAN_HEARTBEAT)
        {
            sample->heartbeats++;
            sample->error = le_u32(data);
            sample->state = data[4];
        }
        else if (cmd == CANSCAN_ENCODER)
        {
            uint32_t bits = le_u32(data);
            sample->encoders++;
            memcpy(&sample->position, &bits, sizeof(bits));
        }
    }
}

void canscan_run(void)
{
    node_sample_t previous[2][CANSCAN_NODES] = {{{0}}};
    uint32_t previous_bus[2] = {0u, 0u};

    for (;;)
    {
        node_sample_t current[2][CANSCAN_NODES];
        uint32_t current_bus[2];

        HAL_Delay(1000);
        uint32_t primask = critical_enter();
        memcpy(current, s_node, sizeof(current));
        memcpy(current_bus, s_bus_frames, sizeof(current_bus));
        critical_exit(primask);

        for (uint8_t bus = 0u; bus < 2u; bus++)
        {
            printf("bus %u (%s): %lu frames/s\r\n", (unsigned)bus,
                   bus == 0u ? "left" : "right",
                   (unsigned long)(current_bus[bus] - previous_bus[bus]));
            previous_bus[bus] = current_bus[bus];

            for (uint8_t node = 1u; node < CANSCAN_NODES; node++)
            {
                node_sample_t *now = &current[bus][node];
                node_sample_t *old = &previous[bus][node];
                uint32_t frames = now->frames - old->frames;

                if (frames == 0u && node > 4u)
                {
                    continue;
                }
                if (frames == 0u)
                {
                    printf("  node %u: SILENT\r\n", (unsigned)node);
                }
                else
                {
                    printf("  node %u: frames=%lu hb=%lu enc=%lu "
                           "state=%u err=0x%08lX pos=%+.4f turns%s\r\n",
                           (unsigned)node, (unsigned long)frames,
                           (unsigned long)(now->heartbeats - old->heartbeats),
                           (unsigned long)(now->encoders - old->encoders),
                           (unsigned)now->state, (unsigned long)now->error,
                           (double)now->position,
                           node > 4u ? " UNEXPECTED NODE" : "");
                }
                *old = *now;
            }
        }
        printf("\r\n");
    }
}
