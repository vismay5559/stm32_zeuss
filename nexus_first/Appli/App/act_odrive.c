#include "act_odrive.h"
#include "main.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* ODrive CANSimple: arbitration id is (node_id << 5) | cmd_id. */
#define ODRV_CMD_HEARTBEAT        0x001u
#define ODRV_CMD_GET_ENCODER      0x009u
#define ODRV_CMD_SET_INPUT_POS    0x00Cu
#define ODRV_CMD_GET_TORQUES      0x01Cu

#define ODRV_NODES_PER_BUS        5
#define ODRV_FILTER_ID_LOW        0x020u   /* node 1, cmd 0  */
#define ODRV_FILTER_ID_HIGH       0x0BFu   /* node 5, cmd 31 */

/* Commands arrive at 250 Hz and are interpolated across the 1 kHz ticks between them. */
#define CMD_SEGMENT_TICKS         4u

/* ODrive expects CAN-FD frames; flip to FDCAN_CLASSIC_CAN if an axis is not in FD mode. */
#define ODRV_TX_FORMAT            FDCAN_FD_CAN
#define ODRV_TX_BRS               FDCAN_BRS_ON

static act_telemetry_t s_telem;

static float    s_seg_start[NEXUS_NUM_JOINTS];
static float    s_seg_end[NEXUS_NUM_JOINTS];
static float    s_out[NEXUS_NUM_JOINTS];
static float    s_prev_out[NEXUS_NUM_JOINTS];
static uint32_t s_seg_tick;
static uint8_t  s_have_target;

static FDCAN_HandleTypeDef *bus_handle(uint8_t bus)
{
    return (bus == 0u) ? &hfdcan1 : &hfdcan2;
}

static float le_f32(const uint8_t *p)
{
    uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float f;
    memcpy(&f, &v, sizeof(f));
    return f;
}

static uint32_t le_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void bus_setup(FDCAN_HandleTypeDef *h)
{
    FDCAN_FilterTypeDef f;

    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = ODRV_FILTER_ID_LOW;
    f.FilterID2    = ODRV_FILTER_ID_HIGH;

    HAL_FDCAN_ConfigFilter(h, &f);
    HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(h);
    HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void act_init(void)
{
    memset(&s_telem, 0, sizeof(s_telem));
    memset(s_seg_start, 0, sizeof(s_seg_start));
    memset(s_seg_end, 0, sizeof(s_seg_end));
    memset(s_out, 0, sizeof(s_out));
    memset(s_prev_out, 0, sizeof(s_prev_out));

    s_seg_tick    = CMD_SEGMENT_TICKS;
    s_have_target = 0;

    bus_setup(&hfdcan1);
    bus_setup(&hfdcan2);
}

void act_on_rx(uint8_t bus_index)
{
    FDCAN_HandleTypeDef *h = bus_handle(bus_index);
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[64];

    while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK)
        {
            return;
        }

        uint32_t node = (hdr.Identifier >> 5) & 0x3Fu;
        uint32_t cmd  = hdr.Identifier & 0x1Fu;

        if ((node < 1u) || (node > (uint32_t)ODRV_NODES_PER_BUS))
        {
            continue;
        }

        int j = (int)(bus_index * ODRV_NODES_PER_BUS) + (int)(node - 1u);

        switch (cmd)
        {
        case ODRV_CMD_GET_ENCODER:
            s_telem.pos[j] = le_f32(&data[0]);
            s_telem.vel[j] = le_f32(&data[4]);
            s_telem.flags[j] |= NEXUS_ACT_TELEM_FRESH;
            break;

        case ODRV_CMD_GET_TORQUES:
            s_telem.torque[j] = le_f32(&data[4]);
            s_telem.flags[j] |= NEXUS_ACT_TELEM_FRESH;
            break;

        case ODRV_CMD_HEARTBEAT:
            s_telem.axis_error[j] = le_u32(&data[0]);
            s_telem.axis_state[j] = data[4];
            s_telem.flags[j] |= NEXUS_ACT_HB_FRESH;
            break;

        default:
            break;
        }
    }
}

void act_set_targets(const float target_pos[NEXUS_NUM_JOINTS])
{
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        if (!s_have_target)
        {
            s_out[j]      = target_pos[j];
            s_prev_out[j] = target_pos[j];
        }
        /* Anchor each segment at where output actually is, so late or jittery
           command frames do not accumulate position error. */
        s_seg_start[j] = s_out[j];
        s_seg_end[j]   = target_pos[j];
    }

    s_seg_tick    = 0;
    s_have_target = 1;
}

static void send_input_pos(uint8_t bus, uint32_t node, float pos, float vel_ff)
{
    FDCAN_TxHeaderTypeDef hdr;
    uint8_t  data[8];
    uint32_t bits;
    int16_t  vff = (int16_t)(vel_ff * 1000.0f);

    memcpy(&bits, &pos, sizeof(bits));
    data[0] = (uint8_t)(bits & 0xFFu);
    data[1] = (uint8_t)((bits >> 8) & 0xFFu);
    data[2] = (uint8_t)((bits >> 16) & 0xFFu);
    data[3] = (uint8_t)((bits >> 24) & 0xFFu);
    data[4] = (uint8_t)((uint16_t)vff & 0xFFu);
    data[5] = (uint8_t)(((uint16_t)vff >> 8) & 0xFFu);
    data[6] = 0;
    data[7] = 0;

    hdr.Identifier          = (node << 5) | ODRV_CMD_SET_INPUT_POS;
    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = FDCAN_DLC_BYTES_8;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = ODRV_TX_BRS;
    hdr.FDFormat            = ODRV_TX_FORMAT;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(bus_handle(bus), &hdr, data);
}

void act_tick_1khz(void)
{
    if (!s_have_target)
    {
        return;
    }

    if (s_seg_tick < CMD_SEGMENT_TICKS)
    {
        s_seg_tick++;
    }

    float alpha = (float)s_seg_tick / (float)CMD_SEGMENT_TICKS;

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_prev_out[j] = s_out[j];
        s_out[j]      = s_seg_start[j] + ((s_seg_end[j] - s_seg_start[j]) * alpha);

        float vel_ff = (s_out[j] - s_prev_out[j]) * 1000.0f;

        uint8_t  bus  = (uint8_t)(j / ODRV_NODES_PER_BUS);
        uint32_t node = (uint32_t)(j % ODRV_NODES_PER_BUS) + 1u;

        send_input_pos(bus, node, s_out[j], vel_ff);
    }
}

void act_get(act_telemetry_t *out)
{
    *out = s_telem;

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_telem.flags[j] = 0;
    }
}
