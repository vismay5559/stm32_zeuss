#include "act_odrive.h"
#include "critical.h"
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

/*
 * Classic CAN 2.0. Measured on hardware: the S1 sends CLASSIC frames and will
 * not acknowledge FD ones, which drives our transmit error counter to bus-off
 * while reception still looks perfect. Flip both back to FD only after the leg
 * test's frame trace shows the drive actually sending FD.
 */
#define ODRV_TX_FORMAT            FDCAN_CLASSIC_CAN
#define ODRV_TX_BRS               FDCAN_BRS_OFF

/*
 * The FDCAN hardware transmit FIFO on this part is fixed at three entries
 * (SRAMCAN_TFQ_NBR in stm32h7rsxx_hal_fdcan.c - it is not configurable).
 * Each 1 kHz tick wants to send five frames per bus, so handing all five
 * straight to the hardware silently loses two of them and nodes 4 and 5
 * would never receive a command.
 *
 * So frames go into a software queue first, and act_tx_pump() moves them into
 * the hardware FIFO as space frees up. At ~50 us per frame on the wire, five
 * frames drain in ~250 us, comfortably inside one tick.
 *
 * Both the producer (act_tick_1khz) and the consumer (act_tx_pump) run in
 * main-loop context, never in an ISR, so the queue needs no locking.
 */
#define CAN_TXQ_LEN   16u    /* must be a power of two */
#define CAN_TXQ_MASK  (CAN_TXQ_LEN - 1u)

typedef struct
{
    uint32_t identifier;
    uint8_t  data[8];
} can_tx_frame_t;

static can_tx_frame_t s_txq[2][CAN_TXQ_LEN];
static uint8_t        s_txq_head[2];
static uint8_t        s_txq_tail[2];
static uint32_t       s_tx_dropped[2];

/* Frames accepted from each bus. Only ever increments, so a health check can
   detect "this bus has gone quiet" by watching it stop changing. */
static volatile uint32_t s_rx_count[2];

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

static void tx_enqueue(uint8_t bus, uint32_t identifier, const uint8_t *data)
{
    uint8_t next = (uint8_t)((s_txq_head[bus] + 1u) & CAN_TXQ_MASK);

    if (next == s_txq_tail[bus])
    {
        /* Queue full: the bus is not keeping up. Drop the frame rather than
           block, and count it - a non-zero value here means the bus is
           oversubscribed and telemetry rates need to come down. */
        s_tx_dropped[bus]++;
        return;
    }

    s_txq[bus][s_txq_head[bus]].identifier = identifier;
    memcpy(s_txq[bus][s_txq_head[bus]].data, data, 8);
    s_txq_head[bus] = next;
}

void act_tx_pump(void)
{
    FDCAN_TxHeaderTypeDef hdr;

    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = FDCAN_DLC_BYTES_8;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = ODRV_TX_BRS;
    hdr.FDFormat            = ODRV_TX_FORMAT;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;

    for (uint8_t bus = 0; bus < 2u; bus++)
    {
        FDCAN_HandleTypeDef *h = bus_handle(bus);

        while (s_txq_tail[bus] != s_txq_head[bus])
        {
            if (HAL_FDCAN_GetTxFifoFreeLevel(h) == 0u)
            {
                break;   /* hardware FIFO full, try again next pass */
            }

            hdr.Identifier = s_txq[bus][s_txq_tail[bus]].identifier;

            if (HAL_FDCAN_AddMessageToTxFifoQ(h, &hdr,
                                              s_txq[bus][s_txq_tail[bus]].data) != HAL_OK)
            {
                break;   /* leave it queued and retry rather than losing it */
            }

            s_txq_tail[bus] = (uint8_t)((s_txq_tail[bus] + 1u) & CAN_TXQ_MASK);
        }
    }
}

uint32_t act_tx_dropped(uint8_t bus)
{
    return (bus < 2u) ? s_tx_dropped[bus] : 0u;
}

uint32_t act_rx_count(uint8_t bus)
{
    return (bus < 2u) ? s_rx_count[bus] : 0u;
}

void act_init(void)
{
    memset(&s_telem, 0, sizeof(s_telem));
    memset(s_txq, 0, sizeof(s_txq));
    memset(s_txq_head, 0, sizeof(s_txq_head));
    memset(s_txq_tail, 0, sizeof(s_txq_tail));
    memset(s_tx_dropped, 0, sizeof(s_tx_dropped));
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

        s_rx_count[bus_index & 1u]++;

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
    uint8_t  data[8];
    uint32_t bits;

    /* ODrive packs vel_ff as int16 in units of 0.001 turns/s, so the field
       saturates at +/-32.7 turns/s. Clamp before the cast: a C cast of an
       out-of-range float to int16 wraps sign, which would turn a large
       positive feedforward into a large negative one. */
    float    vff_milli = vel_ff * 1000.0f;
    int16_t  vff;

    if (vff_milli > 32767.0f)
    {
        vff = 32767;
    }
    else if (vff_milli < -32768.0f)
    {
        vff = -32768;
    }
    else
    {
        vff = (int16_t)vff_milli;
    }

    memcpy(&bits, &pos, sizeof(bits));
    data[0] = (uint8_t)(bits & 0xFFu);
    data[1] = (uint8_t)((bits >> 8) & 0xFFu);
    data[2] = (uint8_t)((bits >> 16) & 0xFFu);
    data[3] = (uint8_t)((bits >> 24) & 0xFFu);
    data[4] = (uint8_t)((uint16_t)vff & 0xFFu);
    data[5] = (uint8_t)(((uint16_t)vff >> 8) & 0xFFu);
    data[6] = 0;
    data[7] = 0;

    tx_enqueue(bus, (node << 5) | ODRV_CMD_SET_INPUT_POS, data);
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

    /* Start draining immediately; app_run() keeps pumping for the rest of the tick. */
    act_tx_pump();
}

void act_get(act_telemetry_t *out)
{
    /* s_telem is written by act_on_rx() from the FDCAN ISRs, so the copy and
       the flag clear have to be one indivisible step. Otherwise an ISR landing
       mid-copy yields a mix of two different ticks, and a freshness flag set
       between the copy and the clear is lost forever. */
    uint32_t primask = critical_enter();

    *out = s_telem;

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_telem.flags[j] = 0;
    }

    critical_exit(primask);
}
