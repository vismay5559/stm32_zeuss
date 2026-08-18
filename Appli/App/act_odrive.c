#include "act_odrive.h"
#include "critical.h"
#include "main.h"
#include <math.h>
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* ODrive CANSimple: arbitration id is (node_id << 5) | cmd_id. */
#define ODRV_CMD_HEARTBEAT        0x001u
#define ODRV_CMD_SET_AXIS_STATE   0x007u
#define ODRV_CMD_GET_ENCODER      0x009u
#define ODRV_CMD_SET_INPUT_POS    0x00Cu
#define ODRV_CMD_GET_TORQUES      0x01Cu

/* How often a standing disarm request is repeated while faulted, in ticks.
   Once is not enough - the frame can be lost, or a drive can reboot into
   closed loop - and 1 kHz would flood a bus that may already be struggling. */
#define ODRV_DISARM_REPEAT_TICKS  100u

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
#define ODRV_USE_FD               0     /* 1 = FD + BRS, 0 = classic 2.0 */

#if ODRV_USE_FD
#define ODRV_TX_FORMAT            FDCAN_FD_CAN
#define ODRV_TX_BRS               FDCAN_BRS_ON
#else
#define ODRV_TX_FORMAT            FDCAN_CLASSIC_CAN
#define ODRV_TX_BRS               FDCAN_BRS_OFF
#endif

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

/* Ticks since each joint last reported a position. Written from the FDCAN
   ISRs and from the tick, so every access is inside a critical section. */
static volatile uint16_t s_pos_age[NEXUS_NUM_JOINTS];

/* Non-zero while a disarm is standing, with a countdown to the next repeat. */
static uint8_t  s_disarmed;
static uint32_t s_disarm_repeat;

/* Set once both FDCAN peripherals are started. Guards the fault-path disarm,
   which can be reached before act_init() has ever run. */
static volatile uint8_t s_can_ready;

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
#if ODRV_USE_FD
    /* See test_leg_can.c: an isolated transceiver's loop delay is most of a
       data bit at 5 Mbit, so without TDC every FD frame fails its own bit
       check. Classic CAN has no data phase and never needs this. */
    HAL_FDCAN_ConfigTxDelayCompensation(h, 8u, 0u);
    HAL_FDCAN_EnableTxDelayCompensation(h);
#endif

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

    /* Start every joint at maximum age. Nothing has been heard from any drive
       yet, and "unknown" must read as stale, not as fresh zeros. */
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        s_pos_age[j] = 0xFFFFu;
    }

    s_disarmed      = 0;
    s_disarm_repeat = 0;

    bus_setup(&hfdcan1);
    bus_setup(&hfdcan2);

    s_can_ready = 1;
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
            s_pos_age[j] = 0u;      /* this is the one that means "trustworthy" */
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

static void send_axis_state(uint8_t bus, uint32_t node, uint32_t state)
{
    uint8_t data[8] = { 0 };

    data[0] = (uint8_t)(state & 0xFFu);
    data[1] = (uint8_t)((state >> 8) & 0xFFu);
    data[2] = (uint8_t)((state >> 16) & 0xFFu);
    data[3] = (uint8_t)((state >> 24) & 0xFFu);

    tx_enqueue(bus, (node << 5) | ODRV_CMD_SET_AXIS_STATE, data);
}

static void request_all_idle(void)
{
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        uint8_t  bus  = (uint8_t)(j / ODRV_NODES_PER_BUS);
        uint32_t node = (uint32_t)(j % ODRV_NODES_PER_BUS) + 1u;

        send_axis_state(bus, node, ODRV_AXIS_STATE_IDLE);
    }
    act_tx_pump();
}

void act_disarm(void)
{
    /*
     * Stop the interpolator dead rather than letting it finish its segment.
     * Clearing s_have_target is what actually silences act_tick_1khz(); the
     * IDLE request on top of it is what makes the drives stop holding torque
     * even if this board goes on to do something worse.
     */
    s_have_target = 0;
    s_seg_tick    = CMD_SEGMENT_TICKS;

    if (!s_disarmed)
    {
        s_disarmed      = 1;
        s_disarm_repeat = 0;
        request_all_idle();
    }
}

uint8_t act_is_armed(void)
{
    return s_have_target;
}

void act_emergency_idle(void)
{
    /*
     * Error_Handler() can fire long before act_init() - a failed clock or BSP
     * init reaches it while the FDCAN handles are still zeroed, and touching
     * h->Instance then turns a reported error into a hard fault inside the
     * error handler.
     */
    if (!s_can_ready)
    {
        return;
    }

    /*
     * Deliberately does not touch the software queue or any module state:
     * this runs from fault handlers, where the queue's indices may be exactly
     * what is corrupt. Straight into the hardware FIFO, bounded spins only.
     */
    FDCAN_TxHeaderTypeDef hdr;

    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = FDCAN_DLC_BYTES_8;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = ODRV_TX_BRS;
    hdr.FDFormat            = ODRV_TX_FORMAT;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;

    uint8_t data[8] = { (uint8_t)ODRV_AXIS_STATE_IDLE, 0, 0, 0, 0, 0, 0, 0 };

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        uint8_t              bus  = (uint8_t)(j / ODRV_NODES_PER_BUS);
        uint32_t             node = (uint32_t)(j % ODRV_NODES_PER_BUS) + 1u;
        FDCAN_HandleTypeDef *h    = bus_handle(bus);

        hdr.Identifier = (node << 5) | ODRV_CMD_SET_AXIS_STATE;

        /* Wait for room, but never forever - a bus-off peripheral never
           frees a slot, and hanging here would strand the other bus too. */
        for (uint32_t spin = 0; spin < 100000u; spin++)
        {
            if (HAL_FDCAN_GetTxFifoFreeLevel(h) != 0u)
            {
                break;
            }
        }

        (void)HAL_FDCAN_AddMessageToTxFifoQ(h, &hdr, data);
    }
}

void act_set_targets(const float target_pos[NEXUS_NUM_JOINTS])
{
    s_disarmed = 0;

    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        if (!s_have_target)
        {
            /*
             * First target after a disarm: start the ramp from where the leg
             * ACTUALLY is, not from where the Pi wants it.
             *
             * Seeding from the commanded value makes the first frame a step
             * input - the drive is told to be somewhere else immediately and
             * gets there as fast as it can. Seeding from the measured
             * position turns the same command into the first millisecond of a
             * normal interpolated move.
             *
             * Only when the measurement can be trusted. A joint that has not
             * reported recently would seed from a stale or zero value, which
             * is the step input again, only with a worse starting point.
             *
             * Read without a critical section, unlike act_get(): both loads
             * are single aligned words, so neither can tear, and the worst an
             * FDCAN interrupt landing between them can do is pair an age with
             * a position one frame newer. For a starting point that is not a
             * difference worth disabling interrupts for.
             */
            float seed = ((s_pos_age[j] <= ACT_POS_STALE_TICKS) &&
                          isfinite(s_telem.pos[j]))
                             ? s_telem.pos[j]
                             : target_pos[j];

            s_out[j]      = seed;
            s_prev_out[j] = seed;
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
    /* Age every joint's position, armed or not - the estimator has to know
       how old a value is even while nothing is being commanded. */
    uint32_t primask = critical_enter();
    for (int j = 0; j < NEXUS_NUM_JOINTS; j++)
    {
        if (s_pos_age[j] < 0xFFFFu)
        {
            s_pos_age[j]++;
        }
    }
    critical_exit(primask);

    /* Keep a standing disarm standing. A single frame can be lost, and a
       drive that reboots comes back in whatever state it was configured for. */
    if (s_disarmed)
    {
        if (++s_disarm_repeat >= ODRV_DISARM_REPEAT_TICKS)
        {
            s_disarm_repeat = 0;
            request_all_idle();
        }
    }

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
        out->pos_age[j]  = s_pos_age[j];
        s_telem.flags[j] = 0;
    }

    critical_exit(primask);
}
