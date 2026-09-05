#include "fdcan.h"

#include <string.h>

FDCAN_HandleTypeDef hfdcan1 = { 1 };
FDCAN_HandleTypeDef hfdcan2 = { 2 };

#define SENT_CAP  512u
#define RX_CAP     64u

typedef struct
{
    /* Frames that have left the board. */
    host_can_frame_t sent[SENT_CAP];
    uint32_t         sent_n;

    /* Occupancy of the three-deep hardware queue. */
    uint32_t         tx_used;
    int              wire_blocked;
    uint32_t         refuse_tx;

    /* Frames waiting to be read by act_on_rx(). */
    host_can_frame_t rx[RX_CAP];
    uint32_t         rx_head;
    uint32_t         rx_tail;
    int              rx_broken;

    uint32_t         tec;
    int              bus_off;
    int              status_broken;
    int              counter_broken;

    uint32_t         starts;
} host_bus_t;

static host_bus_t s_bus[2];

static uint8_t bus_of(const FDCAN_HandleTypeDef *h)
{
    /* The casts are not decoration: the two branches of a ?: are unsigned int,
       and real GCC's -Wconversion rejects narrowing that to uint8_t where
       clang says nothing. CI builds with both for exactly this reason. */
    return (h == &hfdcan1) ? (uint8_t)0u : (uint8_t)1u;
}

void host_can_reset(void)
{
    memset(s_bus, 0, sizeof(s_bus));
}

uint32_t host_can_sent_count(uint8_t bus)
{
    return (bus < 2u) ? s_bus[bus].sent_n : 0u;
}

const host_can_frame_t *host_can_sent(uint8_t bus, uint32_t index)
{
    if ((bus >= 2u) || (index >= s_bus[bus].sent_n))
    {
        return 0;
    }
    return &s_bus[bus].sent[index];
}

void host_can_forget_sent(void)
{
    s_bus[0].sent_n = 0;
    s_bus[1].sent_n = 0;
}

void host_can_wire_blocked(uint8_t bus, int blocked)
{
    s_bus[bus & 1u].wire_blocked = blocked;

    if (!blocked)
    {
        /* The backlog goes out. */
        s_bus[bus & 1u].tx_used = 0;
    }
}

void host_can_refuse_tx(uint8_t bus, uint32_t n)
{
    s_bus[bus & 1u].refuse_tx = n;
}

void host_can_deliver(uint8_t bus, uint32_t identifier, const uint8_t *data)
{
    host_bus_t *b    = &s_bus[bus & 1u];
    uint32_t    next = (b->rx_head + 1u) % RX_CAP;

    if (next == b->rx_tail)
    {
        return;   /* test asked for more than the model holds */
    }

    b->rx[b->rx_head].identifier = identifier;
    memcpy(b->rx[b->rx_head].data, data, 8);
    b->rx_head = next;
}

void host_can_break_rx_read(uint8_t bus, int broken)
{
    s_bus[bus & 1u].rx_broken = broken;
}

void host_can_set_bus_off(uint8_t bus, int off)
{
    s_bus[bus & 1u].bus_off = off;
}

void host_can_set_tx_errors(uint8_t bus, uint32_t tec)
{
    s_bus[bus & 1u].tec = tec;
}

void host_can_break_status_read(uint8_t bus, int broken)
{
    s_bus[bus & 1u].status_broken = broken;
}

void host_can_break_counter_read(uint8_t bus, int broken)
{
    s_bus[bus & 1u].counter_broken = broken;
}

uint32_t host_can_start_count(uint8_t bus)
{
    return (bus < 2u) ? s_bus[bus].starts : 0u;
}

/* --- the HAL surface -------------------------------------------------- */

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h,
                                         const FDCAN_FilterTypeDef *f)
{
    (void)h; (void)f;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h,
                                               uint32_t std, uint32_t ext,
                                               uint32_t std_rmt, uint32_t ext_rmt)
{
    (void)h; (void)std; (void)ext; (void)std_rmt; (void)ext_rmt;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *h,
                                                      uint32_t offset,
                                                      uint32_t filter)
{
    (void)h; (void)offset; (void)filter;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *h)
{
    (void)h;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h)
{
    s_bus[bus_of(h)].starts++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,
                                                 uint32_t its, uint32_t lines)
{
    (void)h; (void)its; (void)lines;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,
                                                const FDCAN_TxHeaderTypeDef *hdr,
                                                const uint8_t *data)
{
    host_bus_t *b = &s_bus[bus_of(h)];

    if (b->refuse_tx > 0u)
    {
        b->refuse_tx--;
        return HAL_ERROR;
    }

    if (b->tx_used >= HOST_CAN_TX_FIFO_DEPTH)
    {
        return HAL_ERROR;
    }

    if (b->wire_blocked)
    {
        /* Accepted by the peripheral, but it never reaches anyone. */
        b->tx_used++;
        return HAL_OK;
    }

    if (b->sent_n < SENT_CAP)
    {
        b->sent[b->sent_n].identifier = hdr->Identifier;
        memcpy(b->sent[b->sent_n].data, data, 8);
        b->sent_n++;
    }

    return HAL_OK;
}

uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h)
{
    const host_bus_t *b = &s_bus[bus_of(h)];

    return HOST_CAN_TX_FIFO_DEPTH - b->tx_used;
}

uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *h, uint32_t fifo)
{
    const host_bus_t *b = &s_bus[bus_of(h)];

    (void)fifo;
    return (b->rx_head + RX_CAP - b->rx_tail) % RX_CAP;
}

HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
                                         FDCAN_RxHeaderTypeDef *hdr,
                                         uint8_t *data)
{
    host_bus_t *b = &s_bus[bus_of(h)];

    (void)fifo;

    if (b->rx_broken)
    {
        return HAL_ERROR;
    }

    if (b->rx_head == b->rx_tail)
    {
        return HAL_ERROR;
    }

    memset(hdr, 0, sizeof(*hdr));
    hdr->Identifier = b->rx[b->rx_tail].identifier;
    hdr->DataLength = 8u;
    memcpy(data, b->rx[b->rx_tail].data, 8);

    b->rx_tail = (b->rx_tail + 1u) % RX_CAP;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *h,
                                              FDCAN_ProtocolStatusTypeDef *ps)
{
    const host_bus_t *b = &s_bus[bus_of(h)];

    if (b->status_broken)
    {
        return HAL_ERROR;
    }

    memset(ps, 0, sizeof(*ps));
    ps->BusOff = b->bus_off ? 1u : 0u;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *h,
                                             FDCAN_ErrorCountersTypeDef *ec)
{
    const host_bus_t *b = &s_bus[bus_of(h)];

    if (b->counter_broken)
    {
        return HAL_ERROR;
    }

    memset(ec, 0, sizeof(*ec));
    ec->TxErrorCnt = b->tec;
    return HAL_OK;
}
