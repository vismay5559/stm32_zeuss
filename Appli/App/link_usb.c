#include "link_usb.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

#define LINK_RX_DMA_SIZE  2048u

/*
 * The OTG_HS core has its own DMA master, so every buffer it touches must live
 * outside the D-cache. The generated CDC buffers are in normal cached RAM, so
 * both directions are redirected here instead.
 */
static uint8_t s_rx_dma[LINK_RX_DMA_SIZE] __attribute__((section("noncacheable_buffer"), aligned(32)));
static uint8_t s_tx_dma[sizeof(nexus_state_t)] __attribute__((section("noncacheable_buffer"), aligned(32)));

static uint8_t  s_acc[sizeof(nexus_cmd_t)];
static uint16_t s_acc_len;
static uint8_t  s_sync_seen;

static nexus_cmd_t s_cmd;
static volatile uint8_t s_cmd_ready;

uint16_t nexus_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;

    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++)
        {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

void link_usb_init(void)
{
    s_acc_len   = 0;
    s_sync_seen = 0;
    s_cmd_ready = 0;
    memset(&s_cmd, 0, sizeof(s_cmd));

    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, s_rx_dma);
}

uint8_t link_usb_send_state(nexus_state_t *st)
{
    st->sync    = NEXUS_SYNC;
    st->msg_id  = NEXUS_MSG_STATE;
    st->version = NEXUS_PROTO_VERSION;
    st->crc     = nexus_crc16((const uint8_t *)st, sizeof(nexus_state_t) - sizeof(uint16_t));

    memcpy(s_tx_dma, st, sizeof(nexus_state_t));

    return CDC_Transmit_HS(s_tx_dma, (uint16_t)sizeof(nexus_state_t));
}

static void feed(uint8_t b)
{
    if (s_acc_len == 0u)
    {
        if (b != (uint8_t)(NEXUS_SYNC & 0xFFu))
        {
            return;
        }
        s_acc[s_acc_len++] = b;
        return;
    }

    if (s_acc_len == 1u)
    {
        if (b != (uint8_t)((NEXUS_SYNC >> 8) & 0xFFu))
        {
            /* Not the second sync byte; it may itself start a new frame. */
            s_acc_len = (b == (uint8_t)(NEXUS_SYNC & 0xFFu)) ? 1u : 0u;
            return;
        }
        s_acc[s_acc_len++] = b;
        return;
    }

    s_acc[s_acc_len++] = b;

    if (s_acc_len < sizeof(nexus_cmd_t))
    {
        return;
    }

    nexus_cmd_t *c   = (nexus_cmd_t *)s_acc;
    uint16_t     crc = nexus_crc16(s_acc, sizeof(nexus_cmd_t) - sizeof(uint16_t));

    if ((c->crc == crc) && (c->msg_id == NEXUS_MSG_COMMAND) &&
        (c->version == NEXUS_PROTO_VERSION))
    {
        memcpy(&s_cmd, c, sizeof(s_cmd));
        s_cmd_ready = 1;
    }

    s_acc_len = 0;
}

void link_usb_on_rx(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        feed(buf[i]);
    }
}

uint8_t link_usb_take_command(nexus_cmd_t *out)
{
    if (!s_cmd_ready)
    {
        return 0;
    }

    memcpy(out, &s_cmd, sizeof(*out));
    s_cmd_ready = 0;
    return 1;
}
