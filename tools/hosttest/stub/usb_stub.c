/*
 * Host stub for the USB peripheral, paired with usbd_cdc_if.h.
 *
 * Records what the firmware tried to send so a test can inspect it, and lets a
 * test say "the endpoint is busy" to exercise the path where a state packet is
 * dropped rather than spliced into the previous one.
 */
#include "usbd_cdc_if.h"

#include <string.h>

USBD_HandleTypeDef hUsbDeviceHS;

static USBD_CDC_HandleTypeDef s_cdc;
static uint32_t s_tx_calls;
static uint16_t s_last_len;
static uint8_t  s_last[1024];

void stub_usb_reset(void)
{
    memset(&s_cdc, 0, sizeof(s_cdc));
    hUsbDeviceHS.pClassData = &s_cdc;
    hUsbDeviceHS.dev_state  = USBD_STATE_CONFIGURED;
    s_tx_calls = 0;
    s_last_len = 0;
}

void stub_usb_set_busy(uint8_t busy)
{
    s_cdc.TxState = busy ? 1U : 0U;
}

uint32_t       stub_usb_tx_calls(void)   { return s_tx_calls; }
uint16_t       stub_usb_last_tx_len(void){ return s_last_len; }
const uint8_t *stub_usb_last_tx(void)    { return s_last; }

uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff)
{
    (void)pdev; (void)pbuff;
    return USBD_OK;
}

uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
    (void)pdev;
    return USBD_OK;
}

uint8_t CDC_Transmit_HS(uint8_t *Buf, uint16_t Len)
{
    if (s_cdc.TxState != 0U)
    {
        return USBD_BUSY;
    }
    s_tx_calls++;
    s_last_len = (Len < sizeof(s_last)) ? Len : (uint16_t)sizeof(s_last);
    memcpy(s_last, Buf, s_last_len);
    return USBD_OK;
}
