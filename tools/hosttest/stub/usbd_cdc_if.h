/*
 * Host stub for the USB CDC interface.
 *
 * link_usb.c is mostly pure logic - framing, CRC checking, deciding whether a
 * command is complete - wrapped in a thin layer that hands bytes to the USB
 * peripheral. This stands in for that peripheral so the logic can be tested on
 * a workstation.
 *
 * It is deliberately minimal: only what link_usb.c actually touches. Anything
 * that would need real USB behaviour to be meaningful belongs on a board, not
 * here.
 */
#ifndef HOSTTEST_USBD_CDC_IF_H
#define HOSTTEST_USBD_CDC_IF_H

#include <stdint.h>

#define USBD_OK    0U
#define USBD_BUSY  1U
#define USBD_FAIL  2U

#define USBD_STATE_CONFIGURED  3U

typedef struct
{
    uint32_t TxState;
    uint32_t RxState;
} USBD_CDC_HandleTypeDef;

typedef struct
{
    void    *pClassData;
    uint32_t dev_state;
} USBD_HandleTypeDef;

/* The test drives these: see stub_usb_set_ready() in the test file. */
uint8_t  USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pbuff);
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev);
uint8_t  CDC_Transmit_HS(uint8_t *Buf, uint16_t Len);

/* Test hooks, not part of the firmware API. */
void     stub_usb_reset(void);
void     stub_usb_set_busy(uint8_t busy);
uint32_t stub_usb_tx_calls(void);
uint16_t stub_usb_last_tx_len(void);
const uint8_t *stub_usb_last_tx(void);

#endif /* HOSTTEST_USBD_CDC_IF_H */
