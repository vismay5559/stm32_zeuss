#include "link_usb.h"
#include "dma_buffer.h"
#include "critical.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceHS;

#define LINK_RX_DMA_SIZE  2048u

/*
 * The OTG_HS core has its own DMA master, so every buffer it touches must live
 * outside the D-cache - see dma_buffer.h. The generated CDC buffers are in
 * normal cached RAM, so both directions are redirected here instead.
 */
static uint8_t s_rx_dma[LINK_RX_DMA_SIZE] NEXUS_DMA_BUFFER;
static uint8_t s_tx_dma[sizeof(nexus_state_t)] NEXUS_DMA_BUFFER;

static uint8_t  s_acc[NEXUS_RX_MAX];
static uint16_t s_acc_len;
static uint8_t  s_sync_seen;

static nexus_cmd_t s_cmd;
static nexus_gains_t s_gains;
static volatile uint8_t s_gains_ready;
static uint32_t s_gains_count;
static uint16_t s_want;          /* bytes expected for the frame being read */
static volatile uint8_t s_cmd_ready;

/* Commands accepted from the Pi. Used to detect a dead link. */
static volatile uint32_t s_cmd_count;

/* State packets skipped because the previous one had not finished going out.
   Non-zero means the host is not draining the endpoint at 1 kHz. */
static uint32_t s_tx_dropped;

uint16_t nexus_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;

    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++)
        {
            /* Shift as unsigned: a uint16_t promotes to int, so the shift
               and XOR happen in a signed type before being narrowed back.
               Same result, but it no longer relies on that promotion. */
            crc = (crc & 0x8000u) ? (uint16_t)(((uint32_t)crc << 1) ^ 0x1021u)
                                  : (uint16_t)((uint32_t)crc << 1);
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

/*
 * WHAT THE USB HARDWARE IS ACTUALLY DOING, AND A LIVE RETRY OF ITS SUPPLY.
 *
 * The transceiver needs VDD33USB, which this MCU makes internally from
 * VDD50USB (fed from the cable's VBUS) via a regulator enabled by USBREGEN.
 * PWR_CSR2.USB33RDY is the hardware saying that rail is up. While it is 0 the
 * PHY cannot pull D+ up, so no host anywhere sees a device - which from the
 * host side is indistinguishable from a dead cable.
 *
 * The enable sequence runs once, at boot, inside MX_USB_DEVICE_Init(). If VBUS
 * was absent or late at that moment the regulator never came up and nothing
 * ever tries again. So this does two things:
 *
 *   - prints the four control bits, not just the ready flag, so a failure says
 *     WHICH step did not take; and
 *   - re-runs the enable whenever USB33RDY is 0, which both tests the
 *     boot-order theory and doubles as hot-plug recovery on the robot.
 *
 * If the bits read back set and USB33RDY still stays 0, the regulator is
 * enabled and getting no VDD50USB - a board-level supply question, not
 * firmware.
 */
void link_usb_diag(void)
{
    /*
     * Stay quiet once the link is healthy.
     *
     * The console is a polled UART: __io_putchar blocks for one character time
     * per byte, 87 us at 115200. Printing this line every second costs the
     * 1 kHz loop over a hundred ticks a minute for news that has not changed.
     * So report every time while something is wrong - that is when someone is
     * watching - and only occasionally once the link is up.
     */
    static uint32_t calls;
    const uint8_t healthy = (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED) &&
                            ((PWR->CSR2 & PWR_CSR2_USB33RDY) != 0u);
    if (healthy && ((calls++ % 30u) != 0u))
    {
        return;
    }

    const USB_OTG_DeviceTypeDef *dev =
        (USB_OTG_DeviceTypeDef *)((uint32_t)USB_OTG_HS + USB_OTG_DEVICE_BASE);

    uint32_t csr2 = PWR->CSR2;
    uint8_t  rdy  = (csr2 & PWR_CSR2_USB33RDY) ? 1u : 0u;

    if (!rdy)
    {
        /*
         * There are TWO supply arrangements, and which one a board uses is a
         * wiring decision the firmware cannot see:
         *
         *   internal  VDD50USB is fed from the cable's VBUS and the on-chip
         *             regulator (USBREGEN) makes VDD33USB from it.
         *   external  VDD33USB is fed directly from the board's own 3V3, with
         *             VDD33USB and VDD50USB tied together. Then the regulator
         *             must be OFF: left on it tries to regulate 3.3 V down to
         *             3.3 V, never reaches threshold, and USB33RDY stays 0
         *             forever with every enable bit reading back set.
         *
         * Enabling the regulator on an external-supply board and enabling
         * nothing on an internal-supply board fail identically. So try both and
         * report which one brought the rail up - that answers the wiring
         * question from the bench instead of from a schematic.
         */
        for (int mode = 0; mode < 2 && !rdy; mode++)
        {
            if (mode == 0)
            {
                HAL_PWREx_EnableUSBReg();           /* internal regulator  */
            }
            else
            {
                HAL_PWREx_DisableUSBReg();          /* external 3V3 supply */
            }
            HAL_PWREx_EnableUSBHSregulator();
            HAL_Delay(2);
            SET_BIT(PWR->CSR2, PWR_CSR2_USB33DEN);

            uint32_t t0 = HAL_GetTick();
            while (((PWR->CSR2 & PWR_CSR2_USB33RDY) == 0u) && ((HAL_GetTick() - t0) < 50u))
            {
            }
            csr2 = PWR->CSR2;
            rdy  = (csr2 & PWR_CSR2_USB33RDY) ? 1u : 0u;
            if (rdy)
            {
                printf("USB: rail came up with the regulator %s\r\n",
                       (mode == 0) ? "ENABLED (VBUS -> internal regulator)"
                                   : "DISABLED (external 3V3 on VDD33USB)");
            }
        }
    }

    const uint32_t dsts = dev->DSTS;
    const uint8_t  st   = hUsbDeviceHS.dev_state;

    printf("USB: USB33RDY=%u  USBREGEN=%u USBHSREGEN=%u USB33DEN=%u  "
           "state=%s(%u) enumspd=%u susp=%u\r\n",
           (unsigned)rdy,
           (unsigned)((csr2 & PWR_CSR2_USBREGEN)   ? 1u : 0u),
           (unsigned)((csr2 & PWR_CSR2_USBHSREGEN) ? 1u : 0u),
           (unsigned)((csr2 & PWR_CSR2_USB33DEN)   ? 1u : 0u),
           (st == USBD_STATE_CONFIGURED) ? "CONFIGURED"
         : (st == USBD_STATE_ADDRESSED)  ? "ADDRESSED"
         : (st == USBD_STATE_SUSPENDED)  ? "SUSPENDED"
         : (st == USBD_STATE_DEFAULT)    ? "DEFAULT (not enumerated)"
                                         : "unknown",
           (unsigned)st,
           (unsigned)((dsts & USB_OTG_DSTS_ENUMSPD) >> USB_OTG_DSTS_ENUMSPD_Pos),
           (unsigned)((dsts & USB_OTG_DSTS_SUSPSTS) ? 1u : 0u));

    if (!rdy)
    {
        printf("     -> VDD33USB stays down in BOTH supply modes. The rail\r\n"
               "        itself is absent at the MCU pin - a board supply path,\r\n"
               "        not a firmware setting.\r\n");
    }
    else if (st == USBD_STATE_DEFAULT)
    {
        printf("     -> rail is UP and D+ is pulled up, but no bus reset from\r\n"
               "        the host: the data pair is not getting through.\r\n");
    }
}

/*
 * True while the USB core still owns s_tx_dma, or while there is no host to
 * send to at all.
 *
 * TxState only goes 0 -> 1 inside CDC_Transmit_HS, which nothing but
 * link_usb_send_state calls, and 1 -> 0 in the transfer-complete ISR. So a
 * zero read here cannot become non-zero behind our back, and the buffer is
 * genuinely safe to overwrite.
 */
static uint8_t tx_busy(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;

    if ((hcdc == NULL) || (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED))
    {
        return 1;   /* nothing enumerated - the usual case with only ST-LINK */
    }

    return (hcdc->TxState != 0u) ? 1u : 0u;
}

uint8_t link_usb_send_state(nexus_state_t *st)
{
    /*
     * Ask BEFORE copying. CDC_Transmit_HS also returns BUSY, but by the time
     * it does the memcpy below has already overwritten the buffer the USB DMA
     * is still reading out - the Pi then receives a packet that is half this
     * tick and half the last one. Its CRC fails so nothing wrong reaches the
     * policy, but the packet is lost and, without this counter, invisibly.
     */
    if (tx_busy())
    {
        s_tx_dropped++;
        return USBD_BUSY;
    }

    st->sync    = NEXUS_SYNC;
    st->msg_id  = NEXUS_MSG_STATE;
    st->version = NEXUS_PROTO_VERSION;
    st->crc     = nexus_crc16((const uint8_t *)st, sizeof(nexus_state_t) - sizeof(uint16_t));

    memcpy(s_tx_dma, st, sizeof(nexus_state_t));

    return CDC_Transmit_HS(s_tx_dma, (uint16_t)sizeof(nexus_state_t));
}

uint32_t link_usb_tx_dropped(void)
{
    return s_tx_dropped;
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

    /*
     * The third byte says what this is, and different messages are different
     * lengths. Deciding here - rather than assuming every frame is a command -
     * is what lets the rare gains message share the same reader without the
     * 250 Hz command path paying for it.
     */
    if (s_acc_len == 2u)
    {
        if (b == (uint8_t)NEXUS_MSG_COMMAND)
        {
            s_want = (uint16_t)sizeof(nexus_cmd_t);
        }
        else if (b == (uint8_t)NEXUS_MSG_GAINS)
        {
            s_want = (uint16_t)sizeof(nexus_gains_t);
        }
        else
        {
            /* Not a message this board knows. Start looking for sync again -
               and this byte may itself be the start of the next frame, so it
               is tested rather than thrown away. */
            s_acc_len = (b == (uint8_t)(NEXUS_SYNC & 0xFFu)) ? 1u : 0u;
            s_acc[0]  = b;
            return;
        }
    }

    s_acc[s_acc_len++] = b;

    if (s_acc_len < s_want)
    {
        return;
    }

    uint16_t crc = nexus_crc16(s_acc, (uint32_t)(s_want - sizeof(uint16_t)));

    if (s_acc[2] == (uint8_t)NEXUS_MSG_COMMAND)
    {
        nexus_cmd_t *c = (nexus_cmd_t *)s_acc;

        if ((c->crc == crc) && (c->version == NEXUS_PROTO_VERSION))
        {
            memcpy(&s_cmd, c, sizeof(s_cmd));
            s_cmd_ready = 1;
            s_cmd_count++;
        }
    }
    else
    {
        nexus_gains_t *g = (nexus_gains_t *)s_acc;

        if ((g->crc == crc) && (g->version == NEXUS_PROTO_VERSION))
        {
            memcpy(&s_gains, g, sizeof(s_gains));
            s_gains_ready = 1;
            s_gains_count++;
        }
    }

    s_acc_len = 0;
    s_want    = 0;
}

void link_usb_on_rx(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        feed(buf[i]);
    }
}

uint32_t link_usb_cmd_count(void)
{
    return s_cmd_count;
}

uint32_t link_usb_gains_count(void)
{
    return s_gains_count;
}

uint8_t link_usb_take_gains(nexus_gains_t *out)
{
    if (!s_gains_ready)
    {
        return 0;
    }

    /* Same reason as the command below: feed() writes s_gains from the USB
       ISR, and half of one gains message spliced onto half of another would
       be applied to the drives without anything noticing. */
    uint32_t primask = critical_enter();

    memcpy(out, &s_gains, sizeof(*out));
    s_gains_ready = 0;

    critical_exit(primask);
    return 1;
}

uint8_t link_usb_take_command(nexus_cmd_t *out)
{
    if (!s_cmd_ready)
    {
        return 0;
    }

    /* s_cmd is written by feed() from the USB OTG_HS ISR. Without this guard a
       command arriving mid-copy blends two different frames, which would hand
       a target position that was never sent to all ten actuators. */
    uint32_t primask = critical_enter();

    memcpy(out, &s_cmd, sizeof(*out));
    s_cmd_ready = 0;

    critical_exit(primask);
    return 1;
}
