#ifndef LINK_USB_H
#define LINK_USB_H

#include <stdint.h>
#include "link_proto.h"

void    link_usb_init(void);
uint8_t link_usb_send_state(nexus_state_t *st);
void    link_usb_on_rx(uint8_t *buf, uint32_t len);
uint8_t link_usb_take_command(nexus_cmd_t *out);

/* Total valid commands received from the Pi. Stops changing if the link dies. */
uint32_t link_usb_cmd_count(void);

/* State packets never sent because the USB endpoint was still busy. Matches
   the sequence gaps the Pi sees in nexus_link's stats. */
uint32_t link_usb_tx_dropped(void);

#endif /* LINK_USB_H */
