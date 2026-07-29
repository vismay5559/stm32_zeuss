#ifndef ENC_AS5048A_H
#define ENC_AS5048A_H

#include <stdint.h>
#include "link_proto.h"

void enc_init(void);
void enc_start_read(void);
void enc_on_dma_complete(void);
void enc_get(uint16_t angle[NEXUS_NUM_ENCODERS], uint8_t *valid_mask);

#endif /* ENC_AS5048A_H */
