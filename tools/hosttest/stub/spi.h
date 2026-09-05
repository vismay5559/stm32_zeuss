#ifndef HOSTTEST_SPI_H
#define HOSTTEST_SPI_H

/*
 * Host stand-in for the SPI half of the HAL.
 *
 * enc_as5048a.c reads the four spring encoders over SPI, and the peripheral
 * fills the buffer by itself while the processor gets on with something else.
 * That "by itself" is the whole reason this file has to model anything: the
 * driver starts a read, and the answer arrives later, from an interrupt. A
 * test needs to be able to decide when later is - or to decide that it never
 * comes, which is the case the driver had to be fixed for.
 *
 * So the read does not complete on its own here. host_spi_reply() supplies
 * the four words and calls the completion handler; leaving it uncalled is
 * exactly what a transfer that never finishes looks like.
 *
 * The names must match Drivers/STM32H7RSxx_HAL_Driver/Inc/stm32h7rsxx_hal_spi.h.
 * The firmware build in CI is what catches a drift, because the same source
 * has to compile against the real header there.
 */

#include <stdint.h>

typedef struct { int id; } SPI_TypeDef;

extern SPI_TypeDef host_spi1;
extern SPI_TypeDef host_spi2;

#define SPI1  (&host_spi1)
#define SPI2  (&host_spi2)

typedef struct
{
    SPI_TypeDef *Instance;
} SPI_HandleTypeDef;

extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *h,
                                              const uint8_t *tx, uint8_t *rx,
                                              uint16_t size);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *h);

/* Provided by the driver under test; the HAL declares it __weak. */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *h);

/* Also provided by the driver: the stub calls it to finish a transfer. */
void enc_on_dma_complete(void);

/* --- what a test can see and control --------------------------------- */

void     host_spi_reset(void);

/* Refuse to start the next `n` transfers. */
void     host_spi_refuse(uint32_t n);

/* Is a transfer in flight, as far as the peripheral is concerned? */
int      host_spi_busy(void);

/* How many transfers have been started, and how many abandoned. */
uint32_t host_spi_started(void);
uint32_t host_spi_aborted(void);

/* What the driver asked to send, from the last started transfer. */
uint16_t host_spi_sent_word(uint32_t index);

/*
 * Complete the transfer in flight with these words, exactly as the interrupt
 * would. Does nothing if no transfer is in flight - which is itself worth
 * asserting on.
 */
void     host_spi_reply(const uint16_t *words, uint32_t count);

#endif /* HOSTTEST_SPI_H */
