#ifndef IMU_BNO085_H
#define IMU_BNO085_H

#include <stdint.h>

typedef struct
{
    float    quat[4];    /* w, x, y, z */
    float    accel[3];   /* m/s^2, specific force - INCLUDES gravity.
                            The estimator needs what the accelerometer
                            physically reads, not linear acceleration. */
    float    gyro[3];    /* rad/s */
    uint32_t seq;
} imu_sample_t;

void imu_init(void);

/*
 * Re-send the SET_FEATURE commands that ask for the three reports.
 *
 * imu_init() sends them once, 100 ms after boot. That is only enough if the
 * sensor was already running by then - a sensor powered up later, reset by
 * hand, or slower to boot than expected never hears them and stays silent
 * forever, which looks exactly like a wiring fault. Calling this periodically
 * while no samples are arriving removes boot order from the list of suspects.
 */
void imu_request_reports(void);

/*
 * Ask for the product ID - the simplest command SH-2 has, two bytes with no
 * parameters. Used to tell a malformed Set Feature apart from a transport that
 * corrupts everything: if even this is rejected, the contents were never the
 * problem.
 */
void imu_request_product_id(void);

/*
 * Software reset, one byte on the SHTP executable channel.
 *
 * The BNO085 can power up wedged - rejecting every command with an SHTP error
 * and never sending its boot advertisement. Only a reset recovers it. This
 * does in firmware what pulling the RST pin low does in hardware, so no extra
 * wire is needed. imu_init() calls it at startup.
 */
void imu_soft_reset(void);

/*
 * Flush the sensor's frame parser with a burst of bare SHTP delimiters.
 *
 * PA9 floats from power-on until the UART is initialised, and the sensor boots
 * in that same instant and reads the floating line as data - leaving its parser
 * stuck mid-frame, swallowing every later command including the reset that
 * would fix it. A run of delimiters closes the phantom frame. Harmless if the
 * parser was already idle.
 */
void imu_resync(void);
void imu_service(void);
void imu_get(imu_sample_t *out);
void imu_on_rx_event(uint16_t size);
uint32_t imu_rx_events(void);

/* Diagnostics: raw bytes off the UART, and valid SHTP frames parsed out of
   them. Together they separate a wiring fault from a protocol fault. */
uint32_t imu_rx_bytes(void);
uint32_t imu_frames(void);

/*
 * Raw peripheral state, for when zero bytes arrive and the question is whether
 * the STM32 is even listening. Everything here is read straight out of the
 * hardware, so it cannot agree with a bug in the driver's own bookkeeping.
 */
typedef struct
{
    uint32_t arm_status;    /* HAL status returned when RX DMA was armed; 0 = OK */
    uint32_t uart_isr;      /* USART1->ISR: bit0 PE, 1 FE, 2 NE, 3 ORE, 5 RXNE  */
    uint32_t uart_error;    /* huart1.ErrorCode                                 */
    uint32_t rx_state;      /* huart1.RxState; 0x22 = busy receiving            */
    uint32_t dma_ndtr;      /* bytes left in the circular buffer; must move     */
    uint32_t rx_events;     /* idle/half/full callbacks seen                    */
    uint32_t errors;        /* UART errors recovered from                       */
} imu_diag_t;

void imu_diag(imu_diag_t *out);

/* UART errors seen and recovered from. A steady rate here means the bytes are
   arriving corrupted - wrong baud, or a link that cannot carry 3 Mbaud. */
uint32_t imu_errors(void);

/*
 * The first bytes ever received, verbatim. Counters describe traffic; only the
 * bytes say what it actually is:
 *   7E 01 ...   SHTP framing, baud is right
 *   AA AA ...   UART-RVC, the sensor is in the wrong mode
 *   noise       baud mismatch - nothing downstream can ever parse it
 */
uint16_t imu_snapshot(uint8_t *out, uint16_t max);

/*
 * Frames received on one SHTP channel. Channels are 0 command, 1 executable,
 * 2 control, 3 input reports. Channel 2 traffic proves SET_FEATURE was heard;
 * channel 3 is the sensor data itself.
 */
uint16_t imu_channel_frames(uint8_t ch);

/* Report ID of the last control-channel reply. 0xF8 = Product ID response,
   0xFC = Get Feature response. Either proves a command was understood, not
   merely received. */
uint8_t imu_last_control_response(void);

/* The sensor sends an advertisement on channel 0 once after every reset. It is
   the only positive proof it has finished booting - commands sent before it are
   discarded with no error, which looks exactly like a dead sensor. */
/* Reports actually decoded, per type. The frame count sums three separate
   streams and so overstates every one of them; these are the real rates. */
void imu_report_counts(uint32_t *rv, uint32_t *accel, uint32_t *gyro);

uint8_t imu_saw_advertisement(void);

/* A reset-complete notice arrived on the executable channel. */
uint8_t imu_saw_reset(void);

/* The full payload of the last control-channel reply. A Get Feature Response
   (0xFC) carries the report ID and the interval actually in force - an interval
   of zero means the report is disabled, which Set Feature never tells us. */
uint16_t imu_control_payload(uint8_t *out, uint16_t max);

/* Ask what the sensor did with a Set Feature. Reply is a Get Feature Response
   on channel 2. */
void imu_request_feature_status(uint8_t report_id);

/* The first frame we transmitted, verbatim. The receive path can be checked
   against bytes on the wire; this is the only way to check the other half. */
uint16_t imu_tx_snapshot(uint8_t *out, uint16_t max);

/* Transmit a byte pattern on the IMU UART. Only used by the loopback test:
   with PA9 jumpered to PA10 the bytes come straight back, which proves the
   STM32 side works without the sensor being involved at all. */
void imu_tx_test_pattern(void);

/* Compare the received snapshot against the transmitted pattern. Returns the
   number of corrupted bytes and, via out_total, how many were checked. A byte
   count alone cannot see corruption - the right number of wrong bytes looks
   identical to success. */
uint32_t imu_loopback_check(uint32_t *out_total);

#endif /* IMU_BNO085_H */
