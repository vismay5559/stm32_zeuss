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

    /*
     * Advances when ANY of the three reports updated. Good for "is the sensor
     * alive at all" - which is all health.c wants - and wrong for anything
     * that needs to know whether a particular signal is new.
     */
    uint32_t seq;

    /*
     * Per-report counters.
     *
     * The three reports arrive at different rates: gyro and accelerometer at
     * 400 Hz, the rotation vector at 100 Hz. A shared counter cannot tell
     * "the accelerometer produced a new reading" from "the quaternion did",
     * and the estimator's prediction step needs the former - propagating on a
     * quaternion-only frame integrates the same gyro and accelerometer sample
     * a second time over a fresh dt.
     */
    uint32_t accel_seq;
    uint32_t gyro_seq;
    uint32_t quat_seq;
} imu_sample_t;

/* Start the movement sensor up. Call once at startup. */
void imu_init(void);

/*
 * Try to bring a silent sensor back, one step per call.
 *
 * HAL_UART_ErrorCallback already recovers the LINK - a noise glitch no longer
 * kills reception permanently. What had no recovery at all was the SENSOR:
 * the BNO085 can wedge (this driver documents that at length as a known
 * power-up behaviour), health.c raises HEALTH_IMU after 50 ms, and the
 * resync/reset/request sequence that fixes it only ever ran once, at boot.
 *
 * It cannot simply be called again from the control loop. That sequence takes
 * the better part of a second - byte-spaced transmission plus the sensor's own
 * boot time - and blocking for that long inside a 1 kHz loop would miss
 * hundreds of ticks and trip the watchdog. So it is a state machine: call this
 * once per tick while the IMU is stale, and it advances one step, with the
 * waits counted in ticks rather than spent in HAL_Delay.
 *
 * The longest single blocking step is the 64-byte resync burst, about 8 ms -
 * comfortably inside the watchdog period, and only ever while the robot is
 * already not being driven.
 *
 * Call ONLY when the actuators are not armed. It transmits to the sensor and
 * throws away the parser state, which is not something to do mid-stride.
 *
 * Returns 1 while a recovery is in progress, 0 when idle or finished.
 */
/*
 * Nudge a jammed movement sensor one step closer to working again. Call
 * repeatedly while the robot is NOT moving; returns 1 once the sensor is back.
 *
 * The full wake-up takes the best part of a second, which is far too long to
 * do in one go - the robot would miss hundreds of heartbeats and reset
 * itself. So it is broken into small steps spread across many heartbeats.
 * Only done while the motors are off, because it is not work to be doing
 * while the robot is standing.
 */
uint8_t imu_recover_step(void);

/* Recovery sequences started since boot. Non-zero means the sensor has been
   wedging in flight, which is worth knowing even after it comes back. */
/*
 * How many times the sensor has had to be revived. Should be zero. Climbing
 * means the sensor keeps jamming, which is a hardware or wiring problem.
 */
uint32_t imu_recoveries(void);

/*
 * Re-send the SET_FEATURE commands that ask for the three reports.
 *
 * imu_init() sends them once, 100 ms after boot. That is only enough if the
 * sensor was already running by then - a sensor powered up later, reset by
 * hand, or slower to boot than expected never hears them and stays silent
 * forever, which looks exactly like a wiring fault. Calling this periodically
 * while no samples are arriving removes boot order from the list of suspects.
 */
/* Ask the sensor to start sending readings, and how often. */
void imu_request_reports(void);

/*
 * Ask for the product ID - the simplest command SH-2 has, two bytes with no
 * parameters. Used to tell a malformed Set Feature apart from a transport that
 * corrupts everything: if even this is rejected, the contents were never the
 * problem.
 */
/* Ask the sensor to identify itself - a simple "are you there?" check. */
void imu_request_product_id(void);

/*
 * Software reset, one byte on the SHTP executable channel.
 *
 * The BNO085 can power up wedged - rejecting every command with an SHTP error
 * and never sending its boot advertisement. Only a reset recovers it. This
 * does in firmware what pulling the RST pin low does in hardware, so no extra
 * wire is needed. imu_init() calls it at startup.
 */
/* Ask the sensor to restart itself, without cutting its power. */
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
/*
 * Recover from being out of step with the sensor's messages.
 *
 * The sensor sends a continuous stream, and if the board loses its place in
 * it, everything after that reads as nonsense. This finds the start of a
 * message again.
 */
void imu_resync(void);
/*
 * Deal with anything the sensor has sent. Call often - much more than once
 * per heartbeat - so readings are picked up promptly rather than piling up.
 */
void imu_service(void);
/*
 * Copy out the newest reading: turning rate, acceleration, and the sensor's
 * own opinion of which way up it is.
 *
 * Each part carries a number that counts up every time it is refreshed. If
 * that number has not moved since last time, this is the same reading you
 * already had - not a new one that happens to look similar. The estimator
 * relies on that to avoid counting one measurement twice.
 */
void imu_get(imu_sample_t *out);
/*
 * The hardware calls this by itself when bytes arrive from the sensor.
 * Nothing else should call it.
 */
void imu_on_rx_event(uint16_t size);
/* How many times data has arrived from the sensor. Stops climbing if the
   sensor goes silent. */
uint32_t imu_rx_events(void);

/* Diagnostics: raw bytes off the UART, and valid SHTP frames parsed out of
   them. Together they separate a wiring fault from a protocol fault. */
/* How much data has arrived from the sensor in total. */
uint32_t imu_rx_bytes(void);
/* How many complete, well-formed messages have been decoded. Data arriving
   while this stays flat means the stream is arriving but unreadable. */
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

/* Copy out a bundle of internal counters, for diagnosing a sensor that is
   behaving oddly. */
void imu_diag(imu_diag_t *out);

/* UART errors seen and recovered from. A steady rate here means the bytes are
   arriving corrupted - wrong baud, or a link that cannot carry 3 Mbaud. */
/* How many communication errors have happened. */
uint32_t imu_errors(void);

/*
 * The first bytes ever received, verbatim. Counters describe traffic; only the
 * bytes say what it actually is:
 *   7E 01 ...   SHTP framing, baud is right
 *   AA AA ...   UART-RVC, the sensor is in the wrong mode
 *   noise       baud mismatch - nothing downstream can ever parse it
 */
/*
 * Copy out the raw bytes most recently received, exactly as they arrived.
 * For when a message will not decode and you need to see what actually came
 * in rather than what was expected. Returns how many bytes were copied.
 */
uint16_t imu_snapshot(uint8_t *out, uint16_t max);

/*
 * Frames received on one SHTP channel. Channels are 0 command, 1 executable,
 * 2 control, 3 input reports. Channel 2 traffic proves SET_FEATURE was heard;
 * channel 3 is the sensor data itself.
 */
/*
 * How many messages have arrived on one of the sensor's separate streams.
 * The sensor keeps readings, replies and status notices apart, so this shows
 * which kinds are flowing and which are not.
 */
uint16_t imu_channel_frames(uint8_t ch);

/* Report ID of the last control-channel reply. 0xF8 = Product ID response,
   0xFC = Get Feature response. Either proves a command was understood, not
   merely received. */
/* What the sensor last answered to a request - did it accept the setting or
   refuse it. */
uint8_t imu_last_control_response(void);

/* The sensor sends an advertisement on channel 0 once after every reset. It is
   the only positive proof it has finished booting - commands sent before it are
   discarded with no error, which looks exactly like a dead sensor. */
/* Reports actually decoded, per type. The frame count sums three separate
   streams and so overstates every one of them; these are the real rates. */
/*
 * How many readings of each kind have arrived: orientation, acceleration and
 * turning rate, counted separately.
 *
 * These should climb together. One of them stuck while the others move means
 * the sensor has stopped producing that particular measurement, which is a
 * different fault from the sensor being silent altogether.
 */
void imu_report_counts(uint32_t *rv, uint32_t *accel, uint32_t *gyro);

/* Whether the sensor has introduced itself, which it does once after
   starting up. Its absence means the sensor never came up at all. */
uint8_t imu_saw_advertisement(void);

/* A reset-complete notice arrived on the executable channel. */
/* Whether the sensor has announced that it restarted. Seeing this
   unexpectedly mid-run means the sensor rebooted on its own. */
uint8_t imu_saw_reset(void);

/* The full payload of the last control-channel reply. A Get Feature Response
   (0xFC) carries the report ID and the interval actually in force - an interval
   of zero means the report is disabled, which Set Feature never tells us. */
/* Copy out the last reply the sensor sent to a request, for inspection when
   a setting will not take. Returns how many bytes were copied. */
uint16_t imu_control_payload(uint8_t *out, uint16_t max);

/* Ask what the sensor did with a Set Feature. Reply is a Get Feature Response
   on channel 2. */
/* Ask the sensor to report how one of its measurements is configured -
   whether it is switched on, and how often it is being sent. */
void imu_request_feature_status(uint8_t report_id);

/* The first frame we transmitted, verbatim. The receive path can be checked
   against bytes on the wire; this is the only way to check the other half. */
/* Copy out the raw bytes most recently SENT to the sensor. The companion to
   imu_snapshot(), for checking whether a request went out as intended.
   Returns how many bytes were copied. */
uint16_t imu_tx_snapshot(uint8_t *out, uint16_t max);

/* Transmit a byte pattern on the IMU UART. Only used by the loopback test:
   with PA9 jumpered to PA10 the bytes come straight back, which proves the
   STM32 side works without the sensor being involved at all. */
/* Send a known pattern of bytes to the sensor, as a wiring test. */
void imu_tx_test_pattern(void);

/* Compare the received snapshot against the transmitted pattern. Returns the
   number of corrupted bytes and, via out_total, how many were checked. A byte
   count alone cannot see corruption - the right number of wrong bytes looks
   identical to success. */
/*
 * Check whether what was sent to the sensor comes back correctly, to tell a
 * broken connection apart from a broken sensor.
 *
 * Returns how many bytes did not match, and puts the number checked into
 * `out_total`. Zero mismatches means the wiring is sound and the problem lies
 * elsewhere.
 */
uint32_t imu_loopback_check(uint32_t *out_total);

#endif /* IMU_BNO085_H */
