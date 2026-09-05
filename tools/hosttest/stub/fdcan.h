#ifndef HOSTTEST_FDCAN_H
#define HOSTTEST_FDCAN_H

/*
 * Host stand-in for the FDCAN half of the HAL.
 *
 * act_odrive.c talks to the ten motor drives over two CAN wires. On the board
 * that means the FDCAN peripheral; here it means this file. It is a working
 * model rather than a set of empty functions - it has a transmit queue that
 * fills up, a receive queue a test can drop frames into, and the error
 * reporting the driver reads - because the behaviour worth testing IS what
 * the driver does when those things misbehave.
 *
 * Two details are copied from the real part on purpose:
 *
 *   - the hardware transmit queue holds exactly three frames. The driver's
 *     own software queue exists because of that number; a stub with room for
 *     twenty would never exercise it.
 *   - a frame that will not fit is refused rather than dropped, so the driver
 *     has the chance to keep it and try again.
 *
 * The names and field spellings must match
 * Drivers/STM32H7RSxx_HAL_Driver/Inc/stm32h7rsxx_hal_fdcan.h. That is not
 * checked automatically; the firmware build in CI is what catches a drift,
 * because the same source file has to compile against the real header there.
 */

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 }
HAL_StatusTypeDef;

/* Only the fields act_odrive.c sets or reads are modelled. */
#define FDCAN_STANDARD_ID           0x00000000u
#define FDCAN_DATA_FRAME            0x00000000u
#define FDCAN_DLC_BYTES_8           0x00000008u
#define FDCAN_ESI_ACTIVE            0x80000000u
#define FDCAN_BRS_OFF               0x00000000u
#define FDCAN_BRS_ON                0x00100000u
#define FDCAN_CLASSIC_CAN           0x00000000u
#define FDCAN_FD_CAN                0x00200000u
#define FDCAN_NO_TX_EVENTS          0x00000000u
#define FDCAN_FILTER_RANGE          0x00000000u
#define FDCAN_FILTER_TO_RXFIFO0     0x00000001u
#define FDCAN_FILTER_REMOTE         0x00000002u
#define FDCAN_REJECT                0x00000002u
#define FDCAN_RX_FIFO0              0x00000040u
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 0x00000001u

typedef struct { int id; } FDCAN_HandleTypeDef;

typedef struct
{
    uint32_t IdType;
    uint32_t FilterIndex;
    uint32_t FilterType;
    uint32_t FilterConfig;
    uint32_t FilterID1;
    uint32_t FilterID2;
} FDCAN_FilterTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
    uint32_t DataLength;
} FDCAN_RxHeaderTypeDef;

typedef struct
{
    uint32_t LastErrorCode;
    uint32_t ErrorPassive;
    uint32_t Warning;
    uint32_t BusOff;
} FDCAN_ProtocolStatusTypeDef;

typedef struct
{
    uint32_t TxErrorCnt;
    uint32_t RxErrorCnt;
    uint32_t ErrorLogging;
} FDCAN_ErrorCountersTypeDef;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h,
                                         const FDCAN_FilterTypeDef *f);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h,
                                               uint32_t std, uint32_t ext,
                                               uint32_t std_rmt, uint32_t ext_rmt);
HAL_StatusTypeDef HAL_FDCAN_ConfigTxDelayCompensation(FDCAN_HandleTypeDef *h,
                                                      uint32_t offset,
                                                      uint32_t filter);
HAL_StatusTypeDef HAL_FDCAN_EnableTxDelayCompensation(FDCAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,
                                                 uint32_t its, uint32_t lines);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,
                                                const FDCAN_TxHeaderTypeDef *hdr,
                                                const uint8_t *data);
uint32_t          HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h);
uint32_t          HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *h,
                                               uint32_t fifo);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo,
                                         FDCAN_RxHeaderTypeDef *hdr,
                                         uint8_t *data);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *h,
                                              FDCAN_ProtocolStatusTypeDef *ps);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *h,
                                             FDCAN_ErrorCountersTypeDef *ec);

/* --- what a test can see and control --------------------------------- */

/* Depth of the real part's transmit queue. Not configurable on hardware. */
#define HOST_CAN_TX_FIFO_DEPTH  3u

typedef struct
{
    uint32_t identifier;
    uint8_t  data[8];
} host_can_frame_t;

/* Forget everything: both wires, both directions, all the fault switches. */
void host_can_reset(void);

/* Every frame that has actually reached the wire on this bus, oldest first. */
uint32_t                host_can_sent_count(uint8_t bus);
const host_can_frame_t *host_can_sent(uint8_t bus, uint32_t index);
void                    host_can_forget_sent(void);

/*
 * Stop this wire carrying anything away. Frames still go into the three-deep
 * hardware queue and then it is full, which is what a wire nobody is
 * listening on looks like from the driver's side. host_can_wire_ok() lets
 * the backlog go out again.
 */
void host_can_wire_blocked(uint8_t bus, int blocked);

/* Refuse the next `n` transmit attempts on this bus outright. */
void host_can_refuse_tx(uint8_t bus, uint32_t n);

/* Hand the driver a frame, as if a drive had sent it. */
void host_can_deliver(uint8_t bus, uint32_t identifier, const uint8_t *data);

/* Make the next read of a delivered frame fail. */
void host_can_break_rx_read(uint8_t bus, int broken);

/* What the error reporting says about this wire. */
void host_can_set_bus_off(uint8_t bus, int off);
void host_can_set_tx_errors(uint8_t bus, uint32_t tec);
void host_can_break_status_read(uint8_t bus, int broken);
void host_can_break_counter_read(uint8_t bus, int broken);

/* How many times the driver has restarted this wire. */
uint32_t host_can_start_count(uint8_t bus);

#endif /* HOSTTEST_FDCAN_H */
