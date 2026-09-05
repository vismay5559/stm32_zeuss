# CAN / ODrive interface — extract for ODrive support

STM32H7S3L8 (NUCLEO-H7S3L8) driving ODrive S1 over CAN-FD. Verbatim from the
project, nothing paraphrased.

## Setup

| | |
|---|---|
| Controller | STM32H7S3L8, FDCAN1, ST HAL |
| FDCAN kernel clock | PLL2P = **80 MHz**, `ClockDivider = FDCAN_CLOCK_DIV1` |
| Transceiver | TI **ISO1042** isolated, STM32 side only |
| Drives | ODrive **S1**, fw **0.6.12**, hw **5.2.0** |
| Nominal / data bitrate | 1 Mbit / **2 Mbit**, BRS on |
| Topology | linear daisy chain: STM32 -> node 1 -> node 3 -> node 4 |
| Node IDs | 1 = hip_pitch, 3 = knee, 4 = ankle |
| Control rate | 1 kHz `Set_Input_Pos` per node |

Message rates are set in odrivetool, not by this firmware: heartbeat ~100 Hz,
encoder estimates ~1000 Hz, torques ~100 Hz per node. Measured bus load ~33%
with two nodes streaming.

## Bit timing

80 MHz kernel clock, `Prescaler = 1` on both phases:

| phase | Seg1 | Seg2 | SJW | tq | bitrate | sample point |
|---|---|---|---|---|---|---|
| nominal | 69 | 10 | 10 | 80 | 1 Mbit | 87.5% |
| data | 32 | 7 | 7 | 40 | 2 Mbit | 82.5% |

TDC enabled, `TdcOffset = 33 tq` = 412.5 ns = **82.5%** of a 500 ns data bit,
i.e. the SSP is placed at the same point in the bit as the data sample point.

`DataTimeSeg1 = 32` is the peripheral's ceiling (`IS_FDCAN_DATA_TSEG1` allows
1..32), so 82.5% is the highest data-phase sample point reachable at 2 Mbit with
`DataPrescaler = 1`. 87.5% would need `Seg1 = 34`.

History, since it may be relevant:

* **5 Mbit** (`Seg1 = 11`, `Seg2 = 4`, 16 tq, 200 ns bit) failed under load —
  `TEC` climbed to bus-off while **`REC` stayed at 0**, so every error was on
  frames we transmitted. The `TdcOffset` was left at 20 tq = 250 ns, which is
  **past the end of a 200 ns bit**. That was probably the fault.
* **2 Mbit at 75% / SSP 50%** ran clean on the bench (`TEC=0 REC=0`) but nodes
  further down the daisy chain later went silent one at a time.
* **2 Mbit at 82.5% / SSP 82.5%** (above) is the current configuration, on the
  advice that the S1 samples at 87.5% and the earlier sampling was too early
  for a daisy chain. Not yet validated on hardware at the time of writing.

## Peripheral init (CubeMX-generated)

```c
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Update SystemCoreClock variable according to RCC registers values. */
  SystemCoreClockUpdate();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
  /* Serial up early so the Appli can report progress on the ST-LINK VCOM
     (115200 8N1), same channel Boot uses. */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  (void)BSP_COM_Init(COM1, &BspCOMInit);

  printf("\r\n>>> APPLI running from XIP at 0x70000000 <<<\r\n");
  printf("APPLI: starting peripheral init\r\n");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* TEMPORARY per-peripheral tracing: whichever line prints last is the one
     that hung. Remove once the Appli boots cleanly. */
  printf("  GPIO...\r\n");    MX_GPIO_Init();
  printf("  GPDMA1...\r\n");  MX_GPDMA1_Init();
  printf("  FDCAN1...\r\n");  MX_FDCAN1_Init();
  printf("  FDCAN2...\r\n");  MX_FDCAN2_Init();
  printf("  SPI1...\r\n");    MX_SPI1_Init();
  printf("  USART1...\r\n");  MX_USART1_UART_Init();
  printf("  TIM2...\r\n");    MX_TIM2_Init();
  printf("  TIM6...\r\n");    MX_TIM6_Init();
  printf("  USB...\r\n");     MX_USB_DEVICE_Init();
  /*
   * XSPI2 is deliberately NOT initialised here.
   *
   * This application executes in place from the external flash behind XSPI2.
   * Boot configures that peripheral and puts it into memory-mapped mode before
   * jumping here; re-initialising it from code being fetched through it tears
   * down the mapping and the next instruction fetch hangs the bus forever -
   * no crash, no fault, just a frozen CPU the debugger cannot even attach to.
   *
   * XSPI2 has now been removed from the Appli context in the .ioc, so CubeMX
   * no longer generates the call at all. Previously it regenerated every time
   * and had to be commented out by hand.
   */
  
  /* USER CODE BEGIN 2 */
#if (NEXUS_MODE == NEXUS_MODE_LEG_CAN)
  printf("APPLI: mode = LEG_CAN (CAN-FD single leg test)\r\n");
  legtest_init();
#elif (NEXUS_MODE == NEXUS_MODE_IMU)
  printf("APPLI: mode = IMU (BNO085 test)\r\n");
  imutest_init();
#else
  printf("APPLI: peripheral init done, entering app_init()\r\n");
  app_init();
  printf("APPLI: app_init() done\r\n");
#endif
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#if (NEXUS_MODE == NEXUS_MODE_LEG_CAN)
  legtest_run();
#elif (NEXUS_MODE == NEXUS_MODE_IMU)
  imutest_run();
#else
  app_run();
#endif

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

## Filter, TDC and start

```c
static void bus_setup(void)
{
    FDCAN_FilterTypeDef f;

    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_RANGE;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x000u;   
    f.FilterID2    = 0x7FFu;   

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &f) != HAL_OK)
    {
        printf("!! HAL_FDCAN_ConfigFilter FAILED\r\n");
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        printf("!! HAL_FDCAN_ConfigGlobalFilter FAILED\r\n");
    }
#if LEGTEST_USE_CAN_FD
    if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, LEGTEST_TDC_OFFSET, 0u) != HAL_OK)
    {
        printf("!! HAL_FDCAN_ConfigTxDelayCompensation FAILED\r\n");
    }
    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
    {
        printf("!! HAL_FDCAN_EnableTxDelayCompensation FAILED\r\n");
    }
    printf("CAN FD: TDC on, SSP offset %u tq (%u ns past the measured"
           " loop delay)\r\n",
           (unsigned)LEGTEST_TDC_OFFSET, (unsigned)(LEGTEST_TDC_OFFSET * 25u / 2u));
#endif

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        printf("!! HAL_FDCAN_Start FAILED - the peripheral is not on the bus\r\n");
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        printf("!! HAL_FDCAN_ActivateNotification FAILED\r\n");
    }
}
```

## Command IDs

```c
#define ODRV_CMD_HEARTBEAT      0x001u
#define ODRV_CMD_SET_AXIS_STATE 0x007u
#define ODRV_CMD_GET_ENCODER    0x009u
#define ODRV_CMD_SET_CTRL_MODE  0x00Bu
#define ODRV_CMD_SET_INPUT_POS  0x00Cu
#define ODRV_CMD_SET_INPUT_VEL  0x00Du
#define ODRV_CTRL_MODE_VELOCITY 2u
#define ODRV_CTRL_MODE_POSITION 3u
#define ODRV_INPUT_MODE_PASSTHR 1u
#define ODRV_CMD_SET_POS_GAIN   0x01Au
#define ODRV_CMD_SET_VEL_GAINS  0x01Bu
#define ODRV_CMD_GET_TORQUES    0x01Cu
#define ODRV_AXIS_STATE_IDLE            1u
#define ODRV_AXIS_STATE_CLOSED_LOOP     8u
```

## Frame construction and transmit

```c
static void tx_enqueue(uint32_t node, uint32_t cmd, const uint8_t *data, uint32_t len)
{
    uint8_t next = (uint8_t)((s_txq_head + 1u) & TXQ_MASK);

    if (next == s_txq_tail)
    {
        s_txq_tail = (uint8_t)((s_txq_tail + 1u) & TXQ_MASK);
        s_txq_drop++;
    }

    s_txq[s_txq_head].identifier = (node << 5) | cmd;
    s_txq[s_txq_head].len        = (uint8_t)len;
    memcpy(s_txq[s_txq_head].data, data, len);
    s_txq_head = next;
}

static uint8_t can_send(uint32_t node, uint32_t cmd, const uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef hdr;

#if LEGTEST_LISTEN_ONLY
    (void)node; (void)cmd; (void)data; (void)len;
    return 1;              
#else
    hdr.Identifier          = (node << 5) | cmd;
    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = (len == 0u) ? FDCAN_DLC_BYTES_0 :
                              (len == 8u) ? FDCAN_DLC_BYTES_8 :
                                            FDCAN_DLC_BYTES_4;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
#if LEGTEST_USE_CAN_FD
    hdr.BitRateSwitch       = FDCAN_BRS_ON;
    hdr.FDFormat            = FDCAN_FD_CAN;
#else
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
#endif
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, (uint8_t *)data) != HAL_OK)
    {
        s_tx_fail++;
        return 0;
    }
    s_tx_ok++;
    return 1;
#endif
}
```

## What we send to each drive

```c
static void send_input_pos(int j, float pos, float vel_ff, float trq_ff)
{
    uint8_t  data[8];
    uint32_t bits;

    memcpy(&bits, &pos, sizeof(bits));
    data[0] = (uint8_t)(bits & 0xFFu);
    data[1] = (uint8_t)((bits >> 8) & 0xFFu);
    data[2] = (uint8_t)((bits >> 16) & 0xFFu);
    data[3] = (uint8_t)((bits >> 24) & 0xFFu);

    uint16_t v = (uint16_t)ff_thousandths(vel_ff);
    uint16_t q = (uint16_t)ff_thousandths(trq_ff);

    data[4] = (uint8_t)(v & 0xFFu);
    data[5] = (uint8_t)((v >> 8) & 0xFFu);
    data[6] = (uint8_t)(q & 0xFFu);
    data[7] = (uint8_t)((q >> 8) & 0xFFu);

    s_joint[j].cmd = pos;
    tx_enqueue(s_node_id[j], ODRV_CMD_SET_INPUT_POS, data, 8u);
}

static void send_controller_mode(int j)
{
    uint8_t data[8];

#if LEGTEST_VEL_POKE
    data[0] = (uint8_t)ODRV_CTRL_MODE_VELOCITY;
#else
    data[0] = (uint8_t)ODRV_CTRL_MODE_POSITION;
#endif
    data[1] = 0; data[2] = 0; data[3] = 0;
    data[4] = (uint8_t)ODRV_INPUT_MODE_PASSTHR;
    data[5] = 0; data[6] = 0; data[7] = 0;

    tx_enqueue(s_node_id[j], ODRV_CMD_SET_CTRL_MODE, data, 8u);
}

/* Little-endian float32 into a byte buffer, the ODrive wire convention. */
static void put_f32(uint8_t *d, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    d[0] = (uint8_t)(bits & 0xFFu);
    d[1] = (uint8_t)((bits >> 8) & 0xFFu);
    d[2] = (uint8_t)((bits >> 16) & 0xFFu);
    d[3] = (uint8_t)((bits >> 24) & 0xFFu);
}

/*
 * Set_Pos_Gain (0x01A): float32 pos_gain.
 * Set_Vel_Gains (0x01B): float32 vel_gain, float32 vel_integrator_gain.
 *
 * Sent before Set_Controller_Mode so the loop is already tuned the moment the
 * axis is energised, rather than running one arming cycle on whatever the
 * drive had saved.
 */
static void send_gains(int j)
{
    uint8_t data[8];

    if (s_pos_gain[j] >= 0.0f)
    {
        put_f32(&data[0], s_pos_gain[j]);
        data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
        tx_enqueue(s_node_id[j], ODRV_CMD_SET_POS_GAIN, data, 4u);
    }

    if ((s_vel_gain[j] >= 0.0f) && (s_vel_int_gain[j] >= 0.0f))
    {
        put_f32(&data[0], s_vel_gain[j]);
        put_f32(&data[4], s_vel_int_gain[j]);
        tx_enqueue(s_node_id[j], ODRV_CMD_SET_VEL_GAINS, data, 8u);
    }

    if ((s_pos_gain[j] >= 0.0f) || (s_vel_gain[j] >= 0.0f))
    {
        printf("  gains -> %s: pos %.2f  vel %.3f  vel_int %.2f\r\n",
               s_joint_name[j], (double)s_pos_gain[j],
               (double)s_vel_gain[j], (double)s_vel_int_gain[j]);
    }
}

static void send_axis_state(int j, uint32_t state)
{
    uint8_t data[4];

    data[0] = (uint8_t)(state & 0xFFu);
    data[1] = (uint8_t)((state >> 8) & 0xFFu);
    data[2] = (uint8_t)((state >> 16) & 0xFFu);
    data[3] = (uint8_t)((state >> 24) & 0xFFu);

    tx_enqueue(s_node_id[j], ODRV_CMD_SET_AXIS_STATE, data, 4u);
}
```

## Arbitrary parameter access (RxSdo 0x004 / TxSdo 0x005)

```c
/*
 * Blocking. Only ever called from legtest_init(), before the 1 kHz tick is
 * started, so pumping the queue and draining RX inline here is safe.
 */
static uint8_t sdo_wait(volatile uint8_t *flag, uint32_t ms)
{
    for (uint32_t i = 0u; i < ms; i++)
    {
        tx_pump();
        legtest_on_rx();
        if (*flag) { return 1u; }
        HAL_Delay(1);
    }
    return 0u;
}

/*
 * The endpoint numbers this file hard-codes are only meaningful for one
 * firmware/hardware pair. Ask the drive what it is and refuse to write if it
 * disagrees - a mismatched write lands on whatever parameter happens to sit at
 * that number, which is far worse than not writing at all.
 */
static uint8_t odrv_check_version(uint8_t node)
{
    uint8_t empty[8] = { 0 };

    s_ver_node = node;
    s_ver_got  = 0u;
    tx_enqueue(node, ODRV_CMD_GET_VERSION, empty, 0u);

    if (!sdo_wait(&s_ver_got, 250u))
    {
        printf("  node %u: no reply to Get_Version - skipped\r\n", (unsigned)node);
        return 0u;
    }

    uint8_t hw_line = s_ver[1], hw_ver = s_ver[2], hw_var = s_ver[3];
    uint8_t fw_maj  = s_ver[4], fw_min = s_ver[5], fw_rev = s_ver[6];

    if ((hw_line != EP_JSON_HW_LINE) || (hw_ver != EP_JSON_HW_VER) ||
        (hw_var != EP_JSON_HW_VAR)   || (fw_maj != EP_JSON_FW_MAJOR) ||
        (fw_min != EP_JSON_FW_MINOR) || (fw_rev != EP_JSON_FW_REV))
    {
        printf("  node %u: hw %u.%u.%u fw %u.%u.%u does not match the endpoint\r\n"
               "          table (hw %u.%u.%u fw %u.%u.%u) - NOT writing.\r\n"
               "          Fetch that drive's own flat_endpoints.json.\r\n",
               (unsigned)node, hw_line, hw_ver, hw_var, fw_maj, fw_min, fw_rev,
               EP_JSON_HW_LINE, EP_JSON_HW_VER, EP_JSON_HW_VAR,
               EP_JSON_FW_MAJOR, EP_JSON_FW_MINOR, EP_JSON_FW_REV);
        return 0u;
    }
    return 1u;
}

static uint8_t sdo_read_f32(uint8_t node, uint16_t ep, float *out)
{
    uint8_t d[4];

    d[0] = SDO_OP_READ;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);
    d[3] = 0u;

    s_sdo_node = node;
    s_sdo_ep   = ep;
    s_sdo_got  = 0u;
    tx_enqueue(node, ODRV_CMD_RX_SDO, d, 4u);

    if (!sdo_wait(&s_sdo_got, 250u)) { return 0u; }

    *out = le_f32((const uint8_t *)s_sdo_val);
    return 1u;
}

static void sdo_write_f32(uint8_t node, uint16_t ep, float v)
{
    uint8_t d[8];

    d[0] = SDO_OP_WRITE;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);
    d[3] = 0u;
    put_f32(&d[4], v);

    tx_enqueue(node, ODRV_CMD_RX_SDO, d, 8u);
    for (uint32_t i = 0u; i < 20u; i++) { tx_pump(); HAL_Delay(1); }
}

static void spi_err_rate_one(uint8_t node, const char *name)
{
    float before = 0.0f, after = 0.0f;

    if (s_node_seen[node] == 0u)
    {
        printf("  node %u %-9s : silent on the scan - skipped\r\n",
               (unsigned)node, name);
        return;
    }
    if (!odrv_check_version(node)) { return; }

    if (!sdo_read_f32(node, EP_SPI_ENC0_MAX_ERROR_RATE, &before))
    {
        printf("  node %u %-9s : no reply reading the endpoint - skipped\r\n",
               (unsigned)node, name);
        return;
    }

    sdo_write_f32(node, EP_SPI_ENC0_MAX_ERROR_RATE,
                  LEGTEST_SPI_MAX_ERROR_RATE);

    if (!sdo_read_f32(node, EP_SPI_ENC0_MAX_ERROR_RATE, &after))
    {
        printf("  node %u %-9s : wrote %.3f but could not read it back\r\n",
               (unsigned)node, name, (double)LEGTEST_SPI_MAX_ERROR_RATE);
        return;
    }

    float want = LEGTEST_SPI_MAX_ERROR_RATE;
    float d    = (after > want) ? (after - want) : (want - after);

    printf("  node %u %-9s : max_error_rate %.4f -> %.4f%s\r\n",
           (unsigned)node, name, (double)before, (double)after,
           (d < 1e-6f) ? "" : "   !! DID NOT TAKE");

#if LEGTEST_SDO_SAVE
    if (d < 1e-6f)
    {
        printf("  node %u %-9s : save_configuration()\r\n", (unsigned)node, name);
        sdo_call(node, EP_SAVE_CONFIGURATION);
        HAL_Delay(500);
    }
#else
    (void)sdo_call;
#endif
}
```

## Node lookup and little-endian helpers

```c
static int joint_from_node(uint32_t node)
{
    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (s_node_id[j] == node) { return j; }
    }
    return -1;
}

static int monitor_from_node(uint32_t node)
{
#if (MONITOR_COUNT > 0)
    for (int m = 0; m < MONITOR_COUNT; m++)
    {
        if (s_mon_node[m] == node) { return m; }
    }
#else
    (void)node;
#endif
    return -1;
}

static float le_f32(const uint8_t *p)
{
    uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float f;
    memcpy(&f, &v, sizeof(f));
    return f;
}

static uint32_t le_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
```

## TX pump, bus-off and error-passive recovery, disarm

```c
static void can_busoff_poll(void)
{
    FDCAN_ProtocolStatusTypeDef ps;

    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);

    /*
     * Error-passive is not bus-off, so the recovery below never fired for it.
     * But once the drives disarm and stop ACKing, TX fills, txfifo_free hits 0
     * and every queued frame is dropped forever - qdrop climbing by 2000/s
     * with tx=0 and rx=0, which is what a wedged bus looks like from here.
     * Drop the backlog so the queue is not spending the whole tick failing.
     */
    if (ps.ErrorPassive && (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u))
    {
        s_txq_tail = s_txq_head;
    }

    if (ps.BusOff == 0u)
    {
        return;
    }
    if (READ_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT) == 0u)
    {
        return;                 
    }

    s_busoff_count++;

    HAL_FDCAN_AbortTxRequest(&hfdcan1,
                             FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 |
                             FDCAN_TX_BUFFER2);

    CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
}

static void tx_pump(void)
{
    can_busoff_poll();

    while (s_txq_tail != s_txq_head)
    {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0u)
        {
            break;                      
        }

        const txq_entry_t *e = &s_txq[s_txq_tail];

        if (!can_send(e->identifier >> 5, e->identifier & 0x1Fu, e->data, e->len))
        {
            break;                      
        }

        s_txq_tail = (uint8_t)((s_txq_tail + 1u) & TXQ_MASK);
    }
}

static void disarm_all(void)
{
    s_txq_tail = s_txq_head;

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        if (s_joint_live[j]) { send_axis_state(j, ODRV_AXIS_STATE_IDLE); }
    }

    for (uint32_t spin = 0u;
         (s_txq_tail != s_txq_head) && (spin < 100000u);
         spin++)
    {
        tx_pump();
    }
}
```

## Feedforward packing and velocity command

```c
static int16_t ff_thousandths(float v)
{
    float scaled = v * 1000.0f;

    if (scaled >  32767.0f) { return  32767; }
    if (scaled < -32768.0f) { return -32768; }

    return (int16_t)scaled;
}

static void send_input_vel(int j, float vel)
{
    uint8_t  data[8];
    uint32_t bits;

    memcpy(&bits, &vel, sizeof(bits));
    data[0] = (uint8_t)(bits & 0xFFu);
    data[1] = (uint8_t)((bits >> 8) & 0xFFu);
    data[2] = (uint8_t)((bits >> 16) & 0xFFu);
    data[3] = (uint8_t)((bits >> 24) & 0xFFu);
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;   

    s_joint[j].cmd = vel;
    tx_enqueue(s_node_id[j], ODRV_CMD_SET_INPUT_VEL, data, 8u);
}
```

## SDO function call and the boot-time write

```c
static void sdo_call(uint8_t node, uint16_t ep)
{
    uint8_t d[4];

    d[0] = SDO_OP_WRITE;
    d[1] = (uint8_t)(ep & 0xFFu);
    d[2] = (uint8_t)(ep >> 8);
    d[3] = 0u;

    tx_enqueue(node, ODRV_CMD_RX_SDO, d, 4u);
    for (uint32_t i = 0u; i < 20u; i++) { tx_pump(); HAL_Delay(1); }
}

static void apply_encoder_config(void)
{
#if LEGTEST_SET_SPI_ERR_RATE
    printf("\r\nspi_encoder0.config.max_error_rate -> %.3f  "
           "(endpoint %u, fw %u.%u.%u / hw %u.%u.%u)\r\n",
           (double)LEGTEST_SPI_MAX_ERROR_RATE,
           (unsigned)EP_SPI_ENC0_MAX_ERROR_RATE,
           EP_JSON_FW_MAJOR, EP_JSON_FW_MINOR, EP_JSON_FW_REV,
           EP_JSON_HW_LINE, EP_JSON_HW_VER, EP_JSON_HW_VAR);

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        spi_err_rate_one(s_node_id[j], s_joint_name[j]);
    }
#if (MONITOR_COUNT > 0)
    for (int m = 0; m < MONITOR_COUNT; m++)
    {
        spi_err_rate_one(s_mon_node[m], s_mon_name[m]);
    }
#endif

#if !LEGTEST_SDO_SAVE
    printf("  not saved to flash - this is a runtime write and is lost on the\r\n"
           "  next power cycle. Set LEGTEST_SDO_SAVE 1 to persist it.\r\n");
#endif
    printf("\r\n");
#endif
}
```

## Diagnostics: error counters and the listen-only scan

```c
static const char *lec_name(uint32_t lec)
{
    switch (lec)
    {
    case 0: return "none";
    case 1: return "STUFF - bitrate mismatch or noise";
    case 2: return "FORM - frame format, often FD vs classic";
    case 3: return "ACK - we transmitted and NOBODY answered";
    case 4: return "BIT1 - drove recessive, read dominant";
    case 5: return "BIT0 - drove dominant, read recessive (shorted? no xcvr?)";
    case 6: return "CRC";
    default: return "no change since last read";
    }
}

static void can_status(void)
{
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef  ec;

    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
    HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);

    printf("    can: TEC=%lu REC=%lu txfifo_free=%lu%s%s%s\r\n",
           (unsigned long)ec.TxErrorCnt, (unsigned long)ec.RxErrorCnt,
           (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1),
           ps.BusOff       ? "  [BUS-OFF]"       : "",
           ps.ErrorPassive ? "  [ERROR-PASSIVE]" : "",
           ps.Warning      ? "  [WARNING]"       : "");
    printf("    last error: %s\r\n", lec_name(ps.LastErrorCode));

    if (ps.BusOff)
    {
        printf("    -> BUS-OFF (event %lu): rejoining\r\n",
               (unsigned long)s_busoff_count);
    }
    else if (s_busoff_count != 0u)
    {
        printf("    -> recovered from %lu bus-off event(s) so far\r\n",
               (unsigned long)s_busoff_count);
    }
}

static void bus_scan(void)
{
    printf("\r\nscanning the bus for %u ms - not transmitting...\r\n",
           (unsigned)LEGTEST_SCAN_MS);

    for (uint32_t i = 0; i < LEGTEST_SCAN_MS; i += 10u)
    {
        HAL_Delay(10);
        legtest_on_rx();          
#if LEGTEST_TRACE_RX
        trace_drain(2);           
#endif
    }

    int found = 0;

    for (int n = 0; n < 64; n++)
    {
        if (s_node_seen[n] == 0u)
        {
            continue;
        }

        found++;
        printf("  node %-2d  %5u frames  axis_state %u  axis_error 0x%08lX%s\r\n",
               n, (unsigned)s_node_seen[n], (unsigned)s_node_state[n],
               (unsigned long)s_node_err[n],
               (joint_from_node((uint32_t)n) >= 0)   ? "   <-- configured" :
               (monitor_from_node((uint32_t)n) >= 0) ? "   <-- monitored" : "");
    }

    can_status();

    if (found == 0)
    {
        printf("  NOTHING ON THE BUS.\r\n");
    }

    /*
     * One silent drive used to disable closed loop for the whole leg. It no
     * longer does: each joint stands or falls on its own, and the run goes
     * ahead with whatever answered. Only a completely silent bus stops it.
     */
    int live = 0;

    for (int j = 0; j < JOINT_COUNT; j++)
    {
        s_joint_live[j] = (s_node_seen[s_node_id[j]] != 0u) ? 1u : 0u;

        if (s_joint_live[j])
        {
            live++;
        }
        else
        {
            printf("  !! node %u (%s) never answered - that joint is SKIPPED\r\n"
                   "     (not armed, not commanded, not captured). The rest of\r\n"
                   "     the leg still runs.\r\n",
                   (unsigned)s_node_id[j], s_joint_name[j]);
        }
    }

    s_scan_ok = (live > 0) ? 1u : 0u;

    if (!s_scan_ok)
    {
        printf("  !! no joint answered at all - closed loop DISABLED\r\n");
    }

    printf("\r\n");
}
```

## The 1 kHz loop (CAN parts only)

```c
/* The 1 kHz control loop, reduced to the statements that touch CAN.
   The full loop also evaluates the reference trajectory; omitted here. */
void legtest_run(void)
{
    for (;;)
    {
        tx_pump();                       /* drain the software TX queue */

        if (s_tick_pending == 0u) { continue; }
        s_tick_pending = 0;              /* set by the 1 kHz timer ISR  */
        s_tick++;

        float target[JOINT_COUNT]     = { 0.0f };
        float target_vel[JOINT_COUNT] = { 0.0f };

        /* ... trajectory fills target[] and target_vel[] ... */

        /* one Set_Input_Pos per live joint per tick -> 1 kHz per node */
        for (int j = 0; j < JOINT_COUNT; j++)
        {
            if (!s_joint_live[j]) { continue; }
            send_input_pos(j, target[j], target_vel[j], 0.0f);
        }

        /* re-arm any joint not in CLOSED_LOOP, every 2 s */
        if ((s_tick > LEGTEST_ARM_DELAY_MS) && ((s_tick % 2000u) == 500u))
        {
            for (int j = 0; j < JOINT_COUNT; j++)
            {
                if (s_joint_live[j] &&
                    (s_joint[j].axis_state != ODRV_AXIS_STATE_CLOSED_LOOP) &&
                    (s_joint[j].axis_error == 0u) &&
                    (s_joint[j].n_heartbeat > 0u))
                {
                    send_gains(j);
                    send_controller_mode(j);
                    send_axis_state(j, ODRV_AXIS_STATE_CLOSED_LOOP);
                }
            }
        }
    }
}

/* 1 kHz timer ISR */
void legtest_on_tick(void) { s_tick_pending++; }
```

## Receive path

```c
void legtest_on_rx(void)
{
    FDCAN_RxHeaderTypeDef hdr;
    uint8_t data[64];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK)
        {
            return;
        }

        s_rx_total++;

        uint32_t node = (hdr.Identifier >> 5) & 0x3Fu;
        uint32_t cmd  = hdr.Identifier & 0x1Fu;

#if LEGTEST_TRACE_RX
        {
            uint8_t next = (uint8_t)((s_trace_head + 1u) % TRACE_LEN);

            if (next != s_trace_tail)
            {
                s_trace[s_trace_head].id  = hdr.Identifier;
                s_trace[s_trace_head].len = 8u;
                s_trace[s_trace_head].fd  = (hdr.FDFormat == FDCAN_FD_CAN) ? 1u : 0u;
                s_trace[s_trace_head].brs = (hdr.BitRateSwitch == FDCAN_BRS_ON) ? 1u : 0u;
                for (int b = 0; b < 8; b++)
                {
                    s_trace[s_trace_head].data[b] = data[b];
                }
                s_trace_head = next;
            }
        }
#endif

        if (node < 64u)
        {
            if (s_node_seen[node] < 0xFFFFu)
            {
                s_node_seen[node]++;
            }
            if (cmd == ODRV_CMD_HEARTBEAT)
            {
                s_node_state[node] = data[4];
                s_node_err[node]   = le_u32(&data[0]);
            }
        }

        /* replies to a setup exchange, from any node - handled before the
           joint lookup so a monitor-only node can answer too */
        if (cmd == ODRV_CMD_GET_VERSION)
        {
            if (node == s_ver_node)
            {
                for (int b = 0; b < 8; b++) { s_ver[b] = data[b]; }
                s_ver_got = 1u;
            }
            continue;
        }
        if (cmd == ODRV_CMD_TX_SDO)
        {
            uint16_t ep = (uint16_t)data[1] | ((uint16_t)data[2] << 8);

            if ((node == s_sdo_node) && (ep == s_sdo_ep))
            {
                for (int b = 0; b < 4; b++) { s_sdo_val[b] = data[4 + b]; }
                s_sdo_got = 1u;
            }
            continue;
        }

        volatile joint_t *t = NULL;
        int j = joint_from_node(node);

        if (j >= 0)
        {
            t = &s_joint[j];
        }
#if (MONITOR_COUNT > 0)
        else
        {
            int m = monitor_from_node(node);
            if (m >= 0) { t = &s_mon[m]; }
        }
#endif
        if (t == NULL)
        {
            s_rx_unknown++;  
            continue;
        }

        t->last_rx_tick = s_tick;

        switch (cmd)
        {
        case ODRV_CMD_GET_ENCODER:
            t->pos = le_f32(&data[0]);
            t->vel = le_f32(&data[4]);
            t->n_encoder++;
            break;

        case ODRV_CMD_GET_TORQUES:
            t->torque = le_f32(&data[4]);
            t->n_torque++;
            break;

        case ODRV_CMD_HEARTBEAT:
            t->axis_error = le_u32(&data[0]);
            t->axis_state = data[4];
            t->n_heartbeat++;
            break;

        default:
            break;
        }
    }
}
```
