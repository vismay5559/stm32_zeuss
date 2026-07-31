/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "extmem_manager.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

XSPI_HandleTypeDef hxspi2;

/* USER CODE BEGIN PV */
/*
 * Boot progress breadcrumb. Written at each checkpoint so that when Boot dies
 * before GPIO/serial are up - the only channels it has to report anything -
 * the debugger can still read exactly how far it got:
 *
 *   STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -r32 <addr-of-g_boot_stage> 4
 *
 * Lives in .data (initialised non-zero) so it is not confused with cleared
 * .bss, and volatile so the compiler cannot optimise the stores away.
 */
volatile uint32_t g_boot_stage = 0xFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_SBS_Init(void);
static void MX_XSPI2_Init(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /*
   * Bring the ST-LINK virtual COM port up EARLY. main() only calls
   * BSP_COM_Init much further down - long after MX_EXTMEM_MANAGER_Init() has
   * already run and failed silently. Initialising it here means printf()
   * works during external-memory setup, which is where the trouble is.
   *
   * Open a serial terminal at 115200 8N1 on the ST-LINK COM port to read it.
   */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  (void)BSP_COM_Init(COM1, &BspCOMInit);

  printf("\r\n\r\n===== nexus BOOT starting =====\r\n");
  printf("SYSCLK = %lu Hz\r\n", (unsigned long)HAL_RCC_GetSysClockFreq());
  printf("XSPI2 kernel clk = %lu Hz\r\n",
         (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_XSPI2));
  printf("--- EXTMEM init trace follows ---\r\n");

  /*
   * Board smoke test. Uncomment the #define below to blink LD1 (green, PD10)
   * forever, right after the clock comes up and BEFORE any peripheral or
   * external-flash init runs.
   *
   * If it blinks, you have proven: board power, ST-LINK/SWD, the toolchain,
   * and the 600 MHz clock tree - with nothing else able to get in the way.
   * (A wrong clock config shows up as an obviously wrong blink rate, which
   * makes this a clock check as well as a "is it alive" check.)
   *
   * Re-comment it to resume the normal boot path: Boot -> jump to the Appli
   * living in external XSPI2 flash at 0x70000000.
   */
/* #define BOOT_LED_SMOKE_TEST */
#ifdef BOOT_LED_SMOKE_TEST
  BSP_LED_Init(LED_GREEN);
  while (1)
  {
    BSP_LED_Toggle(LED_GREEN);
    /* 1 s on, 1 s off. Count it against a watch: 5 full on-off cycles in
       10 seconds means SysTick - and therefore the whole clock tree - is
       running at the 600 MHz the code thinks it is. */
    HAL_Delay(1000);
  }
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_SBS_Init();
  MX_XSPI2_Init();
  MX_EXTMEM_MANAGER_Init();
  /* USER CODE BEGIN 2 */
  /*
   * Boot progress marker. Yellow ON here means Boot got all the way through
   * peripheral + external-memory init and is about to hand over to the Appli.
   * The Appli turns it back OFF as the first thing it does, so:
   *
   *   yellow ON, red OFF   -> the jump never reached the Appli
   *   yellow ON, red ON    -> Boot hit Error_Handler (BOOT_Application failed)
   *   yellow OFF, red OFF  -> Appli started; hang is inside the Appli
   *   yellow OFF, red ON   -> Appli started, MPU region check failed
   *   yellow OFF, green 1 Hz -> everything works
   */
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_On(LED_YELLOW);

  /*
   * Stage-2 bisect. Uncomment to blink GREEN fast (5 Hz) right here, at the
   * point where Boot has finished ALL init - GPIO, GPDMA, SBS, XSPI2 and
   * EXTMEM_MANAGER - and is about to hand over to the Appli.
   *
   *   fast green blink -> Boot is fine; the fault is in BOOT_Application()
   *                       or in the Appli itself
   *   nothing          -> Boot hangs earlier, inside one of the MX_*_Init
   *                       calls above (XSPI2 / EXTMEM_MANAGER most likely)
   *
   * Deliberately GREEN and deliberately a *blink*: a blinking LED cannot be
   * confused with "off", and green at 1 Hz has already been read successfully
   * on this board, so 5 Hz vs 1 Hz vs nothing is unambiguous.
   */
/* #define BOOT_STAGE2_BLINK */
#ifdef BOOT_STAGE2_BLINK
  BSP_LED_Init(LED_GREEN);
  while (1)
  {
    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(100);   /* 5 Hz - clearly faster than the 1 Hz heartbeat */
  }
#endif

  /*
   * Stage-3 bisect. Stage 2 proved Boot reaches this point, so the fault is
   * inside BOOT_Application(). That does two things - MapMemory() then
   * JumpToApplication() - so run them apart and report between.
   *
   *   yellow ON  + red ON            MapMemory() returned an error
   *   yellow ON  + no green          MapMemory() hung
   *   yellow OFF + red ON            reading 0x70000000 hard-faulted:
   *                                  the XIP mapping is broken
   *   yellow OFF + green SLOW 1 Hz   mapping works AND the data is correct;
   *                                  fault is in JumpToApplication/the Appli
   *   yellow OFF + green FAST 5 Hz   mapping works but reads back wrong data
   */
/* #define BOOT_STAGE3_TEST */
#ifdef BOOT_STAGE3_TEST
  {
    extern BOOTStatus_TypeDef MapMemory(void);

    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_RED);

    if (BOOT_OK != MapMemory())
    {
      BSP_LED_On(LED_RED);
      while (1) {}
    }

    /* MapMemory() returned OK - record that by dropping yellow. */
    BSP_LED_Off(LED_YELLOW);

    /* Now actually read through the mapping. If XIP is misconfigured this
       faults, and HardFault_Handler lights red. */
    uint32_t sp = *(volatile uint32_t *)0x70000000u;

    /* 0x20010000 is the Appli's initial stack pointer - top of DTCM. */
    uint32_t period = (sp == 0x20010000u) ? 500u : 100u;

    while (1)
    {
      BSP_LED_Toggle(LED_GREEN);
      HAL_Delay(period);
    }
  }
#endif

  /*
   * Stage-4: report what EXTMEM_Init() actually returned.
   *
   * Stage 3 showed MapMemory() succeeding but the first memory-mapped read
   * hanging the bus - the classic signature of enabling mapped mode on a flash
   * that was never successfully configured. CubeMX throws away EXTMEM_Init()'s
   * return value, so that failure has been invisible until now.
   *
   * GREEN blinks N times, pauses 2 s, repeats. Count the blinks:
   *   steady 1 Hz   EXTMEM_OK - init succeeded, look elsewhere
   *   1 blink       EXTMEM_ERROR_NOTSUPPORTED
   *   2 blinks      EXTMEM_ERROR_UNKNOWNMEMORY
   *   3 blinks      EXTMEM_ERROR_DRIVER      <- expected if SFDP/comms failed
   *   4 blinks      EXTMEM_ERROR_SECTOR_SIZE
   *   5 blinks      EXTMEM_ERROR_INVALID_ID
   *   6 blinks      EXTMEM_ERROR_PARAM
   */
/* #define BOOT_STAGE4_EXTMEM_STATUS */
#ifdef BOOT_STAGE4_EXTMEM_STATUS
  {
    extern EXTMEM_StatusTypeDef g_extmem_status;

    BSP_LED_Init(LED_GREEN);
    BSP_LED_Off(LED_YELLOW);

    int code = (g_extmem_status == EXTMEM_OK) ? 0 : (int)(-(int)g_extmem_status);

    printf("--- EXTMEM init trace end ---\r\n");
    printf("EXTMEM_Init() returned %d (%s)\r\n", (int)g_extmem_status,
           (g_extmem_status == EXTMEM_OK)      ? "EXTMEM_OK" :
           (code == 1) ? "ERROR_NOTSUPPORTED"  :
           (code == 2) ? "ERROR_UNKNOWNMEMORY" :
           (code == 3) ? "ERROR_DRIVER"        :
           (code == 4) ? "ERROR_SECTOR_SIZE"   :
           (code == 5) ? "ERROR_INVALID_ID"    :
           (code == 6) ? "ERROR_PARAM"         : "unknown");
    printf("XSPI2 kernel clk (after MSP) = %lu Hz\r\n",
           (unsigned long)HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_XSPI2));

    if (g_extmem_status == EXTMEM_OK)
    {
      extern BOOTStatus_TypeDef MapMemory(void);

      printf("Calling MapMemory()... ");
      if (BOOT_OK != MapMemory())
      {
        printf("FAILED\r\n");
      }
      else
      {
        printf("ok\r\n");
        printf("Reading 0x70000000 through the mapping... ");
        uint32_t w0 = *(volatile uint32_t *)0x70000000u;
        uint32_t w1 = *(volatile uint32_t *)0x70000004u;
        printf("got 0x%08lX 0x%08lX\r\n", (unsigned long)w0, (unsigned long)w1);
        printf("expected 0x20010000 0x7000FA5D\r\n");
        printf(((w0 == 0x20010000u) && (w1 == 0x7000FA5Du))
               ? "XIP MAPPING WORKS - Appli image is readable\r\n"
               : "MAPPING RETURNS WRONG DATA\r\n");
      }
    }

    printf("Boot halted here on purpose - not jumping to the Appli.\r\n");

    while (1)
    {
      if (code == 0)
      {
        BSP_LED_Toggle(LED_GREEN);
        HAL_Delay(500);              /* steady 1 Hz = init was fine */
      }
      else
      {
        for (int i = 0; i < code; i++)
        {
          BSP_LED_On(LED_GREEN);
          HAL_Delay(200);
          BSP_LED_Off(LED_GREEN);
          HAL_Delay(200);
        }
        HAL_Delay(2000);             /* gap so the count is easy to read */
      }
    }
  }
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

  /* Launch the application */
  if (BOOT_OK != BOOT_Application())
  {
    Error_Handler();
  }
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL1.PLLM = 2;
  RCC_OscInitStruct.PLL1.PLLN = 100;
  RCC_OscInitStruct.PLL1.PLLP = 2;
  RCC_OscInitStruct.PLL1.PLLQ = 12;
  RCC_OscInitStruct.PLL1.PLLR = 2;
  RCC_OscInitStruct.PLL1.PLLS = 2;
  RCC_OscInitStruct.PLL1.PLLT = 2;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 100;
  RCC_OscInitStruct.PLL2.PLLP = 10;
  RCC_OscInitStruct.PLL2.PLLQ = 2;
  RCC_OscInitStruct.PLL2.PLLR = 2;
  RCC_OscInitStruct.PLL2.PLLS = 2;
  RCC_OscInitStruct.PLL2.PLLT = 2;
  RCC_OscInitStruct.PLL2.PLLFractional = 0;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK4|RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief SBS Initialization Function
  * @param None
  * @retval None
  */
static void MX_SBS_Init(void)
{

  /* USER CODE BEGIN SBS_Init 0 */

  /* USER CODE END SBS_Init 0 */

  /* USER CODE BEGIN SBS_Init 1 */

  /* USER CODE END SBS_Init 1 */
  /* USER CODE BEGIN SBS_Init 2 */

  /* USER CODE END SBS_Init 2 */

}

/**
  * @brief XSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_XSPI2_Init(void)
{

  /* USER CODE BEGIN XSPI2_Init 0 */

  /* USER CODE END XSPI2_Init 0 */

  XSPIM_CfgTypeDef sXspiManagerCfg = {0};

  /* USER CODE BEGIN XSPI2_Init 1 */

  /* USER CODE END XSPI2_Init 1 */
  /* XSPI2 parameter configuration*/
  hxspi2.Instance = XSPI2;
  hxspi2.Init.FifoThresholdByte = 4;
  hxspi2.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi2.Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
  hxspi2.Init.MemorySize = HAL_XSPI_SIZE_256MB;
  hxspi2.Init.ChipSelectHighTimeCycle = 2;
  hxspi2.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi2.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi2.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi2.Init.ClockPrescaler = 3;
  hxspi2.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hxspi2.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
  hxspi2.Init.MaxTran = 0;
  hxspi2.Init.Refresh = 0;
  hxspi2.Init.MemorySelect = HAL_XSPI_CSSEL_NCS1;
  if (HAL_XSPI_Init(&hxspi2) != HAL_OK)
  {
    Error_Handler();
  }
  sXspiManagerCfg.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
  sXspiManagerCfg.IOPort = HAL_XSPIM_IOPORT_2;
  sXspiManagerCfg.Req2AckTime = 1;
  if (HAL_XSPIM_Config(&hxspi2, &sXspiManagerCfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN XSPI2_Init 2 */

  /* USER CODE END XSPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOM_CLK_ENABLE();
  __HAL_RCC_GPION_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /* Disables all MPU regions */
  for(uint8_t i=0; i<__MPU_REGIONCOUNT; i++)
  {
    HAL_MPU_DisableRegion(i);
  }

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x70000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x24070000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8KB;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Make the failure visible - otherwise Boot dies completely silently and
     looks identical to "the Appli hung". */
  BSP_LED_Init(LED_RED);
  BSP_LED_On(LED_RED);

  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
