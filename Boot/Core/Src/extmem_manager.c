/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : extmem_manager.c
  * @version        : 1.0.0
  * @brief          : This file implements the extmem configuration
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
#include "extmem_manager.h"
#include <string.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Result of EXTMEM_Init(). 0 = EXTMEM_OK, negative values are the
   EXTMEM_ERROR_* codes in stm32_extmem.h. */
EXTMEM_StatusTypeDef g_extmem_status;

/* Number of data lines used to talk to the external flash.
   CubeMX configures 8LINES (octal). Lowering it is a useful bisect: SFDP init
   fails at step 11 (re-reading the SFDP header through the freshly configured
   octal mode), so if 1LINE succeeds the fault is in the switch into octal
   mode - not the wiring, the chip, or the SFDP data. */
#ifndef NEXUS_EXTMEM_LINES
#define NEXUS_EXTMEM_LINES  EXTMEM_LINK_CONFIG_1LINE
#endif

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init External memory manager
  * @retval None
  */
void MX_EXTMEM_MANAGER_Init(void)
{

  /* USER CODE BEGIN MX_EXTMEM_Init_PreTreatment */

  /* USER CODE END MX_EXTMEM_Init_PreTreatment */
  HAL_RCCEx_EnableClockProtection(RCC_CLOCKPROTECT_XSPI);

  /* Initialization of the memory parameters */
  memset(extmem_list_config, 0x0, sizeof(extmem_list_config));

  /* EXTMEMORY_1 */
  extmem_list_config[0].MemType = EXTMEM_NOR_SFDP;
  extmem_list_config[0].Handle = (void*)&hxspi2;
  extmem_list_config[0].ConfigType = NEXUS_EXTMEM_LINES;

  /* NOTE: CubeMX generates this call with the return value discarded, which is
     why a failed SFDP discovery is completely silent and only shows up later as
     a hung memory-mapped read. Captured here so Boot can report it. */
  g_extmem_status = EXTMEM_Init(EXTMEMORY_1, HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_XSPI2));

  /* USER CODE BEGIN MX_EXTMEM_Init_PostTreatment */

  /* USER CODE END MX_EXTMEM_Init_PostTreatment */
}
