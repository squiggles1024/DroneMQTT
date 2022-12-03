/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    octospi.c
  * @brief   This file provides code for the configuration
  *          of the OCTOSPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "octospi.h"

/* USER CODE BEGIN 0 */
#include <stdint.h>
/* Read Operations */
#define READ_CMD                                0x00
#define READ_LINEAR_BURST_CMD                   0x20
#define READ_HYBRID_BURST_CMD                   0x3F

/* Write Operations */
#define WRITE_CMD                               0x80
#define WRITE_LINEAR_BURST_CMD                  0xA0
#define WRITE_HYBRID_BURST_CMD                  0xBF

/* Default dummy clocks cycles */
#define DUMMY_CLOCK_CYCLES_READ                 5
#define DUMMY_CLOCK_CYCLES_WRITE                4
/* USER CODE END 0 */

OSPI_HandleTypeDef hospi1;

/* OCTOSPI1 init function */
void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
  HAL_OSPI_DLYB_CfgTypeDef HAL_OSPI_DLYB_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_APMEMORY;
  hospi1.Init.DeviceSize = 23;
  hospi1.Init.ChipSelectHighTime = 2;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 2;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi1.Init.ChipSelectBoundary = 10;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 320;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.DQSPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_OSPI_DLYB_Cfg_Struct.Units = 0;
  HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = 0;
  if (HAL_OSPI_DLYB_SetConfig(&hospi1, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

void HAL_OSPI_MspInit(OSPI_HandleTypeDef* ospiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(ospiHandle->Instance==OCTOSPI1)
  {
  /* USER CODE BEGIN OCTOSPI1_MspInit 0 */

  /* USER CODE END OCTOSPI1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
    PeriphClkInit.OspiClockSelection = RCC_OSPICLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* OCTOSPI1 clock enable */
    __HAL_RCC_OSPIM_CLK_ENABLE();
    __HAL_RCC_OSPI1_CLK_ENABLE();

    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**OCTOSPI1 GPIO Configuration
    PI0     ------> OCTOSPIM_P1_IO5
    PE3     ------> OCTOSPIM_P1_DQS
    PD7     ------> OCTOSPIM_P1_IO7
    PF8     ------> OCTOSPIM_P1_IO0
    PH2     ------> OCTOSPIM_P1_IO4
    PF7     ------> OCTOSPIM_P1_IO2
    PF9     ------> OCTOSPIM_P1_IO1
    PF6     ------> OCTOSPIM_P1_IO3
    PC3     ------> OCTOSPIM_P1_IO6
    PB10     ------> OCTOSPIM_P1_CLK
    PB11     ------> OCTOSPIM_P1_NCS
    */
    GPIO_InitStruct.Pin = OCTOSPI_R_IO5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO5_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OCTOSPI_R_DQS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_DQS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OCTOSPI_R_IO7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO7_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OCTOSPI_R_IO0_Pin|OCTOSPI_R_IO2_Pin|OCTOSPI_R_IO1_Pin|OCTOSPI_R_IO3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OCTOSPI_R_IO4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO4_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OCTOSPI_R_IO6_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO6_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OCTOSPI_R_CLK_P_Pin|OCTOSPI_R_NCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN OCTOSPI1_MspInit 1 */

  /* USER CODE END OCTOSPI1_MspInit 1 */
  }
}

void HAL_OSPI_MspDeInit(OSPI_HandleTypeDef* ospiHandle)
{

  if(ospiHandle->Instance==OCTOSPI1)
  {
  /* USER CODE BEGIN OCTOSPI1_MspDeInit 0 */

  /* USER CODE END OCTOSPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_OSPIM_CLK_DISABLE();
    __HAL_RCC_OSPI1_CLK_DISABLE();

    /**OCTOSPI1 GPIO Configuration
    PI0     ------> OCTOSPIM_P1_IO5
    PE3     ------> OCTOSPIM_P1_DQS
    PD7     ------> OCTOSPIM_P1_IO7
    PF8     ------> OCTOSPIM_P1_IO0
    PH2     ------> OCTOSPIM_P1_IO4
    PF7     ------> OCTOSPIM_P1_IO2
    PF9     ------> OCTOSPIM_P1_IO1
    PF6     ------> OCTOSPIM_P1_IO3
    PC3     ------> OCTOSPIM_P1_IO6
    PB10     ------> OCTOSPIM_P1_CLK
    PB11     ------> OCTOSPIM_P1_NCS
    */
    HAL_GPIO_DeInit(OCTOSPI_R_IO5_GPIO_Port, OCTOSPI_R_IO5_Pin);

    HAL_GPIO_DeInit(OCTOSPI_R_DQS_GPIO_Port, OCTOSPI_R_DQS_Pin);

    HAL_GPIO_DeInit(OCTOSPI_R_IO7_GPIO_Port, OCTOSPI_R_IO7_Pin);

    HAL_GPIO_DeInit(GPIOF, OCTOSPI_R_IO0_Pin|OCTOSPI_R_IO2_Pin|OCTOSPI_R_IO1_Pin|OCTOSPI_R_IO3_Pin);

    HAL_GPIO_DeInit(OCTOSPI_R_IO4_GPIO_Port, OCTOSPI_R_IO4_Pin);

    HAL_GPIO_DeInit(OCTOSPI_R_IO6_GPIO_Port, OCTOSPI_R_IO6_Pin);

    HAL_GPIO_DeInit(GPIOB, OCTOSPI_R_CLK_P_Pin|OCTOSPI_R_NCS_Pin);

  /* USER CODE BEGIN OCTOSPI1_MspDeInit 1 */

  /* USER CODE END OCTOSPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void OSPI_RamInit(void){
	  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
	  HAL_OSPI_DLYB_CfgTypeDef HAL_OSPI_DLYB_Cfg_Struct = {0};

	  hospi1.Instance = OCTOSPI1;
	  hospi1.Init.FifoThreshold = 1;
	  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
	  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_APMEMORY;
	  hospi1.Init.DeviceSize = 23;
	  hospi1.Init.ChipSelectHighTime = 1;
	  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
	  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
	  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
	  hospi1.Init.ClockPrescaler = 2;
	  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
	  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
	  hospi1.Init.ChipSelectBoundary = 0;
	  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
	  hospi1.Init.MaxTran = 0;
	  hospi1.Init.Refresh = 320;
	  HAL_OSPI_Init(&hospi1);

	  sOspiManagerCfg.ClkPort = 1;
	  sOspiManagerCfg.DQSPort = 1;
	  sOspiManagerCfg.NCSPort = 1;
	  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
	  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
	  HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);

	  HAL_OSPI_DLYB_Cfg_Struct.Units = 0;
	  HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = 0;
	  HAL_OSPI_DLYB_SetConfig(&hospi1, &HAL_OSPI_DLYB_Cfg_Struct);


}

void OSPI_EnableMemMappedMode(void){
	OSPI_RegularCmdTypeDef sCommand;
	OSPI_MemoryMappedTypeDef sMemMappedCfg;
	  sCommand.OperationType      = HAL_OSPI_OPTYPE_WRITE_CFG;
	  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
	  sCommand.Instruction        = WRITE_CMD;
	  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
	  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
	  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
	  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
	  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
	  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
	  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
	  sCommand.DummyCycles        = DUMMY_CLOCK_CYCLES_WRITE;
	  sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
	  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

	  if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sCommand.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
	  sCommand.Instruction   = READ_CMD;
	  sCommand.DummyCycles   = DUMMY_CLOCK_CYCLES_READ;

	  if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_ENABLE;
	  sMemMappedCfg.TimeOutPeriod     = 0x34;

	  if (HAL_OSPI_MemoryMapped(&hospi1, &sMemMappedCfg) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void OSPI_DelayBlockConfig(void){
	  LL_DLYB_CfgTypeDef dlyb_cfg,dlyb_cfg_test;
	  /* Delay block configuration ------------------------------------------------ */
	  HAL_OSPI_DLYB_GetClockPeriod(&hospi1,&dlyb_cfg);

	  /*when DTR, PhaseSel is divided by 4 (emperic value)*/
	  dlyb_cfg.PhaseSel /=4;

	  /* save the present configuration for check*/
	  dlyb_cfg_test = dlyb_cfg;

	  /*set delay block configuration*/
	  HAL_OSPI_DLYB_SetConfig(&hospi1,&dlyb_cfg);

	  /*check the set value*/
	  HAL_OSPI_DLYB_GetConfig(&hospi1,&dlyb_cfg);
	  if ((dlyb_cfg.PhaseSel != dlyb_cfg_test.PhaseSel) || (dlyb_cfg.Units != dlyb_cfg_test.Units))
	  {
	    Error_Handler();
	  }
}
/* USER CODE END 1 */
