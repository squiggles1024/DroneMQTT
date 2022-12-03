/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dcmi.c
  * @brief   This file provides code for the configuration
  *          of the DCMI instances.
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
#include "dcmi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

DCMI_HandleTypeDef hdcmi;

/* DCMI init function */
void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

void HAL_DCMI_MspInit(DCMI_HandleTypeDef* dcmiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(dcmiHandle->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspInit 0 */

  /* USER CODE END DCMI_MspInit 0 */
    /* DCMI clock enable */
    __HAL_RCC_DCMI_PSSI_CLK_ENABLE();

    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DCMI GPIO Configuration
    PI6     ------> DCMI_D6
    PI4     ------> DCMI_D5
    PI7     ------> DCMI_D7
    PE1     ------> DCMI_D3
    PH14     ------> DCMI_D4
    PH8     ------> DCMI_HSYNC
    PB7     ------> DCMI_VSYNC
    PC8     ------> DCMI_D2
    PC7     ------> DCMI_D1
    PC6     ------> DCMI_D0
    PA6     ------> DCMI_PIXCLK
    */
    GPIO_InitStruct.Pin = CAM_D6_Pin|CAM_D5_Pin|CAM_D7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_DCMI;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAM_D3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_DCMI;
    HAL_GPIO_Init(CAM_D3_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAM_D4_Pin|CAM_HSYNC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_DCMI;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAM_VSYNC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_DCMI;
    HAL_GPIO_Init(CAM_VSYNC_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAM_D2_Pin|CAM_D1_Pin|CAM_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_DCMI;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAM_PIXCLK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_DCMI;
    HAL_GPIO_Init(CAM_PIXCLK_GPIO_Port, &GPIO_InitStruct);

    /* DCMI interrupt Init */
    HAL_NVIC_SetPriority(DCMI_PSSI_IRQn, 12, 0);
    HAL_NVIC_EnableIRQ(DCMI_PSSI_IRQn);
  /* USER CODE BEGIN DCMI_MspInit 1 */

  /* USER CODE END DCMI_MspInit 1 */
  }
}

void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* dcmiHandle)
{

  if(dcmiHandle->Instance==DCMI)
  {
  /* USER CODE BEGIN DCMI_MspDeInit 0 */

  /* USER CODE END DCMI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DCMI_PSSI_CLK_DISABLE();

    /**DCMI GPIO Configuration
    PI6     ------> DCMI_D6
    PI4     ------> DCMI_D5
    PI7     ------> DCMI_D7
    PE1     ------> DCMI_D3
    PH14     ------> DCMI_D4
    PH8     ------> DCMI_HSYNC
    PB7     ------> DCMI_VSYNC
    PC8     ------> DCMI_D2
    PC7     ------> DCMI_D1
    PC6     ------> DCMI_D0
    PA6     ------> DCMI_PIXCLK
    */
    HAL_GPIO_DeInit(GPIOI, CAM_D6_Pin|CAM_D5_Pin|CAM_D7_Pin);

    HAL_GPIO_DeInit(CAM_D3_GPIO_Port, CAM_D3_Pin);

    HAL_GPIO_DeInit(GPIOH, CAM_D4_Pin|CAM_HSYNC_Pin);

    HAL_GPIO_DeInit(CAM_VSYNC_GPIO_Port, CAM_VSYNC_Pin);

    HAL_GPIO_DeInit(GPIOC, CAM_D2_Pin|CAM_D1_Pin|CAM_D0_Pin);

    HAL_GPIO_DeInit(CAM_PIXCLK_GPIO_Port, CAM_PIXCLK_Pin);

    /* DCMI interrupt Deinit */
    HAL_NVIC_DisableIRQ(DCMI_PSSI_IRQn);
  /* USER CODE BEGIN DCMI_MspDeInit 1 */

  /* USER CODE END DCMI_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
