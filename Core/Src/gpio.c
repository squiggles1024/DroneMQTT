/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PC11   ------> UART4_RX
     PA15 (JTDI)   ------> UCPD1_CC1
     PI5   ------> OCTOSPIM_P2_NCS
     PH12   ------> OCTOSPIM_P2_IO7
     PC10   ------> UART4_TX
     PA14 (JTCK/SWCLK)   ------> DEBUG_JTCK-SWCLK
     PH10   ------> OCTOSPIM_P2_IO5
     PC14-OSC32_IN (PC14)   ------> RCC_OSC32_IN
     PB3 (JTDO/TRACESWO)   ------> DEBUG_JTDO-SWO
     PH11   ------> OCTOSPIM_P2_IO6
     PC15-OSC32_OUT (PC15)   ------> RCC_OSC32_OUT
     PF0   ------> OCTOSPIM_P2_IO0
     PH9   ------> OCTOSPIM_P2_IO4
     PF1   ------> OCTOSPIM_P2_IO1
     PF2   ------> OCTOSPIM_P2_IO2
     PA13 (JTMS/SWDIO)   ------> DEBUG_JTMS-SWDIO
     PA12   ------> USB_OTG_FS_DP
     PF3   ------> OCTOSPIM_P2_IO3
     PF4   ------> OCTOSPIM_P2_CLK
     PA11   ------> USB_OTG_FS_DM
     PF10   ------> MDF1_CCK1
     PE10   ------> ADF1_SDI0
     PE9   ------> ADF1_CCK0
     PF12   ------> OCTOSPIM_P2_DQS
     PB15   ------> UCPD1_CC2
     PB1   ------> MDF1_SDI0
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UCPD_PWR_GPIO_Port, UCPD_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAM_XSDN_GPIO_Port, CAM_XSDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAM_RSTI_GPIO_Port, CAM_RSTI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_RED_Pin|LED_GREEN_Pin|Mems_VL53_xshut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WRLS_WKUP_B_GPIO_Port, WRLS_WKUP_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MXCHIP_NSS_GPIO_Port, MXCHIP_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Mems_STSAFE_RESET_Pin|MXCHIP_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = MXCHIP_FLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MXCHIP_FLOW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = WRLS_UART4_RX_Pin|WRLS_UART4_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = USB_UCPD_CC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_UCPD_CC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = CAM_PLUG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CAM_PLUG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = OCTOSPI_F_NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
  HAL_GPIO_Init(OCTOSPI_F_NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PHPin PHPin PHPin PHPin */
  GPIO_InitStruct.Pin = OCTOSPI_F_IO7_Pin|OCTOSPI_F_IO5_Pin|OCTOSPI_F_IO6_Pin|OCTOSPI_F_IO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PH3_BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PH3_BOOT0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = UCPD_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UCPD_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = CAM_XSDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAM_XSDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = CAM_RSTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAM_RSTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PFPin PFPin PFPin PFPin
                           PFPin PFPin */
  GPIO_InitStruct.Pin = OCTOSPI_F_IO0_Pin|OCTOSPI_F_IO1_Pin|OCTOSPI_F_IO2_Pin|OCTOSPI_F_IO3_Pin
                          |OCTOSPI_F_CLK_P_Pin|OCTOSPI_F_DQS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_OCTOSPI2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = USER_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PHPin PHPin PHPin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|Mems_VL53_xshut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = USB_C_P_Pin|USB_C_PA11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = MIC_CCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
  HAL_GPIO_Init(MIC_CCK1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin */
  GPIO_InitStruct.Pin = MIC_SDINx_Pin|MIC_CCK0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_ADF1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = WRLS_WKUP_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WRLS_WKUP_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PGPin PGPin */
  GPIO_InitStruct.Pin = Mems_VLX_GPIO_Pin|Mems_INT_LPS22HH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = MXCHIP_NOTIFY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MXCHIP_NOTIFY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin */
  GPIO_InitStruct.Pin = USB_UCPD_FLT_Pin|Mems_ISM330DLC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin */
  GPIO_InitStruct.Pin = Mems_INT_IIS2MDC_Pin|USB_IANA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = MXCHIP_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MXCHIP_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = USB_UCPD_CC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_UCPD_CC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Mems_STSAFE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Mems_STSAFE_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = MIC_SDIN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
  HAL_GPIO_Init(MIC_SDIN0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = MXCHIP_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MXCHIP_RESET_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI14_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI14_IRQn);

  HAL_NVIC_SetPriority(EXTI15_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_IRQn);

}

/* USER CODE BEGIN 2 */
void CAM_GPIO_Init(void){
	GPIO_InitTypeDef InitStruct = {0};
	__HAL_RCC_GPIOI_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CAM_XSDN_GPIO_Port, CAM_XSDN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CAM_RSTI_GPIO_Port, CAM_RSTI_Pin, GPIO_PIN_SET);

	InitStruct.Pin       = CAM_PLUG_Pin;
	InitStruct.Pull      = GPIO_PULLUP;
	InitStruct.Mode      = GPIO_MODE_INPUT;
	InitStruct.Alternate = GPIO_AF10_DCMI;
    HAL_GPIO_Init(CAM_PLUG_GPIO_Port, &InitStruct);

	//Init Reset Pin
	InitStruct.Pin = CAM_RSTI_Pin | CAM_XSDN_Pin;
	InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	InitStruct.Pull = GPIO_NOPULL;
	InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	InitStruct.Alternate = GPIO_AF10_DCMI;
	HAL_GPIO_Init(CAM_RSTI_GPIO_Port, &InitStruct);
}

void DCMI_GPIO_Init(void){
    GPIO_InitTypeDef InitStruct = {0};
     __HAL_RCC_DCMI_PSSI_CLK_ENABLE();

     __HAL_RCC_GPIOI_CLK_ENABLE();
     __HAL_RCC_GPIOE_CLK_ENABLE();
     __HAL_RCC_GPIOH_CLK_ENABLE();
     __HAL_RCC_GPIOB_CLK_ENABLE();
     __HAL_RCC_GPIOC_CLK_ENABLE();
     __HAL_RCC_GPIOA_CLK_ENABLE();

     InitStruct.Pin = CAM_D6_Pin|CAM_D5_Pin|CAM_D7_Pin;
     InitStruct.Mode = GPIO_MODE_AF_PP;
     InitStruct.Pull = GPIO_NOPULL;
     InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     InitStruct.Alternate = GPIO_AF10_DCMI;
     HAL_GPIO_Init(GPIOI, &InitStruct);

     InitStruct.Pin = CAM_D3_Pin;
     InitStruct.Mode = GPIO_MODE_AF_PP;
     InitStruct.Pull = GPIO_NOPULL;
     InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     InitStruct.Alternate = GPIO_AF10_DCMI;
     HAL_GPIO_Init(CAM_D3_GPIO_Port, &InitStruct);

     InitStruct.Pin = CAM_D4_Pin|CAM_HSYNC_Pin;
     InitStruct.Mode = GPIO_MODE_AF_PP;
     InitStruct.Pull = GPIO_NOPULL;
     InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     InitStruct.Alternate = GPIO_AF10_DCMI;
     HAL_GPIO_Init(GPIOH, &InitStruct);

     InitStruct.Pin = CAM_VSYNC_Pin;
     InitStruct.Mode = GPIO_MODE_AF_PP;
     InitStruct.Pull = GPIO_NOPULL;
     InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     InitStruct.Alternate = GPIO_AF10_DCMI;
     HAL_GPIO_Init(CAM_VSYNC_GPIO_Port, &InitStruct);

     InitStruct.Pin = CAM_D2_Pin|CAM_D1_Pin|CAM_D0_Pin;
     InitStruct.Mode = GPIO_MODE_AF_PP;
     InitStruct.Pull = GPIO_NOPULL;
     InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     InitStruct.Alternate = GPIO_AF10_DCMI;
     HAL_GPIO_Init(GPIOC, &InitStruct);

     InitStruct.Pin = CAM_PIXCLK_Pin;
     InitStruct.Mode = GPIO_MODE_AF_PP;
     InitStruct.Pull = GPIO_NOPULL;
     InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
     InitStruct.Alternate = GPIO_AF4_DCMI;
     HAL_GPIO_Init(CAM_PIXCLK_GPIO_Port, &InitStruct);

}

void Ram_GPIO_Init(void){
	GPIO_InitTypeDef InitStruct = {0};
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    InitStruct.Pin = OCTOSPI_R_IO5_Pin;
    InitStruct.Mode = GPIO_MODE_AF_PP;
    InitStruct.Pull = GPIO_NOPULL;
    InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO5_GPIO_Port, &InitStruct);

    InitStruct.Pin = OCTOSPI_R_DQS_Pin;
    InitStruct.Mode = GPIO_MODE_AF_PP;
    InitStruct.Pull = GPIO_NOPULL;
    InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_DQS_GPIO_Port, &InitStruct);

    InitStruct.Pin = OCTOSPI_R_IO7_Pin;
    InitStruct.Mode = GPIO_MODE_AF_PP;
    InitStruct.Pull = GPIO_NOPULL;
    InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO7_GPIO_Port, &InitStruct);

    InitStruct.Pin = OCTOSPI_R_IO0_Pin|OCTOSPI_R_IO2_Pin|OCTOSPI_R_IO1_Pin|OCTOSPI_R_IO3_Pin;
    InitStruct.Mode = GPIO_MODE_AF_PP;
    InitStruct.Pull = GPIO_NOPULL;
    InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOF, &InitStruct);

    InitStruct.Pin = OCTOSPI_R_IO4_Pin;
    InitStruct.Mode = GPIO_MODE_AF_PP;
    InitStruct.Pull = GPIO_NOPULL;
    InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO4_GPIO_Port, &InitStruct);

    InitStruct.Pin = OCTOSPI_R_IO6_Pin;
    InitStruct.Mode = GPIO_MODE_AF_PP;
    InitStruct.Pull = GPIO_NOPULL;
    InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(OCTOSPI_R_IO6_GPIO_Port, &InitStruct);

    InitStruct.Pin = OCTOSPI_R_CLK_P_Pin|OCTOSPI_R_NCS_Pin;
    InitStruct.Mode = GPIO_MODE_AF_PP;
    InitStruct.Pull = GPIO_NOPULL;
    InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOB, &InitStruct);
}

void MotionSensor_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = Mems_ISM330DLC_INT1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Mems_ISM330DLC_INT1_GPIO_Port, &GPIO_InitStruct);
}

void PressureSensor_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = Mems_INT_LPS22HH_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Mems_INT_LPS22HH_GPIO_Port, &GPIO_InitStruct);
}

void MagSensor_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = Mems_INT_IIS2MDC_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Mems_INT_IIS2MDC_GPIO_Port, &GPIO_InitStruct);
}
/* USER CODE END 2 */
