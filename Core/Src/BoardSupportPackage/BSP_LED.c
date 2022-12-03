/*
 * BSP_LED.c
 *
 *  Created on: Oct 27, 2022
 *      Author: evanl
 */

#include "BSP_LED.h"
#include "stm32u5xx_hal.h"
#include "main.h"

/******************************************************
 * @Brief: Toggles board Red LED
 * @Params: None
 * @Return: None
 * @Post Condition: Red LED will be inverted
 * @Pre Condition: GPIO Pin should be configured
 ******************************************************/
void BSP_LEDToggleRed(void)
{
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
}

/******************************************************
 * @Brief: Toggles board Green LED
 * @Params: None
 * @Return: None
 * @Post Condition: Green LED will be inverted
 * @Pre Condition: GPIO Pin should be configured
 ******************************************************/
void BSP_LEDToggleGreen(void)
{
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}

/******************************************************
 * @Brief: Turns board Red LED Off
 * @Params: None
 * @Return: None
 * @Post Condition: Red LED will be turned off
 * @Pre Condition: GPIO Pin should be configured
 ******************************************************/
void BSP_LEDTurnOffRed(void)
{
   HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}

/******************************************************
 * @Brief: Turns board Red LED On
 * @Params: None
 * @Return: None
 * @Post Condition: Red LED will be turned on
 * @Pre Condition: GPIO Pin should be configured
 ******************************************************/
void BSP_LEDTurnOnRed(void)
{
	   HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
}

/******************************************************
 * @Brief: Turns board Green LED off
 * @Params: None
 * @Return: None
 * @Post Condition: Green LED will be turned off
 * @Pre Condition: GPIO Pin should be configured
 ******************************************************/
void BSP_LEDTurnOffGreen(void)
{
	   HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
}

/******************************************************
 * @Brief: Turns board Green LED on
 * @Params: None
 * @Return: None
 * @Post Condition: Green LED will be turned on
 * @Pre Condition: GPIO Pin should be configured
 ******************************************************/
void BSP_LEDTurnOnGreen(void)
{
	   HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
}
