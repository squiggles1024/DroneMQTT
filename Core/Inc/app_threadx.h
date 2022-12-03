/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.h
  * @author  MCD Application Team
  * @brief   ThreadX applicative header file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_THREADX_H
#define __APP_THREADX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
 extern TX_QUEUE TemperatureQueue;             //Queue used for storing Temperature sample for transport over MQTT
 extern TX_QUEUE LightQueue;                   //Queue used for storing Ambient Light sample for transport over MQTT
 extern TX_QUEUE PressureQueue;                //Queue used for storing 10 Pressure samples for transport over MQTT
 extern TX_QUEUE HumidityQueue;                //Queue used for storing humidity sample for transport over MQTT
 extern TX_QUEUE MagXQueue;                    //Queue used for storing 10 B-Field X samples for transport over MQTT
 extern TX_QUEUE MagYQueue;                    //Queue used for storing 10 B-Field Y samples for transport over MQTT
 extern TX_QUEUE MagZQueue;                    //Queue used for storing 10 B-Field Z samples for transport over MQTT
 extern TX_QUEUE CameraQueue;                  //Queue used for storing pointers to camera frame buffer so video data can be transmitted over MQTT
 extern TX_SEMAPHORE CameraSendFrame[2];       //Semaphore used to signal that a camera frame has been transmitted, and thus the frame in that frame buffer can be overwritten
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT App_ThreadX_Init(VOID *memory_ptr);
void MX_ThreadX_Init(void);

/* USER CODE BEGIN EFP */
void TIM7_ResumeMotionThread(void); //Called from Timer7 ISR, resumes thread that reads Accel/Gyro (data that is most critical for drone flight)
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_THREADX_H__ */
