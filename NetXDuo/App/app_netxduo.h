/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_netxduo.h
  * @author  MCD Application Team
  * @brief   NetXDuo applicative header file
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
#ifndef __APP_NETXDUO_H__
#define __APP_NETXDUO_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "nx_api.h"

/* USER CODE BEGIN Includes */
#include "nxd_dhcp_client.h"
#include "nxd_sntp_client.h"
#include "nxd_mqtt_client.h"
#include "nxd_dns.h"
#include "nx_driver_emw3080.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
 extern TX_EVENT_FLAGS_GROUP MQTT_TREvent;
 extern TX_THREAD MQTTThread;
 extern NXD_MQTT_CLIENT MQTTClient;
 extern TX_MUTEX SPI_MUTEX;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define PAYLOAD_SIZE      (1514 + 28 + 2)
#define PACKET_POOL_SIZE (( PAYLOAD_SIZE + sizeof(NX_PACKET)) * 50)
#define NULL_ADDRESS                (0)
#define IP_HELPER_STACK_SIZE        (2048)
#define ARP_STACK_SIZE              (1024)
#define IP_INIT_STACK_SIZE          (4096)
#define DNS_ADDRESS                 IP_ADDRESS(192, 168, 1, 254)
#define SNTP_SERVER_URL             "time1.google.com"
#define SNTP_TIMEOUT                (10 * NX_IP_PERIODIC_RATE)
#define EPOCH_TIME_DIFF              2208988800     /* is equivalent to 70 years in sec calculated with www.epochconverter.com/date-difference */
#define MQTT_SERVER_IP              IP_ADDRESS(192, 168, 1, 119)
#define MQTT_KEEPALIVE_TIMER        (0)
#define CERT_FILE                   "ca.h"
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT MX_NetXDuo_Init(VOID *memory_ptr);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define MQTT_CLIENT_SUB_TOPIC01   "Drone/Controls"

#define MQTT_CLIENT_PUB_TOPIC01   "Env/Temperature"
#define MQTT_CLIENT_PUB_TOPIC02   "Env/Humidity"
#define MQTT_CLIENT_PUB_TOPIC03   "Env/Pressure"
#define MQTT_CLIENT_PUB_TOPIC04   "Env/Light"
#define MQTT_CLIENT_PUB_TOPIC05   "Env/Magnetic"
#define MQTT_CLIENT_PUB_TOPIC06   "Drone/Roll"
#define MQTT_CLIENT_PUB_TOPIC07   "Drone/Pitch"
#define MQTT_CLIENT_PUB_TOPIC08   "Drone/Yaw"
#define MQTT_CLIENT_PUB_TOPIC09   "Drone/RollRate"
#define MQTT_CLIENT_PUB_TOPIC10   "Drone/PitchRate"
#define MQTT_CLIENT_PUB_TOPIC11   "Env/Video"

#define MESSAGE_RECIEVED_EVT_Msk (1 << 0)
#define MESSAGE_TRANSMIT_PUB01_EVT_Msk (1 << 1)
#define MESSAGE_TRANSMIT_PUB02_EVT_Msk (1 << 2)
#define MESSAGE_TRANSMIT_PUB03_EVT_Msk (1 << 3)
#define MESSAGE_TRANSMIT_PUB04_EVT_Msk (1 << 4)
#define MESSAGE_TRANSMIT_PUB05_EVT_Msk (1 << 5)
#define MESSAGE_TRANSMIT_PUB06_EVT_Msk (1 << 8)
#define MESSAGE_TRANSMIT_PUB07_EVT_Msk (1 << 9)
#define MESSAGE_TRANSMIT_PUB08_EVT_Msk (1 << 10)
#define MESSAGE_TRANSMIT_PUB09_EVT_Msk (1 << 11)
#define MESSAGE_TRANSMIT_PUB10_EVT_Msk (1 << 12)
#define MESSAGE_TRANSMIT_PUB11_EVT_Msk (1 << 13)
#define MESSAGE_ALL_EVT_Msk (MESSAGE_RECIEVED_EVT_Msk | \
                             MESSAGE_TRANSMIT_PUB01_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB02_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB03_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB04_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB05_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB06_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB07_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB08_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB09_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB10_EVT_Msk | \
							 MESSAGE_TRANSMIT_PUB11_EVT_Msk)

#define PRESSURE_PAYLOAD_SAMPLES    (10) //10hz sampling frequency, 10 samples = 1 second of data
#define PRESSURE_MESSAGE_SIZE       (sizeof(float))
#define PRESSURE_QUEUE_SIZE         (PRESSURE_PAYLOAD_SAMPLES * PRESSURE_MESSAGE_SIZE)
#define MAGNETIC_PAYLOAD_SAMPLES    (10) //10hz sampling frequency, 10 samples = 1 second of data
#define MAGNETIC_MESSAGE_SIZE       (sizeof(float))
#define MAGNETIC_QUEUE_SIZE         (MAGNETIC_PAYLOAD_SAMPLES * MAGNETIC_MESSAGE_SIZE)
#define TEMPERATURE_PAYLOAD_SAMPLES (1)  //1Hz sampling frequency
#define TEMPERATURE_MESSAGE_SIZE    (sizeof(float))
#define TEMPERATURE_QUEUE_SIZE      (TEMPERATURE_PAYLOAD_SAMPLES * TEMPERATURE_MESSAGE_SIZE)
#define LIGHT_PAYLOAD_SAMPLES       (1)  //1Hz sampling frequency
#define LIGHT_MESSAGE_SIZE          (sizeof(float))
#define LIGHT_QUEUE_SIZE            (LIGHT_PAYLOAD_SAMPLES * LIGHT_MESSAGE_SIZE)
#define HUMIDITY_PAYLOAD_SAMPLES    (1)  //1Hz sampling frequency
#define HUMIDITY_MESSAGE_SIZE       (sizeof(float))
#define HUMIDITY_QUEUE_SIZE         (HUMIDITY_PAYLOAD_SAMPLES * HUMIDITY_MESSAGE_SIZE)
#define CAMERA_PAYLOAD_FRAMES       (1)
#define CAMERA_MESSAGE_SIZE         (sizeof(UINT*))
#define CAMERA_QUEUE_SIZE           (CAMERA_PAYLOAD_FRAMES * CAMERA_MESSAGE_SIZE)

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_NETXDUO_H__ */
