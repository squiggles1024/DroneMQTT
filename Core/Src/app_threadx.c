/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_azure_rtos.h"
#include "main.h"
#include "tim.h"
#include "dcache.h"
#include "BSP_ram.h"
#include "BSP_camera.h"
#include "BSP_environment.h"
#include "BSP_motion.h"
#include "BSP_LED.h"
#include "BSP_TrigFunctions.h"
#include "app_netxduo.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	float Roll;
	float Pitch;
	float RollRate;
	float PitchRate;
	float YawRate;
}AHRS_Handle_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AHRS_INIT_TIMESLICE (5000)          //Initialize Roll/Pitch with 5 seconds worth of raw data

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

//Debug Variables - Live Expressions
//float temp_data;
//float humidity_data;
//float pressure_data;
float accelx, accely, accelz;
float gyrox, gyroy, gyroz;
//float magx, magy, magz;
//float light;
//uint32_t Frame0Send = 0;
//uint32_t Frame1Send = 0;
//#define TRACEX_BUFFER_SIZE (64000)
//uint8_t tracex_buffer[TRACEX_BUFFER_SIZE] __attribute__ ((section (".trace")));


/****** TX Thread Pointers *******/
TX_THREAD LED_Red_Toggle;                  //Thread Pointer to "Red LED Toggle" Thread
TX_THREAD LED_Green_Toggle;                //Thread Pointer to "Green LED Toggle" Thread
TX_THREAD Read_TemperatureThreadPtr;       //Thread Pointer to "Read Temperature" Thread
TX_THREAD Read_HumidityThreadPtr;          //Thread Pointer to "Read Humidity" Thread
TX_THREAD Read_PressureThreadPtr;          //Thread Pointer to "Read Pressure" Thread
TX_THREAD Read_MotionThreadPtr;            //Thread Pointer to "Read Motion Sensor" Thread
TX_THREAD Read_MagneticThreadPtr;          //Thread Pointer to "Read Magnetometer" Thread
TX_THREAD Read_LightThreadPtr;             //Thread Pointer to "Read Ambient Light" Thread
TX_THREAD CaptureFrameThreadPtr;           //Thread Pointer to "Take Photo" Thread

/** TX Flow Control Variables **/
TX_MUTEX MutexI2C2;                        //Mutex used to protect I2C2 Bus which is shared by several sensors
TX_SEMAPHORE CameraCaptureFrame[2];        //Semaphore taken when camera is capturing a photo, and put back when it has been captured
TX_SEMAPHORE CameraSendFrame[2];           //Semaphore taken when photo data is ready to be transmitted, and put back when the photo has been transmitted

/******* TX Queues *******/
TX_QUEUE TemperatureQueue;                 //Queue used to transport Temperature data to MQTT Thread
TX_QUEUE LightQueue;                       //Queue used to transport Light data to MQTT Thread
TX_QUEUE PressureQueue;                    //Queue used to transport Pressure data to MQTT Thread
TX_QUEUE HumidityQueue;                    //Queue used to transport Humidity data to MQTT Thread
TX_QUEUE MagXQueue;                        //Queue used to transport Magnetic Field X Direction data to MQTT Thread
TX_QUEUE MagYQueue;                        //Queue used to transport Magnetic Field Y Direction data to MQTT Thread
TX_QUEUE MagZQueue;                        //Queue used to transport Magnetic Field Z Direction data to MQTT Thread
TX_QUEUE CameraQueue;                      //Queue used to store starting addresses of photo buffers to be sent to MQTT Thread

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//See function definitions for full descriptions
VOID RedLEDToggleThread(ULONG init);
VOID GreenLEDToggleThread(ULONG init);
VOID ReadTemperatureThread(ULONG init);
VOID ReadHumidityThread(ULONG init);
VOID ReadPressureThread(ULONG init);
VOID ReadMagneticThread(ULONG init);
VOID ReadMotionThread(ULONG init);
VOID ReadLightThread(ULONG init);
VOID CaptureFrameThread(ULONG init);
VOID SendFrameThread(ULONG init);
VOID InitializeAHRS(AHRS_Handle_t *Attitude);
inline static VOID ConvertVectorOrientation(float *Xvector, float *Yvector);
static VOID DataSendNotify(TX_QUEUE *QueuePtr);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

   /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  (void)byte_pool;
  UCHAR *Ptr = byte_pool->tx_byte_pool_start;
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */



  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, LED_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS) //Allocate stack space for Red LED Thread
  {
	  return TX_POOL_ERROR;
  }
  //Create Red LED Thread
  ret = tx_thread_create(&LED_Red_Toggle,                 //Thread Ptr
		            "Red LED",                            //Thread Name
					RedLEDToggleThread,                   //Thread Fn Ptr (address)
					0,                                    //Initial input
					Ptr,                                  //Stack Ptr
					LED_STACK_SIZE,                       //Stack Size
					LED_THREAD_PRIORITY,                  //Priority
					LED_THREAD_PRIORITY,                  //Preempt Threshold
					1,                                    //Time Slice
					TX_AUTO_START);                       //Auto Start or Auto Activate
  ret = tx_byte_allocate(byte_pool, (VOID **) &Ptr, LED_STACK_SIZE, TX_NO_WAIT);          //Allocate stack space for Green LED Thread

  if(ret != TX_SUCCESS)
  {
	  return TX_POOL_ERROR;
  }
  //Create Green LED Thread
  ret = tx_thread_create(&LED_Green_Toggle,           //Thread Ptr
		            "Green LED",                      //Thread Name
					GreenLEDToggleThread,             //Thread Fn Ptr (address)
					0,                                //Initial input
					Ptr,                              //Stack Ptr
					LED_STACK_SIZE,                   //Stack Size
					LED_THREAD_PRIORITY,              //Priority
					LED_THREAD_PRIORITY,              //Preempt Threshold
					1,                                //Time Slice
					TX_AUTO_START);                   //Auto Start or Auto Activate


  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, MOT_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS) //Allocate stack space for Accelerometer thread
  {
	  return TX_POOL_ERROR;
  }
  //Create red LED thread
  ret = tx_thread_create(&Read_MotionThreadPtr,      //Thread Ptr
		            "Motion Sensor",                 //Thread Name
					ReadMotionThread,                //Thread Fn Ptr (address)
					0,                               //Initial input
					Ptr,                             //Stack Ptr
					MOT_THREAD_STACK_SIZE,           //Stack Size
					GET_MOTION_PRIORITY,             //Priority
					GET_MOTION_PRIORITY,             //Preempt Threshold
					TX_NO_TIME_SLICE,                //Time Slice
					TX_AUTO_START);                  //Auto Start or Auto Activate

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, TEMP_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS) //Allocate stack space for temperature thread
  {
	  return TX_POOL_ERROR;
  }

  //Create temperature thread
  ret = tx_thread_create(&Read_TemperatureThreadPtr,   //Thread Ptr
		            "Temp Sensor",                     //Thread Name
					ReadTemperatureThread,             //Thread Fn Ptr (address)
					0,                                 //Initial input
					Ptr,                               //Stack Ptr
					TEMP_THREAD_STACK_SIZE,            //Stack Size
					GET_TEMPERATURE_PRIORITY,          //Priority
					GET_TEMPERATURE_PRIORITY,          //Preempt Threshold
				    TX_NO_TIME_SLICE,                  //Time Slice
					TX_AUTO_START);                    //Auto Start or Auto Activate

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, HUM_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS) //Allocate stack space for humidity thread
  {
	  return TX_POOL_ERROR;
  }

  //Create humidity thread
  ret = tx_thread_create(&Read_HumidityThreadPtr,   //Thread Ptr
		            "Humidity Sensor",              //Thread Name
					ReadHumidityThread,             //Thread Fn Ptr (address)
					0,                              //Initial input
					Ptr,                            //Stack Ptr
					HUM_THREAD_STACK_SIZE,          //Stack Size
					GET_HUMIDITY_PRIORITY,          //Priority
					GET_HUMIDITY_PRIORITY,          //Preempt Threshold
					TX_NO_TIME_SLICE,               //Time Slice
					TX_AUTO_START);                 //Auto Start or Auto Activate

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, PRES_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)   //Allocate stack space for pressure thread
  {
	  return TX_POOL_ERROR;
  }

  //Create pressure thread
  ret = tx_thread_create(&Read_PressureThreadPtr,   //Thread Ptr
		            "Pressure Sensor",              //Thread Name
					ReadPressureThread,             //Thread Fn Ptr (address)
					0,                              //Initial input
					Ptr,                            //Stack Ptr
					PRES_THREAD_STACK_SIZE,         //Stack Size
					GET_PRESSURE_PRIORITY,          //Priority
					GET_PRESSURE_PRIORITY,          //Preempt Threshold
					TX_NO_TIME_SLICE,               //Time Slice
					TX_AUTO_START);                 //Auto Start or Auto Activate


  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, MAG_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)    //Allocate stack space for magnetometer thread
  {
	  return TX_POOL_ERROR;
  }

  //Create magnetometer thread
  ret = tx_thread_create(&Read_MagneticThreadPtr,   //Thread Ptr
		            "Magnetic Sensor",              //Thread Name
					ReadMagneticThread,             //Thread Fn Ptr (address)
					0,                              //Initial input
					Ptr,                            //Stack Ptr
					MAG_THREAD_STACK_SIZE,          //Stack Size
					GET_MAGNETIC_PRIORITY,          //Priority
					GET_MAGNETIC_PRIORITY,          //Preempt Threshold
					TX_NO_TIME_SLICE,               //Time Slice
					TX_AUTO_START);                 //Auto Start or Auto Activate

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, LIGHT_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)    //Allocate stack space for Light thread
  {
	  return TX_POOL_ERROR;
  }

  //Create light thread
  ret = tx_thread_create(&Read_LightThreadPtr,   //Thread Ptr
		            "Light Sensor",              //Thread Name
					ReadLightThread,             //Thread Fn Ptr (address)
					0,                           //Initial input
					Ptr,                         //Stack Ptr
					LIGHT_THREAD_STACK_SIZE,     //Stack Size
					GET_LIGHT_PRIORITY,          //Priority
					GET_LIGHT_PRIORITY,          //Preempt Threshold
					TX_NO_TIME_SLICE,            //Time Slice
					TX_AUTO_START);              //Auto Start or Auto Activate


  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, CAPFRAME_THREAD_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS) //Allocate space for camera thread stack
  {
	  return TX_POOL_ERROR;
  }

  //Create camera thread
  ret = tx_thread_create(&CaptureFrameThreadPtr,   //Thread Ptr
		            "Capture Frame Thread",        //Thread Name
					CaptureFrameThread,            //Thread Fn Ptr (address)
					0,                             //Initial input
					Ptr,                           //Stack Ptr
					CAPFRAME_THREAD_STACK_SIZE,    //Stack Size
					CAPTURE_FRAME_PRIORITY,        //Priority
					CAPTURE_FRAME_PRIORITY,        //Preempt Threshold
					2,                             //Time Slice
					TX_AUTO_START);                //Auto Start or Auto Activate


  ret = tx_mutex_create(&MutexI2C2, "I2C2 Mutex", TX_INHERIT);                               //Create I2C2 Mutex
  ret = tx_semaphore_create(&CameraCaptureFrame[0], "Camera Buffer 1 Capture Semaphore", 0); //Create Camera Buffer 1 Capture Semaphore
  ret = tx_semaphore_create(&CameraCaptureFrame[1], "Camera Buffer 2 Capture Semaphore", 0); //Create Camera Buffer 2 Capture Semaphore
  ret = tx_semaphore_create(&CameraSendFrame[0], "Camera Buffer 1 Send Semaphore", 0);       //Create Camera Buffer 1 Transmit Semaphore
  ret = tx_semaphore_create(&CameraSendFrame[1], "Camera Buffer 2 Send Semaphore", 0);       //Create Camera Buffer 2 Transmit Semaphore

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, ENV_DATA_QUEUE_SIZE, TX_NO_WAIT) != TX_SUCCESS)          //Allocate space for temperature Queue
  {
	  return TX_POOL_ERROR;
  }
  ret = tx_queue_create(&TemperatureQueue, "Temperature Queue", TX_1_ULONG, Ptr, TEMPERATURE_QUEUE_SIZE); //Create temperature queue

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, ENV_DATA_QUEUE_SIZE, TX_NO_WAIT) != TX_SUCCESS)   //Allocate space for pressure Queue
  {
	  return TX_POOL_ERROR;
  }
  ret = tx_queue_create(&PressureQueue, "Pressure Queue", TX_1_ULONG, Ptr, PRESSURE_QUEUE_SIZE);   //Create pressure queue

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, ENV_DATA_QUEUE_SIZE, TX_NO_WAIT) != TX_SUCCESS)   //Allocate space for humidity Queue
  {
	  return TX_POOL_ERROR;
  }
  ret = tx_queue_create(&HumidityQueue, "Humidity Queue", TX_1_ULONG, Ptr, HUMIDITY_QUEUE_SIZE);   //Create humidity queue

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, ENV_DATA_QUEUE_SIZE, TX_NO_WAIT) != TX_SUCCESS)   //Allocate space for light Queue
  {
	  return TX_POOL_ERROR;
  }
  ret = tx_queue_create(&LightQueue, "Light Queue", TX_1_ULONG, Ptr, LIGHT_QUEUE_SIZE);           //Create Light queue

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, ENV_DATA_QUEUE_SIZE, TX_NO_WAIT) != TX_SUCCESS)  //Allocate space for mag X Queue
  {
	  return TX_POOL_ERROR;
  }
  ret = tx_queue_create(&MagXQueue, "MagX Queue", TX_1_ULONG, Ptr, MAGNETIC_QUEUE_SIZE);          //Create Mag X queue

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, ENV_DATA_QUEUE_SIZE, TX_NO_WAIT) != TX_SUCCESS)  //Allocate space for mag Y Queue
  {
	  return TX_POOL_ERROR;
  }
  ret = tx_queue_create(&MagYQueue, "MagY Queue", TX_1_ULONG, Ptr, MAGNETIC_QUEUE_SIZE);          //Create Mag Y queue

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, ENV_DATA_QUEUE_SIZE, TX_NO_WAIT) != TX_SUCCESS)  //Allocate space for mag Z Queue
  {
	  return TX_POOL_ERROR;
  }
  ret = tx_queue_create(&MagZQueue, "MagZ Queue", TX_1_ULONG, Ptr, MAGNETIC_QUEUE_SIZE);          //Create Mag Z queue

  if(tx_byte_allocate(byte_pool, (VOID **) &Ptr, ENV_DATA_QUEUE_SIZE, TX_NO_WAIT) != TX_SUCCESS)  //Allocate space for Camera Queue
  {
	  return TX_POOL_ERROR;
  }
  ret = tx_queue_create(&CameraQueue, "Camera Queue", TX_1_ULONG, Ptr, CAMERA_QUEUE_SIZE);        //Create Camera queue

  //Register DataSendNotify function as the queue notify callback
  ret = tx_queue_send_notify(&TemperatureQueue,DataSendNotify);
  ret = tx_queue_send_notify(&PressureQueue,DataSendNotify);
  ret = tx_queue_send_notify(&HumidityQueue,DataSendNotify);
  ret = tx_queue_send_notify(&LightQueue,DataSendNotify);
  ret = tx_queue_send_notify(&MagZQueue,DataSendNotify);
  ret = tx_queue_send_notify(&CameraQueue,DataSendNotify);
  //tx_trace_enable(&tracex_buffer, TRACEX_BUFFER_SIZE,30);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/***************************************************************************************************
 * @Brief: Function reads data from temperature sensor and puts it in queue to be transmitted.
 *         If the queue is full, it will remove the data and replace it with the more recent sample
 *         Thread puts itself to sleep for an amount of time dependent on the sampling period.
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
VOID ReadTemperatureThread(ULONG init)
{
	int32_t ret = HTS221_Ok;
	uint32_t SleepTime = 0;
	float Temperature = 0;
	float DummyData;
	while(1)
	{
		tx_mutex_get(&MutexI2C2,TX_WAIT_FOREVER); 		//Take I2C Mutex (if available)
		ret = BSP_ReadTemperature(&Temperature);       	//Fetch a new temperature sample
		tx_mutex_put(&MutexI2C2);                       //Put mutex back
		if(ret == HTS221_DataReady)
		{
			if(TemperatureQueue.tx_queue_available_storage == 0)                          //Remove old data from queue if it is full
			{
				tx_queue_receive(&TemperatureQueue, &DummyData, TX_NO_WAIT);
			}
            ret = tx_queue_send(&TemperatureQueue, (VOID*)&Temperature, TX_WAIT_FOREVER); //Put new data into the queue
		}
		ret = BSP_GetTempPeriod(&SleepTime);                                              //Retrieve the sample period in ms
		if(ret == HTS221_Ok)
		{

			if(SleepTime % 10 == 0)                    //Sleep Time given in milliseconds. OS Timeslice = 10 ms
			{
				tx_thread_sleep(SleepTime / 10);       //Sleep time in ms, but OS time slice is 10ms, so divide Sample period in ms by 10ms
			} else
			{
				tx_thread_sleep((SleepTime / 10) + 1); //Sleep 1 additional time slice if the period isn't evenly divisible by 10.
			}
		}
	}
}

/***************************************************************************************************
 * @Brief: Function reads data from humidity sensor and puts it in queue to be transmitted.
 *         If the queue is full, it will remove the data and replace it with the more recent sample
 *         Thread puts itself to sleep for an amount of time dependent on the sampling period.
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
VOID ReadHumidityThread(ULONG init)
{
	int32_t ret = HTS221_Ok;
	uint32_t SleepTime = 0;
	float Humidity = 0;
	float DummyData;
	while(1)
	{
		tx_mutex_get(&MutexI2C2,TX_WAIT_FOREVER);            //Take I2C Mutex (if available)
		ret = BSP_ReadHumidity(&Humidity);                   //Fetch a new data sample
		tx_mutex_put(&MutexI2C2);                            //Put mutex back
		if(ret == HTS221_DataReady)
		{
			if(HumidityQueue.tx_queue_available_storage == 0)                        //Remove old data from queue if it's full
			{
				tx_queue_receive(&HumidityQueue, &DummyData, TX_NO_WAIT);
			}
            ret = tx_queue_send(&HumidityQueue, (VOID*)&Humidity, TX_WAIT_FOREVER);  //Put new data in queue

		}
		ret = BSP_GetTempPeriod(&SleepTime);                                         //Fetch sample period (given in ms)
		if(ret == HTS221_Ok)
		{
			//Sleep Time given in milliseconds. OS Timeslice = 10 ms, +1 is to handle possible non-zero remainders.
			if(SleepTime % 10 == 0)
			{
				tx_thread_sleep(SleepTime / 10);                                    //Put thread to sleep. OS Timeslice = 10ms. SleepTime / 10ms is sleep time in OS TimeSlices
			} else
			{
				tx_thread_sleep((SleepTime / 10) + 1);                              //Put thread to sleep. OS Timeslice = 10ms. SleepTime / 10ms is sleep time in OS TimeSlices
			}                                                                       //The +1 is to account for a non-zero remainder in "SleepTime" / 10ms
		}
	}
}

/***************************************************************************************************
 * @Brief: Function reads data from pressure sensor and puts it in queue to be transmitted.
 *         If the queue is full, it will remove the data and replace it with the more recent sample
 *         Thread puts itself to sleep for an amount of time dependent on the sampling period.
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
VOID ReadPressureThread(ULONG init)
{
	int32_t ret = LPS22HH_Ok;
	uint32_t SleepTime = 0;
	float Pressure = 0;
	float DummyData;
	while(1)
	{
		tx_mutex_get(&MutexI2C2,TX_WAIT_FOREVER);                                //Take I2C Mutex
		ret = BSP_ReadPressure(&Pressure);                                       //Fetch pressure data
		tx_mutex_put(&MutexI2C2);                                                //Put mutex back
		if(ret == LPS22HH_DataReady)
		{
			if(PressureQueue.tx_queue_available_storage == 0)
			{
				tx_queue_receive(&PressureQueue, &DummyData, TX_NO_WAIT);           //Remove old data from queue if its full
			}
			ret = tx_queue_send(&PressureQueue, (VOID*)&Pressure, TX_WAIT_FOREVER); //Put new data in queue

		}
		ret = BSP_GetPressurePeriod(&SleepTime);                                    //Get Sample period in ms
		if(ret == LPS22HH_Ok)
		{
			//Sleep Time given in milliseconds. OS Timeslice = 10 ms, +1 is to handle possible non-zero remainders.
			if(SleepTime % 10 == 0)
			{
				tx_thread_sleep(SleepTime / 10); //If period (in ms) was evenly divisible by OS time slices (10 ms), sleep for: Period (in ms) / OS Time Slice (10ms)
			} else
			{
				tx_thread_sleep((SleepTime / 10) + 1); //If period (in ms) was NOT evenly divisible by OS time slices (10 ms), sleep for: Period (in ms) / OS Time Slice (10ms) + 1
			}
		}
	}
}

/***************************************************************************************************
 * @Brief: Function reads data from magnetometer and puts it in queue to be transmitted.
 *         If the queue is full, it will remove the data and replace it with the more recent sample
 *         Thread puts itself to sleep for an amount of time dependent on the sampling period.
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
VOID ReadMagneticThread(ULONG init)
{
	int32_t ret = IIS2MDC_Ok;
	uint32_t SleepTime = 0;
	float MagX = 0;
	float MagY = 0;
	float MagZ = 0;
	float DummyData;
	while(1)
	{
		tx_mutex_get(&MutexI2C2,TX_WAIT_FOREVER); //Take I2C Mutex
		ret = BSP_ReadMagnetometerXYZ(&MagX, &MagY, &MagZ); //Read magnetometer
		tx_mutex_put(&MutexI2C2);                 //Put mutex back
		if(ret == IIS2MDC_DataReady)
		{
            if(MagZQueue.tx_queue_available_storage == 0)
            {
            	tx_queue_receive(&MagXQueue, &DummyData, TX_NO_WAIT); //Remove old data if the queue is full
            	tx_queue_receive(&MagYQueue, &DummyData, TX_NO_WAIT);
            	tx_queue_receive(&MagZQueue, &DummyData, TX_NO_WAIT);
            }
			ret = tx_queue_send(&MagXQueue, (VOID*)&MagX, TX_WAIT_FOREVER); //Put new data into the queue
			ret = tx_queue_send(&MagYQueue, (VOID*)&MagY, TX_WAIT_FOREVER);
			ret = tx_queue_send(&MagZQueue, (VOID*)&MagZ, TX_WAIT_FOREVER);
		}
		ret = BSP_GetMagneticPeriod(&SleepTime); //Retrieve sample period in ms
		if(ret == IIS2MDC_Ok)
		{
			//Sleep Time given in milliseconds. OS Timeslice = 10 ms, +1 is to handle possible non-zero remainders.
			if(SleepTime % 10 == 0)
			{
				tx_thread_sleep(SleepTime / 10); //Sleep for Period in ms / OS Timeslice period (10ms)
			} else
			{
				tx_thread_sleep((SleepTime / 10) + 1); //Sleep for period in ms / OS Timeslice + 1 if period is not evenly divisible by OS Timeslice.
			}
		}
	}
}

/***************************************************************************************************
 * @Brief: Initializes Drone Heading by taking 5 seconds worth of data from the accelerometer and
 *         gyroscope and taking the average... and doing some trig.
 * @Param: Attitude handle to initialize.
 * @Return: None
 ***************************************************************************************************/
VOID InitializeAHRS(AHRS_Handle_t *Attitude)
{
	float AvgAccX = 0, AvgAccY = 0, AvgAccZ = 0;
	float AvgGyroX = 0, AvgGyroY = 0, AvgGyroZ = 0;
	float DummyDataBuffer[3];
	uint32_t TimeStart = HAL_GetTick();                                                                                  //Initialize a timer
	uint16_t AccSamples = 0, GyroSamples = 0;
	while(HAL_GetTick() < (TimeStart + AHRS_INIT_TIMESLICE))
	{
		tx_mutex_get(&MutexI2C2,TX_NO_WAIT);
		if(BSP_ReadAccelXYZ(&DummyDataBuffer[0], &DummyDataBuffer[1], &DummyDataBuffer[2]) == ISM330DHCX_DataReady)      //Gather Data from Accel/Gyro for 5 seconds
		{
			AvgAccX +=  DummyDataBuffer[0];
			AvgAccY +=  DummyDataBuffer[1];
			AvgAccZ +=  DummyDataBuffer[2];
			AccSamples++;
		}

		if(BSP_ReadGyroXYZ(&DummyDataBuffer[0], &DummyDataBuffer[1], &DummyDataBuffer[2]) == ISM330DHCX_DataReady)
		{
			AvgGyroX +=  DummyDataBuffer[0];
			AvgGyroY +=  DummyDataBuffer[1];
			AvgGyroZ +=  DummyDataBuffer[2];
			GyroSamples++;
		}
		tx_mutex_put(&MutexI2C2);
	}
	ConvertVectorOrientation(&AvgAccX, &AvgAccY);            //Convert Vector to standard drone orientation. (X = Y, Y = -X)
	ConvertVectorOrientation(&AvgGyroX, &AvgGyroY);
	AvgAccX = AvgAccX / AccSamples;                          //Calculate the Average acceleration vector and rotation vectors
	AvgAccY = AvgAccY / AccSamples;
	AvgAccZ = AvgAccZ / AccSamples;
	AvgGyroX = AvgGyroX / GyroSamples;
	AvgGyroY = AvgGyroY / GyroSamples;
	AvgGyroZ = AvgGyroZ / GyroSamples;

	Attitude->Roll = RAD_2_DEG(BSP_atan(AvgAccY/AvgAccZ)); //Calculate roll
	Attitude->Pitch = RAD_2_DEG(asin(AvgAccX));            //Calculuate pitch
	Attitude->RollRate = AvgGyroY;                         //Rates are measured directly and do not require additional calculation.
	Attitude->PitchRate = AvgGyroX;
	Attitude->YawRate = AvgGyroZ;
}

/***************************************************************************************************
 * @Brief: Function reads data from accelerometer/gyro data and calculates roll, pitch, roll rate,
 *         pitch rate, and yaw rate using an extended Kalman Filter.
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
VOID ReadMotionThread(ULONG init)
{
	int32_t ret = ISM330DHCX_Ok;
	float AccelX = 0, AccelY = 0, AccelZ = 0;
	float GyroX = 0, GyroY = 0, GyroZ = 0;
	static AHRS_Handle_t AHRS;
	uint32_t AccelInitTime, GyroInitTime;

	InitializeAHRS(&AHRS);
	AccelInitTime = HAL_GetTick();
	GyroInitTime = AccelInitTime;
	BSP_SynchronizeIRQ(); //Synchronize with hardware timer with accelerometer sample frequency
	while(1)
	{
	    tx_mutex_get(&MutexI2C2,TX_NO_WAIT);               //Take I2C Mutex
	    ret = BSP_ReadAccelXYZ(&AccelX, &AccelY, &AccelZ); //Read Accelerometer
	    if(ret == ISM330DHCX_DataReady)
	    {
		    accelx = AccelX;                               //Store Data
		    accely = AccelY;
		    accelz = AccelZ;
	    }
	    ret = BSP_ReadGyroXYZ(&GyroX, &GyroY, &GyroZ);    //Read Gyroscope
	    if(ret == ISM330DHCX_DataReady)
	    {
		    gyrox = GyroX;                                //Store Data
		    gyroy = GyroY;
		    gyroz = GyroZ;
	    }
	    tx_mutex_put(&MutexI2C2);                         //Put I2C Mutex back
	    tx_thread_suspend(&Read_MotionThreadPtr);         //Suspend thread
	}
}

/***************************************************************************************************
 * @Brief: Function is called from Timer7 ISR in stm32u5xx_it.c
 *         All it does is resume the Read Motion thread.
 *         Timer7 ISR is set to trigger at appx the same period of the XL/Gyro sample period
 *         Accelerometer and Gyro data is the most critical data for Drone flight
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
void TIM7_ResumeMotionThread(void)
{
	if(Read_MotionThreadPtr.tx_thread_state == TX_SUSPENDED)
	{
	    tx_thread_resume(&Read_MotionThreadPtr); //Resume Accelerometer/Gyro Thread
	}
}

/***************************************************************************************************
 * @Brief: Function reads data from light sensor and puts it in queue to be transmitted.
 *         If the queue is full, it will remove the data and replace it with the more recent sample
 *         Thread puts itself to sleep for 1 second.
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
VOID ReadLightThread(ULONG init)
{
	float Light = 0;
	int32_t ret = 0;
	float DummyData;
	while(1)
	{
	    tx_mutex_get(&MutexI2C2,TX_WAIT_FOREVER);
		ret = BSP_ReadAmbientLight(&Light);
	    tx_mutex_put(&MutexI2C2);
		if(ret == VEML6030_Ok)
		{
			if(LightQueue.tx_queue_available_storage == 0)
			{
				tx_queue_receive(&LightQueue, &DummyData, TX_NO_WAIT);
			}
			ret = tx_queue_send(&LightQueue, &Light, TX_WAIT_FOREVER);

		}
		tx_thread_sleep(100);
	}
}

/***************************************************************************************************
 * @Brief: Function toggles Red LED and puts itself to sleep for 1 second.
 *         This is primarily for debug purposes. The LED will stop blinking if a fault occurs.
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
VOID RedLEDToggleThread(ULONG init)
{
	while(1)
	{
		BSP_LEDToggleRed();
		tx_thread_sleep(100);
	}
}

/***************************************************************************************************
 * @Brief: Function toggles Green LED and puts itself to sleep for 1 second.
 *         This is primarily for debug purposes. The LED will stop blinking if a fault occurs.
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
VOID GreenLEDToggleThread(ULONG init)
{
	while(1)
	{
		BSP_LEDToggleGreen();
		tx_thread_sleep(100);
	}
}

/***************************************************************************************************
 * @Brief: Function implements a 'ping pong buffer' for camera data capture
 *         First it loads a photo into the first buffer and waits for it to finish.
 *         Then it begins capturing a 2nd photo to put in the second buffer (via DMA)
 *         The Address of the first buffer is loaded into the CameraQueue
 *         Putting this address into the camera queue alerts the MQTT Thread that a photo is available
 *         for transmit.
 *
 *         The general pattern is: Start Capturing a Photo, and Let the MQTT Thread know to transmit the
 *         previously captured photo. The buffer of the photo being captured and the photo being
 *         transmitted alternates back and forth.
 * @Param: None
 * @Return: None
 ***************************************************************************************************/
VOID CaptureFrameThread(ULONG init)
{
	ULONG BufferAddressToSend = 0;
	BSP_CameraCaptureFrame((uint8_t*)CAMERA_FRAMEBUFFER1_ADDR); //Begin capturing frame1
	tx_semaphore_get(&CameraCaptureFrame[0],TX_WAIT_FOREVER);   //Wait for frame to complete
    while(1)
    {
        BSP_CameraCaptureFrame((uint8_t*)CAMERA_FRAMEBUFFER2_ADDR);       //Begin capturing frame 2
        BufferAddressToSend = CAMERA_FRAMEBUFFER1_ADDR;
        tx_queue_send(&CameraQueue,&BufferAddressToSend,TX_WAIT_FOREVER); //Put frame 1 into queue
        tx_semaphore_get(&CameraSendFrame[0],TX_WAIT_FOREVER);            //wait for Frame 1 to send
        tx_semaphore_get(&CameraCaptureFrame[1],TX_WAIT_FOREVER);         //wait for Frame 2 to finish capturing

        BSP_CameraCaptureFrame((uint8_t*)CAMERA_FRAMEBUFFER1_ADDR);       //Capture frame 1
        BufferAddressToSend = CAMERA_FRAMEBUFFER2_ADDR;
        tx_queue_send(&CameraQueue,&BufferAddressToSend,TX_WAIT_FOREVER); //Put frame 2 into queue
        tx_semaphore_get(&CameraSendFrame[1],TX_WAIT_FOREVER);            //wait for frame 2 to send
        tx_semaphore_get(&CameraCaptureFrame[0],TX_WAIT_FOREVER);         //wait for frame 1 to finish capturing
    }
}


/***************************************************************************************************
 * @Brief: Function is called from the DCMI/PSSI ISR when a frame is captured.
 *         When a GPDMA transfer is complete, the processor needs to know to invalidate it's DCACHE
 *         in the region where the DMA transfer was performed.
 *         Thus, when a frame is captured, the DCACHE at the frame buffer address is invalidated.
 * @Return: None
 ***************************************************************************************************/
static uint32_t FrameCount = 0; //Keeps track of the number of captured frames.
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{

	if(FrameCount % 2 == 0)
	{
		HAL_DCACHE_CleanInvalidByAddr_IT(&hdcache1, (const uint32_t *const)CAMERA_FRAMEBUFFER1_ADDR, CAMERA_DATA_SIZE_BYTES); //Clear DCACHE in Frame1 Buffer Region
		hdcmi->DMA_Handle->State = HAL_DMA_STATE_READY; //Fix STM32 HAL Bug. DCMI GPDMA Handle will not reset to ready upon finishing frame capture in Snapshot mode.

	} else
	{
		HAL_DCACHE_CleanInvalidByAddr_IT(&hdcache1, (const uint32_t *const)CAMERA_FRAMEBUFFER2_ADDR, CAMERA_DATA_SIZE_BYTES); //Clear DCACHE in Frame2 Buffer Region
		hdcmi->DMA_Handle->State = HAL_DMA_STATE_READY; //Fix STM32 HAL Bug. DCMI GPDMA Handle will not reset to ready upon finishing frame capture in Snapshot mode.
	}

}


/***************************************************************************************************
 * @Brief: Function is called from the DCACHE ISR which will be triggered by the DCMI Frame Event ISR
 *         After the frame is captured and the DCACHE is cleaned, the "Capture Frame" semaphore is
 *         put back. This allows the CameraFrameCapture thread to capture the next frame and
 *         transmit the previously transmitted frame.
 * @Return: None
 ***************************************************************************************************/
void HAL_DCACHE_CleanAndInvalidateByAddrCallback(DCACHE_HandleTypeDef *hdcache)
{
	if((FrameCount % 2)== 0)
	{
		tx_semaphore_put(&CameraCaptureFrame[0]); //Signal that frame 1 is finished capturing
	} else
	{
		tx_semaphore_put(&CameraCaptureFrame[1]); //Signal that frame 2 is finished capturing
	}
	FrameCount++; //Increment frame count (used in DCMI frame event callback above)
}

/***************************************************************************************************
 * @Brief: Convert orientation of measured vectors from Gyro/Accel sensors to "standard" flight
 *         orientation. i.e.: X axis = nose of plane, Y axis = wing of plane, Z axis = points down
 * @Param: X and Y measurements from accelerometer or gyroscope.
 * @Return: None
 ***************************************************************************************************/
inline static VOID ConvertVectorOrientation(float *Xvector, float *Yvector)
{
	float temp = *Xvector;
	*Xvector = *Yvector;
	*Yvector = temp * (-1);
}

/***************************************************************************************************
 * @Brief: Function is called when data is put into a queue. If the queue is full, the associated
 *         MQTT Event will be triggered and the MQTT Client thread will be enabled to transmit
 *         the data.
 * @Param: Queue that data was put into.
 * @Return: None
 ***************************************************************************************************/
static VOID DataSendNotify(TX_QUEUE *QueuePtr)
{
	if(QueuePtr->tx_queue_available_storage != 0) //return if the queue isn't full
	{
		return;
	}

	if(QueuePtr == &TemperatureQueue)
	{
        tx_event_flags_set(&MQTT_TREvent,MESSAGE_TRANSMIT_PUB01_EVT_Msk, TX_OR); //Signal Temperature data ready to send
	}
	else if(QueuePtr == &LightQueue)
	{
		tx_event_flags_set(&MQTT_TREvent,MESSAGE_TRANSMIT_PUB04_EVT_Msk, TX_OR); //Signal Light data ready to send
	}
	else if(QueuePtr == &PressureQueue)
	{
		tx_event_flags_set(&MQTT_TREvent,MESSAGE_TRANSMIT_PUB03_EVT_Msk, TX_OR); //Signal Pressure data ready to send
	}
	else if(QueuePtr == &HumidityQueue)
	{
		tx_event_flags_set(&MQTT_TREvent,MESSAGE_TRANSMIT_PUB02_EVT_Msk, TX_OR); //Signal Humidity data ready to send
	}
	else if(QueuePtr == &MagZQueue)
	{
		tx_event_flags_set(&MQTT_TREvent,MESSAGE_TRANSMIT_PUB05_EVT_Msk, TX_OR); //Signal Mag data ready to send
	}else if(QueuePtr == &CameraQueue)
	{
		tx_event_flags_set(&MQTT_TREvent,MESSAGE_TRANSMIT_PUB11_EVT_Msk,TX_OR);  //Signal Camera data ready to send
	}

	if(MQTTClient.nxd_mqtt_client_state == NXD_MQTT_CLIENT_STATE_CONNECTED)      //Do not resume the MQTT Thread until the Client has finished initializing.
	{
        tx_thread_resume(&MQTTThread);
	}

}

/* USER CODE END 1 */
