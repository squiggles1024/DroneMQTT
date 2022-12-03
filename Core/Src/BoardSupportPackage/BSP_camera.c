/*
 * camera.c
 *
 *  Created on: Sep 15, 2022
 *      Author: evanl
 */
/* Includes */
#include "BSP_camera.h"
#include "BSP_IOBus.h"
#include "../ExternalHardware/OV5640.h"
#include "gpio.h"
#include "gpdma.h"
#include "dcmi.h"

/* Private Defines */
#define CAMERA_OV5640_ADDRESS          0x78U         //I2C Address for OV5640
#define CAMERA_PF                      OV5640_RGB565 //Pixel format set to RGB565 to reduce bytes per pixel (reduced due to wifi bottleneck)

/* Private helper function prototypes */
static void BSP_CameraHwReset(void);
static void BSP_CameraHWInit(void);

/*****************************************************************************************
 *@Brief: Resets and initializes Camera module
 *@Param: None
 *@Return: CameraOK
 *@Post Condition: Camera will be initialized and ready to use
 *****************************************************************************************/
BSP_CameraStatus_t BSP_CameraInit(void)
{
	BSP_CameraHwReset();//Reset it
    BSP_CameraHWInit(); //Init OV5460 driver
	return CameraOK;
}

/*****************************************************************************************
 *@Brief: Starts camera in continous capture mode
 *@Param: Address of frame buffer.(Should be a uint32_t which is typecase into a pointer)
 *@Return: None
 *@Post Condition: Camera will begin capturing data and be stored in frame buffer via DMA
 *@Pre Condition: Camera Init function, DCMI, and GPIO pins should be configured first.
 *****************************************************************************************/
void BSP_CameraStart(uint8_t* Buffer){
	  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)Buffer, CAMERA_DATA_SIZE_WORDS);
}

/*****************************************************************************************
 *@Brief: Stops camera data capture
 *@Param: None
 *@Return: None
 *@Post Condition: Camera will stop capturing data
 *@Pre Condition: Camera should be started in continuous capture mode
 *****************************************************************************************/
void BSP_CameraStop(void){
	  HAL_DCMI_Stop(&hdcmi);
}

/*****************************************************************************************
 *@Brief: Starts camera in snapshot mode
 *@Param: Address of frame buffer.(Should be a uint32_t which is typecase into a pointer)
 *@Return: None
 *@Post Condition: Camera will start capturing a single frame and store in frame buffer via DMA
 *@Pre Condition: Camera Init function, DCMI, and GPIO pins should be configured first.
 *****************************************************************************************/
void BSP_CameraCaptureFrame(uint8_t* Buffer)
{
	HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)Buffer, CAMERA_DATA_SIZE_WORDS);
}

/*****************************************************************************************
 *@Brief: Performs hardware reset of OV5640 camera module
 *@Param: None
 *@Return: None
 *@Post Condition: Camera module will be reset to default POR settings
 *@Pre Condition: Camera GPIO Reset and Shutdown pins should be configured
 *****************************************************************************************/
static void BSP_CameraHwReset(void)
{
    /* Camera sensor RESET sequence */
    /* Assert the camera STANDBY and RSTI pins */

    HAL_GPIO_WritePin(CAM_XSDN_GPIO_Port, CAM_XSDN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CAM_RSTI_GPIO_Port, CAM_RSTI_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);   /* RST and XSDN signals asserted during 100ms */

    /* De-assert the camera STANDBY pin (active high) */
    HAL_GPIO_WritePin(CAM_XSDN_GPIO_Port, CAM_XSDN_Pin, GPIO_PIN_RESET);
    HAL_Delay(3);     /* RST de-asserted and XSDN de-asserted during 3ms */

    /* De-assert the camera RSTI pin (active low) */
    HAL_GPIO_WritePin(CAM_RSTI_GPIO_Port, CAM_RSTI_Pin, GPIO_PIN_SET);
    HAL_Delay(20);     /* RST de-asserted during 20ms */
}

/*****************************************************************************************
 *@Brief: Initializes and links low level IO driver to camera driver
 *@Param: None
 *@Return: None
 *@Post Condition: Camera module will be ready to use with 320x240 resolution and RGN565 pixel format
 *@Pre Condition: None
 *****************************************************************************************/
static void BSP_CameraHWInit(void){
	  uint32_t temp = 0;
	  OV5640_IO_t              IO;
	  static OV5640_Handle_t   OV5640_Handle;
	  IO.Address = CAMERA_OV5640_ADDRESS;
	  IO.Init = BSP_I2C1_Init;
	  IO.DeInit = BSP_I2C1_DeInit;
      IO.Read = BSP_I2C1_ReadReg16;
      IO.Write = BSP_I2C1_WriteReg16;
      IO.GetTick = BSP_GetTick;
      OV5640_LinkBus(&OV5640_Handle, &IO);
      OV5640_ReadID(&OV5640_Handle,&temp);
      OV5640_Init(&OV5640_Handle, OV5640_R320x240, CAMERA_PF);
}


