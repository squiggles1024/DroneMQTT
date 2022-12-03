/*
 * IIS2MDC.h
 *
 *  Created on: Sep 15, 2022
 *      Author: evanl
 */

#ifndef SRC_EXTERNALHARDWARE_IIS2MDC_H_
#define SRC_EXTERNALHARDWARE_IIS2MDC_H_
#include "IIS2MDC_Registers.h"
/* SEE ISM2MDC.c For Driver Details */

//Used for when the user wants to keep init settings as "factory default"
#define IIS2MDC_DefaultSetting (0U)

//Error/OK Status codes
#define IIS2MDC_Ok (0U)
#define IIS2MDC_HandleError (-1)
#define IIS2MDC_IOError (-2)
#define IIS2MDC_InitError (-3)
#define IIS2MDC_FloatError (-4)
#define IIS2MDC_OneShotError (-5)
#define IIS2MDC_Timeout (-6)
#define IIS2MDC_ODR_PeriodOneShot (1)
#define IIS2MDC_ODR_Error (2)

//Used to save ODR Value in IIS2MDC Device Handle.
typedef uint8_t IIS2MDC_ODR_t;

//Status codes when reading sensor data
typedef enum{
	IIS2MDC_DataReady = 0,
	IIS2MDC_DataNotReady = 0xBEEF
}IIS2MDC_DataReady_t;

//Used to save continuous or one-shot-mode value in IIS2MDC Device Handle.
typedef enum
{
	IIS2MDC_OneShotMode = 0,
	IIS2MDC_Continuous = 1
}IIS2MDC_DataMode_t;

//Config Reg A Settings be OR'd together in Init struct.
typedef enum
{
	IIS2MDC_ODR_10Hz = 0,
	IIS2MDC_ODR_OneShot = 1,
	IIS2MDC_ODR_20Hz = 1 << 2,
	IIS2MDC_ODR_50Hz = 2 << 2,
	IIS2MDC_ODR_100Hz = 3 << 2,

	IIS2MDC_LowPowerModeEnabled = 1 << 4,

	IIS2MDC_TempCompensationEnabled = 1 << 7
}IIS2MDC_ConfigRegA_t;

//Config Reg B Settings be OR'd together in Init struct.
typedef enum
{
	IIS2MDC_OffsetCancellationEnabled = 1 << 4,
	IIS2MDC_INT_on_DataOFF = 1 << 3,
	IIS2MDC_Set_FREQ = 1 << 2,
	IIS2MDC_OFF_CANC = 1 << 1,
	IIS2MDC_LPF = 1
}IIS2MDC_ConfigRegB_t;

//Config Reg C Settings be OR'd together in Init struct.
typedef enum
{
	IIS2MDC_INT_on_PIN = 1 << 6,
	IIS2MDC_I2C_DIS = 1 << 5,
	IIS2MDC_BDU = 1 << 4,
	IIS2MDC_DRDY_on_PIN = 1
}IIS2MDC_ConfigRegC_t;

//Int Control Settings be OR'd together in Init struct.
typedef enum
{
	IIS2MDC_XIEN = 1 << 7,
	IIS2MDC_YIEN = 1 << 6,
	IIS2MDC_ZIEN = 1 << 5,
	IIS2MDC_IEA = 1 << 2,
	IIS2MDC_IEL = 1 << 1,
	IIS2MDC_IEN = 1 << 0
}IIS2MDC_IntCtrlReg_t;

//Init struct to be initialized by user
typedef struct
{
	float OffsetX;
	float OffsetY;
	float OffsetZ;
	IIS2MDC_ConfigRegA_t ConfigRegA;
	IIS2MDC_ConfigRegB_t ConfigRegB;
	IIS2MDC_ConfigRegC_t ConfigRegC;
	IIS2MDC_IntCtrlReg_t IntCtrlReg;
	float IntThreshold;
}IIS2MDC_InitStruct_t;

//Saves whether the handle has been initialized or not.
typedef enum
{
	IIS2MDC_Initialized = 1,
	IIS2MDC_Uninitialized = 0
}IIS2MDC_InitStatus_t;

//IO struct to be provided by user
typedef struct
{
	void (*Init)(void);
	void (*Deinit)(void);
	int32_t (*Write)(uint8_t, uint8_t*, uint8_t);
	int32_t (*Read)(uint8_t, uint8_t*, uint8_t);
	int32_t (*ReadPin)(void);
	uint32_t (*GetTick)(void);
}IIS2MDC_IO_t;

//Device handle to be be provided by user.
typedef struct
{
	IIS2MDC_Context_t Context;
	IIS2MDC_IO_t IO;
	IIS2MDC_InitStatus_t Status;
	IIS2MDC_DataMode_t DataMode;
	IIS2MDC_ODR_t OutputDataRate;
}IIS2MDC_Handle_t;

int32_t IIS2MDC_Init(IIS2MDC_Handle_t *Handle, IIS2MDC_InitStruct_t Settings, IIS2MDC_IO_t *IO);
int32_t IIS2MDC_DeInit(IIS2MDC_Handle_t *Handle);
int32_t IIS2MDC_ResetDevice(IIS2MDC_Handle_t *Handle);

int32_t IIS2MDC_SetOffsetMagnetismX(IIS2MDC_Handle_t *Handle, float Offset);
int32_t IIS2MDC_GetOffsetMagnetismX(IIS2MDC_Handle_t *Handle, float *Offset);

int32_t IIS2MDC_SetOffsetMagnetismY(IIS2MDC_Handle_t *Handle, float Offset);
int32_t IIS2MDC_GetOffsetMagnetismY(IIS2MDC_Handle_t *Handle, float *Offset);

int32_t IIS2MDC_SetOffsetMagnetismZ(IIS2MDC_Handle_t *Handle, float Offset);
int32_t IIS2MDC_GetOffsetMagnetismZ(IIS2MDC_Handle_t *Handle, float *Offset);

int32_t IIS2MDC_StartConversion(IIS2MDC_Handle_t *Handle);
int32_t IIS2MDC_ReadMagnetismXYZ(IIS2MDC_Handle_t *Handle, float *Mx, float *My, float *Mz);
int32_t IIS2MDC_ReadMagnetismX(IIS2MDC_Handle_t *Handle, float *Mx);
int32_t IIS2MDC_ReadMagnetismY(IIS2MDC_Handle_t *Handle, float *My);
int32_t IIS2MDC_ReadMagnetismZ(IIS2MDC_Handle_t *Handle, float *Mz);

int32_t IIS2MDC_GetSamplePeriod(IIS2MDC_Handle_t *Handle, uint32_t *Period);

#endif /* SRC_EXTERNALHARDWARE_IIS2MDC_H_ */
