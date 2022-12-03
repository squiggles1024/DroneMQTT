/*
 * VEML6030.h
 *
 *  Created on: Oct 27, 2022
 *      Author: evanl
 */

/* See VEML6030.c for driver details*/

#ifndef SRC_EXTERNALHARDWARE_VEML6030_H_
#define SRC_EXTERNALHARDWARE_VEML6030_H_
#include <stdint.h>
#include "VEML6030_Registers.h"

//Status Codes
#define VEML6030_Ok (0U)
#define VEML6030_HandleError (-1)
#define VEML6030_PSM_Err  (-4)
#define VEML6030_IOError (-2)
#define VEML6030_InitError (-3)
#define VEML6030_ParamError (-5)

//Initialization States
#define VEML6030_STATUS_INITIALIZED (0x1)
#define VEML6030_STATUS_UNINITIALIZED (0x0)

//ALS GAIN SETTINGS
#define VEML6030_ALS_GAIN_1 (0x00 << 11)
#define VEML6030_ALS_GAIN_2 (0x01 << 11)
#define VEML6030_ALS_GAIN_DIV8 (0x02 << 11)
#define VEML6030_ALS_GAIN_DIV4 (0x03 << 11)

//ALS INTEGRATION TIME
#define VEML6030_INTTIME_025 (0x0C << 6)
#define VEML6030_INTTIME_050 (0x08 << 6)
#define VEML6030_INTTIME_100 (0x00 << 6)
#define VEML6030_INTTIME_200 (0x01 << 6)
#define VEML6030_INTTIME_400 (0x02 << 6)
#define VEML6030_INTTIME_800 (0x03 << 6)

//ALS PERSISTENCE PROTECTION
#define VEML6030_PERS_1 (0x0 << 4)
#define VEML6030_PERS_2 (0x1 << 4)
#define VEML6030_PERS_4 (0x2 << 4)
#define VEML6030_PERS_8 (0x3 << 4)

//ALS IRQ ENABLE
#define VEML6030_INT_EN (0x01 << 1)

#define VEML_POWERSAVING_ENABLE (1)
#define VEML_POWERSAVING_MODE1  (0 << 1)
#define VEML_POWERSAVING_MODE2  (1 << 1)
#define VEML_POWERSAVING_MODE3  (2 << 1)
#define VEML_POWERSAVING_MODE4  (3 << 1)

typedef uint8_t VEML6030_InitStatus_t;
typedef uint8_t VEML6030_PowerSaveMode_t;
typedef uint16_t VEML6030_IntegrationTime_t;
typedef uint16_t VEML6030_ALSConfig_t;

//Settings struct
typedef struct
{
	VEML6030_ALSConfig_t ALSConfig;            //"OR" Together desired settings
	VEML6030_PowerSaveMode_t PowerSavingMode;  //Power saving settings
}VEML_InitSettings_t;

//Low level IO Struct
typedef struct
{
	void (*Init)(void);
	void (*Deinit)(void);
	int32_t (*Write)(uint8_t,uint8_t*,uint8_t);
	int32_t (*Read)(uint8_t,uint8_t*,uint8_t);
	uint32_t (*GetTick)(void);
	int32_t (*ReadPin)(void);
}VEML6030_IO_t;

//Device Handle
typedef struct
{
	VEML6030_IO_t IO;
	VEML6030_Context_t Context;
	VEML6030_InitStatus_t Status;
    float Resolution;
}VEML6030_Handle_t;

int32_t VEML6030_Init(VEML6030_Handle_t *Handle, VEML_InitSettings_t Settings, const VEML6030_IO_t *IO);
int32_t VEML6030_ReadLight(VEML6030_Handle_t *Handle, float *Light);
int32_t VEML6030_SetALSHighThreshold(VEML6030_Handle_t *Handle, float Light);
int32_t VEML6030_SetALSLowThreshold(VEML6030_Handle_t *Handle, float Light);
int32_t VEML6030_GetALSHighThreshold(VEML6030_Handle_t *Handle, float *Light);
int32_t VEML6030_GetALSLowThreshold(VEML6030_Handle_t *Handle, float *Light);

#endif /* SRC_EXTERNALHARDWARE_VEML6030_H_ */
