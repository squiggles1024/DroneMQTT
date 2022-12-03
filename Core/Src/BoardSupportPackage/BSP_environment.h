/*
 * BSP_environment.h
 *
 *  Created on: Sep 16, 2022
 *      Author: evanl
 */

#ifndef SRC_BOARDSUPPORTPACKAGE_BSP_ENVIRONMENT_H_
#define SRC_BOARDSUPPORTPACKAGE_BSP_ENVIRONMENT_H_
#include <stdint.h>
#include "LPS22HH.h"
#include "HTS221.h"
#include "IIS2MDC.h"
#include "VEML6030.h"

/* See BSP_Environment.c for full function descriptions */
/* Public function prototypes */
int32_t BSP_TempHumSensorInit(void);
int32_t BSP_ReadTemperature(float *result);
int32_t BSP_ReadHumidity(float *result);
int32_t BSP_EnableHeater(void);
int32_t BSP_DisableHeater(void);

int32_t BSP_PressureSensorInit(void);
int32_t BSP_ReadPressure(float *result);

int32_t BSP_MagnetometerInit(void);
int32_t BSP_ReadMagnetometerXYZ(float *Mx, float *My, float *Mz);
int32_t BSP_ReadMagnetometerX(float *Mx);
int32_t BSP_ReadMagnetometerY(float *My);
int32_t BSP_ReadMagnetometerZ(float *Mz);

int32_t BSP_AmbientLightInit(void);
int32_t BSP_ReadAmbientLight(float *Light);

int32_t BSP_GetTempPeriod(uint32_t *Period);
int32_t BSP_GetMagneticPeriod(uint32_t *Period);
int32_t BSP_GetPressurePeriod(uint32_t *Period);

#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_ENVIRONMENT_H_ */
