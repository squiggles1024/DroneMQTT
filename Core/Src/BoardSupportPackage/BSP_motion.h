/*
 * BSP_motion.h
 *
 *  Created on: Sep 16, 2022
 *      Author: evanl
 */

#ifndef SRC_BOARDSUPPORTPACKAGE_BSP_MOTION_H_
#define SRC_BOARDSUPPORTPACKAGE_BSP_MOTION_H_
#include <stdint.h>
#include "ISM330DHCX.h"
/* See BSP_Motion.c for more details */

/* Public Function prototypes */
int32_t BSP_MotionSensorInit(void);
int32_t BSP_ReadAccelXYZ(float *x, float *y, float *z);
int32_t BSP_ReadGyroXYZ(float *x, float *y, float *z);
//int32_t BSP_ReadTemperature(float *temp);
int32_t BSP_GetAccelPeriod(uint32_t *Period);
int32_t BSP_GetGyroPeriod(uint32_t *Period);
void BSP_SynchronizeIRQ(void);

#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_MOTION_H_ */
