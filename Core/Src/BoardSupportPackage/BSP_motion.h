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

#define KALMAN_PREDICT (0)
#define KALMAN_UPDATE (1)

typedef struct
{
	float Roll;
	float Pitch;
	float RollRate;
	float PitchRate;
	float YawRate;
	float P[4][4]; //5 = Top Row, Column 3, 6 =
}AHRS_Handle_t;

/* Public Function prototypes */
int32_t BSP_MotionSensorInit(void);
int32_t BSP_ReadAccelXYZ(float *x, float *y, float *z);
int32_t BSP_ReadGyroXYZ(float *x, float *y, float *z);
//int32_t BSP_ReadTemperature(float *temp);
int32_t BSP_GetAccelPeriod(uint32_t *Period);
int32_t BSP_GetGyroPeriod(uint32_t *Period);
void BSP_SynchronizeIRQ(void);
void BSP_InitializeAHRS(AHRS_Handle_t *Attitude);
void BSP_KalmanGyroPredict(AHRS_Handle_t *Attitude, float NewGyroX, float NewGyroY, float NewGyroZ, uint32_t TimeDelta);
void BSP_KalmanAccelUpdate(AHRS_Handle_t *Attitude, float AccelX, float AccelY, float AccelZ);

#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_MOTION_H_ */
