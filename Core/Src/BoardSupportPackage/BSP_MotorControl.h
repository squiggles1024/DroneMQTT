/*
 * BSP_MotorControl.h
 *
 *  Created on: Dec 26, 2022
 *      Author: evanl
 */

#ifndef SRC_BOARDSUPPORTPACKAGE_BSP_MOTORCONTROL_H_
#define SRC_BOARDSUPPORTPACKAGE_BSP_MOTORCONTROL_H_
#include <stdint.h>

typedef struct
{
	float Roll;
	float Pitch;
	float YawRate;
	float Thrust;
}__attribute__((packed))DroneSetpoint_t;

void BSP_MotorInit(void);
void BSP_FlightPID(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float DeltaT);

#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_MOTORCONTROL_H_ */
