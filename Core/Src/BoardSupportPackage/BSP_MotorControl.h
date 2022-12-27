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
	float UpDown;
}DroneSetpoint_t;

void BSP_MotorInit(void);

void BSP_PIDFrontRight(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float dt);
void BSP_PIDFrontLeft(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float dt);
void BSP_PIDBackLeft(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float dt);
void BSP_PIDBackRight(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float dt);

#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_MOTORCONTROL_H_ */
