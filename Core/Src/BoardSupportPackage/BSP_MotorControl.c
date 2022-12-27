/*
 * BSP_MotorControl.c
 *
 *  Created on: Dec 26, 2022
 *      Author: evanl
 */
#include "tim.h"
#include "BSP_MotorControl.h"

#define DUTY_100_TIMER (13450)           // 13439 = 82us.... +11 is from experimentation with logic analyzer
#define DUTY_0_TIMER   (6730)            // 6719 = 42us..... +11 is from experimentation with logic analyzer
#define TICKS_PER_PERCENT ((DUTY_100_TIMER - DUTY_0_TIMER) / 100.0f)
#define DUTY_LIMIT     (80.0)

#define FRONT_RIGHT_MOTOR_CHANNEL (htim3.Instance->CCR1)
#define FRONT_LEFT_MOTOR_CHANNEL  (htim3.Instance->CCR2)
#define BACK_LEFT_MOTOR_CHANNEL   (htim3.Instance->CCR3)
#define BACK_RIGHT_MOTOR_CHANNEL  (htim3.Instance->CCR4)

#define FrKp (0.0f)
#define FrKi (0.0f)
#define FrKd (0.0f)

#define FlKp (0.0f)
#define FlKi (0.0f)
#define FlKd (0.0f)

#define BlKp (0.0f)
#define BlKi (0.0f)
#define BlKd (0.0f)

#define BrKp (0.0f)
#define BrKi (0.0f)
#define BrKd (0.0f)

static void BSP_SetMotorFrontRight(float Duty);
static void BSP_SetMotorFrontLeft(float Duty);
static void BSP_SetMotorBackLeft(float Duty);
static void BSP_SetMotorBackRight(float Duty);

void BSP_MotorInit(void)
{
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void BSP_PIDFrontRight(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float dt);
void BSP_PIDFrontLeft(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float dt);
void BSP_PIDBackLeft(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float dt);
void BSP_PIDBackRight(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float dt);

static void BSP_SetMotorFrontRight(float Duty)
{
	if(Duty < 0)
	{
		Duty = 0;
	}

	if(Duty > DUTY_LIMIT)
	{
		Duty = DUTY_LIMIT;
	}

	FRONT_RIGHT_MOTOR_CHANNEL = TICKS_PER_PERCENT * Duty + DUTY_0_TIMER;
}

static void BSP_SetMotorFrontLeft(float Duty)
{
	if(Duty < 0)
	{
		Duty = 0;
	}

	if(Duty > DUTY_LIMIT)
	{
		Duty = DUTY_LIMIT;
	}

	FRONT_LEFT_MOTOR_CHANNEL = TICKS_PER_PERCENT * Duty + DUTY_0_TIMER;
}

static void BSP_SetMotorBackLeft(float Duty)
{
	if(Duty < 0)
	{
		Duty = 0;
	}

	if(Duty > DUTY_LIMIT)
	{
		Duty = DUTY_LIMIT;
	}

	BACK_LEFT_MOTOR_CHANNEL = TICKS_PER_PERCENT * Duty + DUTY_0_TIMER;
}

static void BSP_SetMotorBackRight(float Duty)
{
	if(Duty < 0)
	{
		Duty = 0;
	}

	if(Duty > DUTY_LIMIT)
	{
		Duty = DUTY_LIMIT;
	}

	BACK_RIGHT_MOTOR_CHANNEL = TICKS_PER_PERCENT * Duty + DUTY_0_TIMER;
}
