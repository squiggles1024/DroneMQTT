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

#define FRONT_RIGHT_MOTOR_CHANNEL (htim3.Instance->CCR3)
#define FRONT_LEFT_MOTOR_CHANNEL  (htim3.Instance->CCR4)
#define BACK_LEFT_MOTOR_CHANNEL   (htim3.Instance->CCR1)
#define BACK_RIGHT_MOTOR_CHANNEL  (htim3.Instance->CCR2)

#define PitchKp (0.0f)
#define PitchKi (0.0f)
#define PitchKd (0.0f)

#define RollKp (0.0f)
#define RollKi (0.0f)
#define RollKd (0.0f)

#define YawRateKp (0.0f)
#define YawRateKi (0.0f)
#define YawRateKd (0.0f)


#define HOVER_DUTY (40.0)

static void BSP_SetMotorFrontRight(float Duty);
static void BSP_SetMotorFrontLeft(float Duty);
static void BSP_SetMotorBackLeft(float Duty);
static void BSP_SetMotorBackRight(float Duty);

static float PIDPitch(float PitchSetpoint, float PitchCurrent, float dt);
static float PIDRoll(float RollSetpoint, float RollCurrent, float dt);
static float PIDYawRate(float YawRateSetpoint, float YawRateCurrent, float dt);

void BSP_MotorInit(void)
{
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void BSP_FlightPID(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float DeltaT)
{
	float dt = DeltaT / 1000.0f;
	float PID_Pitch = PIDPitch(Setpoint.Pitch, CurrentState.Pitch, dt);
	float PID_Roll  = PIDRoll(Setpoint.Roll, CurrentState.Roll, dt);
	float PID_Yaw   = PIDYawRate(Setpoint.YawRate, CurrentState.YawRate, dt);

	float FrontRightDuty = Setpoint.Thrust + PID_Pitch + PID_Roll - PID_Yaw;
	float FrontLeftDuty  = Setpoint.Thrust + PID_Pitch - PID_Roll + PID_Yaw;
	float BackLeftDuty   = Setpoint.Thrust - PID_Pitch - PID_Roll - PID_Yaw;
	float BackRightDuty  = Setpoint.Thrust - PID_Pitch + PID_Roll + PID_Yaw;

	BSP_SetMotorFrontRight(FrontRightDuty);
	BSP_SetMotorFrontLeft(FrontLeftDuty);
	BSP_SetMotorBackLeft(BackLeftDuty);
	BSP_SetMotorBackRight(BackRightDuty);
}

static float PIDPitch(float PitchSetpoint, float PitchCurrent, float dt)
{
	static float ErrorAccumulator, PreviousError;
	float CurrentError = PitchCurrent - PitchSetpoint;

	float result =  PitchKp*CurrentError + PitchKi*(ErrorAccumulator + ((CurrentError + PreviousError)/2)*dt) + PitchKd*((CurrentError - PreviousError)/dt);
	ErrorAccumulator += CurrentError;
	PreviousError = CurrentError;
	return result;
}

static float PIDRoll(float RollSetpoint, float RollCurrent, float dt)
{
	static float ErrorAccumulator, PreviousError;
	float CurrentError = RollCurrent - RollSetpoint;

	float result =  RollKp*CurrentError + RollKi*(ErrorAccumulator + ((CurrentError + PreviousError)/2)*dt) + RollKd*((CurrentError - PreviousError)/dt);
	ErrorAccumulator += CurrentError;
	PreviousError = CurrentError;
	return result;
}

static float PIDYawRate(float YawRateSetpoint, float YawRateCurrent, float dt)
{
	static float ErrorAccumulator, PreviousError;
	float CurrentError = YawRateCurrent - YawRateSetpoint;

	float result =  YawRateKp*CurrentError + YawRateKi*(ErrorAccumulator + ((CurrentError + PreviousError)/2)*dt) + YawRateKd*((CurrentError - PreviousError)/dt);
	ErrorAccumulator += CurrentError;
	PreviousError = CurrentError;
	return result;
}



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
