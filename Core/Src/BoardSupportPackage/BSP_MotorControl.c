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
#define DUTY_LIMIT     (40.0)

#define FRONT_RIGHT_MOTOR_CHANNEL (htim3.Instance->CCR3)
#define FRONT_LEFT_MOTOR_CHANNEL  (htim3.Instance->CCR4)
#define BACK_LEFT_MOTOR_CHANNEL   (htim3.Instance->CCR1)
#define BACK_RIGHT_MOTOR_CHANNEL  (htim3.Instance->CCR2)

#define RollKp (0.05f)
#define RollKi (0.04f)
#define RollKd (0.005f)

#define PitchKp (0.06f)
#define PitchKi (0.05f)
#define PitchKd (0.006f)

#define YawRateKp (0.005f)
#define YawRateKi (0.004f)
#define YawRateKd (0.0005f)

#define IntegralLimit (0.1)

#define HOVER_DUTY (7.5)
#define FRONT_RIGHT_CAL_FACTOR (.5)
#define FRONT_LEFT_CAL_FACTOR (.2)
#define BACK_LEFT_CAL_FACTOR (.45)
#define BACK_RIGHT_CAL_FACTOR (.15)


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

	  BSP_SetMotorFrontRight(10.0);
	  BSP_SetMotorFrontLeft(10.0);
	  BSP_SetMotorBackLeft(10.0);
	  BSP_SetMotorBackRight(10.0);

	  HAL_Delay(1000);

	  BSP_SetMotorFrontRight(0.0);
	  BSP_SetMotorFrontLeft(0.0);
	  BSP_SetMotorBackLeft(0.0);
	  BSP_SetMotorBackRight(0.0);

}

float *PID_Pitch_Mon;
float *PID_Roll_Mon;
float *PID_Yaw_Mon;

float *FR_Duty_Mon;
float *BR_Duty_Mon;
float *FL_Duty_Mon;
float *BL_Duty_Mon;

void BSP_FlightPID(DroneSetpoint_t Setpoint, DroneSetpoint_t CurrentState, float DeltaT)
{

	static float PID_Pitch;
	static float PID_Roll;
	static float PID_Yaw;
	static float FrontRightDuty;
	static float FrontLeftDuty;
	static float BackLeftDuty;
	static float BackRightDuty;

	PID_Pitch_Mon = &PID_Pitch;
	PID_Roll_Mon = &PID_Roll;
	PID_Yaw_Mon = &PID_Yaw;
	FR_Duty_Mon = &FrontRightDuty;
	BR_Duty_Mon = &BackRightDuty;
	FL_Duty_Mon = &FrontLeftDuty;
	BL_Duty_Mon = &BackLeftDuty;

	float dt = DeltaT / 1000.0f;
	PID_Pitch = PIDPitch(Setpoint.Pitch, CurrentState.Pitch, dt);
	PID_Roll  = PIDRoll(Setpoint.Roll, CurrentState.Roll, dt);
	PID_Yaw   = PIDYawRate(Setpoint.YawRate, CurrentState.YawRate, dt);

	FrontRightDuty = Setpoint.Thrust + PID_Pitch + PID_Roll - PID_Yaw;
	FrontLeftDuty  = Setpoint.Thrust + PID_Pitch - PID_Roll + PID_Yaw;
	BackLeftDuty   = Setpoint.Thrust - PID_Pitch - PID_Roll - PID_Yaw;
	BackRightDuty  = Setpoint.Thrust - PID_Pitch + PID_Roll + PID_Yaw;



	BSP_SetMotorFrontRight(FrontRightDuty);
	BSP_SetMotorFrontLeft(FrontLeftDuty);
	BSP_SetMotorBackLeft(BackLeftDuty);
	BSP_SetMotorBackRight(BackRightDuty);
}

static float PIDPitch(float PitchSetpoint, float PitchCurrent, float dt)
{
	static float PreviousError;
	static float ProportionalTerm;
	static float IntegralTerm;
	static float DerivativeTerm;
	float CurrentError =  PitchSetpoint - PitchCurrent;

	ProportionalTerm = PitchKp*CurrentError;
	IntegralTerm     = IntegralTerm + PitchKi*(((CurrentError + PreviousError)/2)*dt);
	DerivativeTerm   = PitchKd*((CurrentError - PreviousError)/dt);

	if(IntegralTerm >= IntegralLimit)
	{
		IntegralTerm = IntegralLimit;
	}else if(IntegralLimit <= -IntegralLimit)
	{
		IntegralTerm = -IntegralLimit;
	}

	PreviousError = CurrentError;
	return ProportionalTerm + IntegralTerm + DerivativeTerm;
}

static float PIDRoll(float RollSetpoint, float RollCurrent, float dt)
{
	static float PreviousError;
	static float ProportionalTerm;
	static float IntegralTerm;
	static float DerivativeTerm;
	float CurrentError =  RollSetpoint - RollCurrent;

	ProportionalTerm = RollKp*CurrentError;
	IntegralTerm     = IntegralTerm + RollKi*(((CurrentError + PreviousError)/2)*dt);
	DerivativeTerm   = RollKd*((CurrentError - PreviousError)/dt);

	if(IntegralTerm >= IntegralLimit)
	{
		IntegralTerm = IntegralLimit;
	}else if(IntegralLimit <= -IntegralLimit)
	{
		IntegralTerm = -IntegralLimit;
	}

	PreviousError = CurrentError;
	return ProportionalTerm + IntegralTerm + DerivativeTerm;
}

static float PIDYawRate(float YawRateSetpoint, float YawRateCurrent, float dt)
{
	static float PreviousError;
	static float ProportionalTerm;
	static float IntegralTerm;
	static float DerivativeTerm;
	float CurrentError =  YawRateSetpoint - YawRateCurrent;

	ProportionalTerm = YawRateKp*CurrentError;
	IntegralTerm     = IntegralTerm + YawRateKi*(((CurrentError + PreviousError)/2)*dt);
	DerivativeTerm   = YawRateKi*((CurrentError - PreviousError)/dt);

	if(IntegralTerm >= IntegralLimit)
	{
		IntegralTerm = IntegralLimit;
	}else if(IntegralLimit <= -IntegralLimit)
	{
		IntegralTerm = -IntegralLimit;
	}

	PreviousError = CurrentError;
	return ProportionalTerm + IntegralTerm + DerivativeTerm;
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

	Duty = Duty + FRONT_RIGHT_CAL_FACTOR;
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

	Duty = Duty + FRONT_LEFT_CAL_FACTOR;
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

	Duty = Duty + BACK_LEFT_CAL_FACTOR;
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

	Duty = Duty + BACK_RIGHT_CAL_FACTOR;
	BACK_RIGHT_MOTOR_CHANNEL = TICKS_PER_PERCENT * Duty + DUTY_0_TIMER;
}
