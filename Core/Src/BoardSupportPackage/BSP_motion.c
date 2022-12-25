/*
 * BSP_motion.c
 *
 *  Created on: Sep 16, 2022
 *      Author: evanl
 */
/* Includes */
#include "../ExternalHardware/ISM330DHCX.h"
#include "BSP_IOBus.h"
#include "BSP_motion.h"
#include <stddef.h>
#include "tim.h"
#include "BSP_TrigFunctions.h"
#include "math.h"

/* Private Variables (Sensor Handle) */
static ISM330DHCX_Handle_t MotionSensor;
static const float CalibOffsetAccelX = 0.0016;
static const float CalibOffsetAccelY = 0.0030;
static const float CalibOffsetAccelZ = 0.0082;
static const float CalibOffsetGyroX = -0.02;
static const float CalibOffsetGyroY = -0.62;
static const float CalibOffsetGyroZ = -0.20;
inline static void ConvertVectorOrientation(float *Xvector, float *Yvector, float *Zvector);

#define AHRS_INIT_TIMESLICE (5000)          //Initialize Roll/Pitch with 5 seconds worth of raw data
//#define KAL_P_INIT    (1.0f)
//#define KAL_Q_INIT    (0.000001f)
//#define KAL_R_INIT   (-0.000001f)
#define ACCEL_STD_DEV (0.5) //Accel measures angles with std dev of 0.5 degrees
#define GYRO_STD_DEV  (5.0) //Gyro measures angles with std dev of 3.0 degrees
#define ROLL_RATE_INIT  (0.0)
#define PITCH_RATE_INIT (0.0)
#define YAW_RATE_INIT   (0.0)
#define ROLL_INIT_STD   (1.0)
#define PITCH_INIT_STD  (1.0)

/********************************************************************************************************************************************************************
 * @Brief: Initializes motion sensor hardware and handle
 * @Param: None
 * @Return: HAL Status or ISM330DHCX Status depending on where/if an error occurred.
 * @Post Condition: Motion sensor handle will be initialized and ready to use
 * @Pre Condition: None
 ********************************************************************************************************************************************************************/
int32_t BSP_MotionSensorInit(void)
{
	int32_t ret = 0;
	float JunkData[3];
	uint8_t DiscardedXLSamples = 0;
	uint8_t DiscardedGyroSamples = 0;
	ISM330DHCX_IO_t SensorIO =
	{
			.Init = BSP_ISM330DHCX_IO_Init,
			.DeInit = BSP_I2C2_DeInit,
			.Read = BSP_I2C2_ReadRegISM330DHCX,
			.Write = BSP_I2C2_WriteRegISM330DHCX,
			.GetTick = BSP_GetTick,
			.ReadInt1Pin = BSP_ReadPinISM330DHCX,
			.ReadInt2Pin = NULL
	};

	ISM330DHCX_Init_Struct_t Init =
	{
			.SDO_PU_EN = ISM330DHCX_SDOPullupDisconnected,
			.SIM = ISM330DHCX_DefaultSetting,
			.IF_INC = ISM330DHCX_AutoIncrementEnabled,

			.TriggerMode = ISM330DHCX_DataEnableOff,
			.DEN_X = ISM330DHCX_DefaultSetting,
			.DEN_Y = ISM330DHCX_DefaultSetting,
			.DEN_Z = ISM330DHCX_DefaultSetting,
			.DEN_XL_G = ISM330DHCX_DefaultSetting,
			.DEN_XL_EN = ISM330DHCX_DefaultSetting,
			.DEN_LH = ISM330DHCX_DefaultSetting,

			.FIFO_MODE = ISM330DHCX_FifoBypassed,
			.WTM = ISM330DHCX_DefaultSetting,
			.STOP_ON_WTM = ISM330DHCX_DefaultSetting,
			.BDR_GY = ISM330DHCX_DefaultSetting,
			.BDR_XL = ISM330DHCX_DefaultSetting,
			.ODR_T_BATCH = ISM330DHCX_DefaultSetting,
			.DEC_TS_BATCH = ISM330DHCX_DefaultSetting,
			.dataready_pulsed = ISM330DHCX_DefaultSetting,
			.TRIG_COUNTER_BDR = ISM330DHCX_DefaultSetting,
			.CNT_BDR_TH = ISM330DHCX_DefaultSetting,

			.ODR_XL = ISM330DHCX_XL_52Hz,
			.XL_HM_MODE = ISM330DHCX_XL_HPModeEnabled,
			.HP_SLOPE_XL_EN = ISM330DHCX_XL_LowPass,
			.LPF2_XL_EN =ISM330DHCX_XL_SSF_Enabled,
			.HPCF_XL = ISM330DHCX_ODRDiv20,
			.USR_OFF_ON_OUT = ISM330DHCX_XL_UserOffsetOutputDisabled,
			.USR_OFF_W = ISM330DHCX_DefaultSetting,
			.X_OFS_USR = ISM330DHCX_DefaultSetting,
			.Y_OFS_USR = ISM330DHCX_DefaultSetting,
			.Z_OFS_USR = ISM330DHCX_DefaultSetting,
			.HP_REF_MODE_XL = ISM330DHCX_XL_HighPassRefDisabled,
			.FASTSETTL_MODE_XL = ISM330DHCX_DefaultSetting,

			.FS_XL = ISM330DHCX_XL_4g,

			.ODR_G = ISM330DHCX_G_52Hz,
			.HPM_G = ISM330DHCX_DefaultSetting,
			.HP_EN_G = ISM330DHCX_G_HP_Disabled,
			.FTYPE = ISM330DHCX_DefaultSetting,
			.LPF1_SEL_G = ISM330DHCX_LPF_Disabled,
			.G_HM_MODE = ISM330DHCX_G_HPModeEnabled,

			.FS_G = ISM330DHCX_G_2000DPS,

			.DRDY_MASK = ISM330DHCX_DefaultSetting,

			.H_LACTIVE = ISM330DHCX_IRQ_ActiveHigh,
			.PP_OD = ISM330DHCX_IRQ_PushPull,
			.INT1_CTRL = ISM330DHCX_INT1_DRDY_G | ISM330DHCX_INT1_DRDY_XL,
			.INT2_CTRL = ISM330DHCX_DefaultSetting,
			.INT2_on_INT1 = ISM330DHCX_DefaultSetting,

			.TIMESTAMP_EN = ISM330DHCX_DefaultSetting

	};

	ret = ISM330DHCX_Init(&MotionSensor, Init, &SensorIO);
	if(ret !=ISM330DHCX_Ok)
	{
		return ret;
	}

	//Discard Junk Samples. See App Note for details (first few samples are garbage,depending on sample frequency)
	while(DiscardedXLSamples < 6 && DiscardedGyroSamples < 6)
	{
        if(BSP_ReadAccelXYZ(&JunkData[0],&JunkData[1],&JunkData[2]) == ISM330DHCX_DataReady)
        {
        	DiscardedXLSamples++;
        }

        if(BSP_ReadGyroXYZ(&JunkData[0],&JunkData[1],&JunkData[2]) == ISM330DHCX_DataReady)
        {
        	DiscardedGyroSamples++;
        }
	}
	return ISM330DHCX_Ok;
}

/********************************************************************************************************************************************************************
 * @Brief: Reads Accelerometer data
 * @Param: pointers to floats to store data in
 * @Return: HAL Status or ISM330DHCX Status depending on where/if an error occurred, DataReady if new data is returned, DataNotReady if new data is not available yet.
 * @Post Condition: Ax, Ay, Az will have data if new data is available.
 * @Pre Condition: device should be initialized with BSP_MotionSensorInit
 ********************************************************************************************************************************************************************/
int32_t BSP_ReadAccelXYZ(float *Ax, float *Ay, float *Az)
{
    int32_t ret = 0;
    ret = ISM330DHCX_ReadAccel(&MotionSensor, Ax, Ay, Az);
    if(ret != ISM330DHCX_DataReady)
    {
    	return ret;
    }
    *Ax = *Ax - CalibOffsetAccelX;
    *Ay = *Ay - CalibOffsetAccelY;
    *Az = *Az - CalibOffsetAccelZ;
    ConvertVectorOrientation(Ax, Ay, Az);

    return ISM330DHCX_DataReady;

}

/********************************************************************************************************************************************************************
 * @Brief: Reads Gyroscope data
 * @Param: pointers to floats to store data in
 * @Return: HAL Status or ISM330DHCX Status depending on where/if an error occurred, DataReady if new data is returned, DataNotReady if new data is not available yet.
 * @Post Condition: Ax, Ay, Az will have data if new data is available.
 * @Pre Condition: device should be initialized with BSP_MotionSensorInit
 ********************************************************************************************************************************************************************/
int32_t BSP_ReadGyroXYZ(float *Wx, float *Wy, float *Wz)
{
    int32_t ret = 0;
    ret = ISM330DHCX_ReadGyro(&MotionSensor, Wx, Wy, Wz);
    if(ret != ISM330DHCX_DataReady)
    {
    	return ret;
    }
    *Wx = *Wx - CalibOffsetGyroX;
    *Wy = *Wy - CalibOffsetGyroY;
    *Wz = *Wz - CalibOffsetGyroZ;
    ConvertVectorOrientation(Wx, Wy, Wz);


    return ISM330DHCX_DataReady;

}

/*
int32_t BSP_ReadTemperatureMotion(float *temp)
{
	return ISM330DHCX_ReadTemp(&MotionSensor, temp);
}
*/

/********************************************************************************************************************************************************************
 * @Brief: Retrieves accelerometer period (in ms) based on sample frequency. Useful for putting threads to sleep for an amount of time.
 * @Param: pointers to uint32 to store period
 * @Return: HAL Status or ISM330DHCX Status depending on where/if an error occurred
 * @Post Condition: sample period will be stored in "Period" param
 * @Pre Condition: device should be initialized with BSP_MotionSensorInit
 ********************************************************************************************************************************************************************/
int32_t BSP_GetAccelPeriod(uint32_t *Period)
{
	int32_t ret = ISM330DHCX_GetAccelPeriod(&MotionSensor, Period);
	return ret;
}

/********************************************************************************************************************************************************************
 * @Brief: Retrieves gyroscope period (in ms) based on sample frequency. Useful for putting threads to sleep for an amount of time.
 * @Param: pointers to uint32 to store period
 * @Return: HAL Status or ISM330DHCX Status depending on where/if an error occurred
 * @Post Condition: sample period will be stored in "Period" param
 * @Pre Condition: device should be initialized with BSP_MotionSensorInit
 ********************************************************************************************************************************************************************/
int32_t BSP_GetGyroPeriod(uint32_t *Period)
{
	int32_t ret = ISM330DHCX_GetGyroPeriod(&MotionSensor, Period);
	return ret;
}

/********************************************************************************************************************************************************************
 * @Brief: Synchronize Timer7 IRQ with Gyro/Accel sample frequency. Timer triggers and enables ReadAccel/Gyro thread.
 * @Param: none
 * @Return: none
 * @Post Condition: Timer7 and associated timer IRQ will be started.
 * @Pre Condition: device should be initialized with BSP_MotionSensorInit, Timer7 should be initialized but not started.
 ********************************************************************************************************************************************************************/
void BSP_SynchronizeIRQ(void)
{
    float dummy_data[3];
    //Read any data thats available
    BSP_ReadAccelXYZ(&dummy_data[0],&dummy_data[1],&dummy_data[2]);
    BSP_ReadGyroXYZ(&dummy_data[0],&dummy_data[1],&dummy_data[2]);
    //Wait for New Data
    while(BSP_ReadAccelXYZ(&dummy_data[0],&dummy_data[1],&dummy_data[2]) == ISM330DHCX_DataNotReady);
    BSP_ReadGyroXYZ(&dummy_data[0],&dummy_data[1],&dummy_data[2]);
    //Start timer
    HAL_TIM_Base_Start_IT(&htim7);
}

/***************************************************************************************************
 * @Brief: Initializes Drone Heading by taking 5 seconds worth of data from the accelerometer and
 *         gyroscope and taking the average... and doing some trig.
 * @Param: Attitude handle to initialize.
 * @Return: None
 ***************************************************************************************************/
void BSP_InitializeAHRS(AHRS_Handle_t *Attitude)
{
	float AvgAccX = 0, AvgAccY = 0, AvgAccZ = 0;
	float AvgGyroX = 0, AvgGyroY = 0, AvgGyroZ = 0;
	float DummyDataBuffer[3];
	uint32_t TimeStart = HAL_GetTick(); //Initialize a timer
	uint16_t AccSamples = 0, GyroSamples = 0;

	Attitude->P[0][0] = ROLL_INIT_STD*ROLL_INIT_STD; Attitude->P[0][1] = 0;                             Attitude->P[0][2] = 0; Attitude->P[0][3] = 0;
	Attitude->P[1][0] = 0;                           Attitude->P[1][1] = PITCH_INIT_STD*PITCH_INIT_STD; Attitude->P[1][2] = 0; Attitude->P[1][3] = 0;
	Attitude->P[2][0] = 0;                           Attitude->P[2][1] = 0;                             Attitude->P[2][2] = 0; Attitude->P[2][3] = 0;
	Attitude->P[3][0] = 0;                           Attitude->P[3][1] = 0;                             Attitude->P[3][2] = 0; Attitude->P[3][3] = 0;

	while(HAL_GetTick() < (TimeStart + AHRS_INIT_TIMESLICE))
	{

		if(BSP_ReadAccelXYZ(&DummyDataBuffer[0], &DummyDataBuffer[1], &DummyDataBuffer[2]) == ISM330DHCX_DataReady)      //Gather Data from Accel/Gyro for 5 seconds
		{
			AvgAccX +=  DummyDataBuffer[0];
			AvgAccY +=  DummyDataBuffer[1];
			AvgAccZ +=  DummyDataBuffer[2];
			AccSamples++;
		}

		if(BSP_ReadGyroXYZ(&DummyDataBuffer[0], &DummyDataBuffer[1], &DummyDataBuffer[2]) == ISM330DHCX_DataReady)
		{
			AvgGyroX +=  DummyDataBuffer[0];
			AvgGyroY +=  DummyDataBuffer[1];
			AvgGyroZ +=  DummyDataBuffer[2];
			GyroSamples++;
		}

	}
	AvgAccX = AvgAccX / AccSamples;                          //Calculate the Average acceleration vector and rotation vectors
	AvgAccY = AvgAccY / AccSamples;
	AvgAccZ = AvgAccZ / AccSamples;
	AvgGyroX = AvgGyroX / GyroSamples;
	AvgGyroY = AvgGyroY / GyroSamples;
	AvgGyroZ = AvgGyroZ / GyroSamples;

	Attitude->Roll      = RAD_2_DEG(BSP_atan(AvgAccY/AvgAccZ)); //Calculate roll
	Attitude->Pitch     = RAD_2_DEG(asin(AvgAccX));            //Calculuate pitch
	Attitude->RollRate  =  ROLL_RATE_INIT;                  //Drone starts out at rest.
	Attitude->PitchRate = PITCH_RATE_INIT;
	Attitude->YawRate   = YAW_RATE_INIT;
}

/***************************************************************************************************
 * @Brief: Convert orientation of measured vectors from Gyro/Accel sensors to "standard" flight
 *         orientation. i.e.: X axis = nose of plane, Y axis = wing of plane, Z axis = points down
 * @Param: X and Y measurements from accelerometer or gyroscope.
 * @Return: None
 ***************************************************************************************************/
inline static void ConvertVectorOrientation(float *Xvector, float *Yvector, float *Zvector)
{
	float temp = *Xvector;
	*Xvector = *Yvector;
	*Yvector = temp * (-1);
	*Zvector = -*Zvector;
}

/***************************************************************************************************
 * @Brief:
 * @Param:
 * @Return:
 ***************************************************************************************************/
void BSP_KalmanGyroPredict (AHRS_Handle_t *Attitude, float NewGyroX, float NewGyroY, float NewGyroZ, uint32_t TimeDelta)
{
	float dt = TimeDelta / 1000.0f;

	//Use Previous Gyro Measurements to new Predict Roll and Pitch Angle
	Attitude->Roll =   Attitude->Roll  +  Attitude->RollRate * dt;
	Attitude->Pitch =  Attitude->Pitch +  Attitude->PitchRate * dt;

	//Calculate Constants using predicted data
	float SinRoll  = BSP_sin(DEG_2_RAD(Attitude->Roll));
	float CosRoll  = BSP_cos(DEG_2_RAD(Attitude->Roll));
	float CosPitch = BSP_cos(DEG_2_RAD(Attitude->Pitch));
	float TanPitch = BSP_sin(DEG_2_RAD(Attitude->Pitch)) / BSP_cos(DEG_2_RAD(Attitude->Pitch));

	//Compute New Roll, Pitch, and Yaw Rates
	Attitude->RollRate  =  (NewGyroX + TanPitch * (NewGyroY * SinRoll + NewGyroZ * CosRoll));
	Attitude->PitchRate =  (NewGyroY*CosRoll    -  NewGyroZ*SinRoll);
	Attitude->YawRate   =  (NewGyroY*SinRoll / CosPitch) + (NewGyroZ * CosRoll / CosPitch);

	Attitude->P[0][0] = ((dt*dt*dt*dt*GYRO_STD_DEV*GYRO_STD_DEV)/4) + dt*(Attitude->P[0][2]+Attitude->P[2][2]*dt) + (Attitude->P[0][0] + Attitude->P[2][0]*dt);
	Attitude->P[0][1] = dt*(Attitude->P[0][3] + Attitude->P[2][3]*dt) + Attitude->P[0][1] + Attitude->P[2][1]*dt;
	Attitude->P[0][2] = ((dt*dt*dt*GYRO_STD_DEV*GYRO_STD_DEV)/2) + (Attitude->P[0][2] + Attitude->P[2][2]*dt);
	Attitude->P[0][3] = Attitude->P[0][3] + Attitude->P[2][3]*dt;

	Attitude->P[1][0] = dt*(Attitude->P[1][2] + Attitude->P[3][2]*dt) + Attitude->P[1][0]+Attitude->P[3][0]*dt;
	Attitude->P[1][1] = ((dt*dt*dt*dt*GYRO_STD_DEV*GYRO_STD_DEV)/4) + dt*(Attitude->P[1][3]+Attitude->P[3][3]*dt) + Attitude->P[1][1] + Attitude->P[3][1]*dt;
	Attitude->P[1][2] = Attitude->P[1][2] + Attitude->P[3][2]*dt;
	Attitude->P[1][3] = (dt*dt*dt*GYRO_STD_DEV*GYRO_STD_DEV/2) + Attitude->P[1][3] + Attitude->P[3][3]*dt;

	Attitude->P[2][0] = Attitude->P[2][0] + Attitude->P[2][2]*dt + (dt*dt*dt*GYRO_STD_DEV*GYRO_STD_DEV/2);
	Attitude->P[2][1] = Attitude->P[2][1] + Attitude->P[2][3]*dt;
	Attitude->P[2][2] = Attitude->P[2][2] + dt*dt*GYRO_STD_DEV*GYRO_STD_DEV;
	Attitude->P[2][3] = Attitude->P[2][3];

	Attitude->P[3][0] = Attitude->P[3][0] + Attitude->P[3][2]*dt;
	Attitude->P[3][1] = Attitude->P[3][1] + Attitude->P[3][3]*dt + (dt*dt*dt*GYRO_STD_DEV*GYRO_STD_DEV/2);
	Attitude->P[3][2] = Attitude->P[3][2];
	Attitude->P[3][3] = Attitude->P[3][3] + dt*dt*GYRO_STD_DEV*GYRO_STD_DEV;

};

void BSP_KalmanAccelUpdate(AHRS_Handle_t *Attitude, float AccelX, float AccelY, float AccelZ)
{
 	 float RollNew      = RAD_2_DEG(atan2(AccelY,AccelZ)); //Calculate roll
 	 float PitchNew     = RAD_2_DEG(asin(AccelX));             //Calculuate pitch

 	 float SigmaSqr    = ACCEL_STD_DEV*ACCEL_STD_DEV;
 	 float denominator = -Attitude->P[0][1]*Attitude->P[1][0] + (Attitude->P[0][0] + SigmaSqr)*(Attitude->P[1][1] + SigmaSqr);
 	 float RollDelta   = RollNew - Attitude->Roll;
 	 float PitchDelta  = PitchNew - Attitude->Pitch;


 	 Attitude->Roll      = Attitude->Roll        + (RollDelta*(-Attitude->P[0][1]*Attitude->P[1][0] + Attitude->P[0][0]*(Attitude->P[1][1] + SigmaSqr)) + PitchDelta*(-Attitude->P[0][0]*Attitude->P[0][1] + Attitude->P[0][1]*(Attitude->P[0][0] + SigmaSqr))) / denominator;
 	 Attitude->Pitch     = Attitude->Pitch       + (RollDelta*(-Attitude->P[1][0]*Attitude->P[1][1] + Attitude->P[1][0]*(Attitude->P[1][1] + SigmaSqr)) + PitchDelta*(-Attitude->P[0][1]*Attitude->P[1][0] + Attitude->P[1][1]*(Attitude->P[0][0] + SigmaSqr))) / denominator;
 	 Attitude->RollRate  = Attitude->RollRate    + (RollDelta*(-Attitude->P[1][0]*Attitude->P[2][1] + Attitude->P[2][0]*(Attitude->P[1][1] + SigmaSqr)) + PitchDelta*(-Attitude->P[0][1]*Attitude->P[2][0] + Attitude->P[2][1]*(Attitude->P[0][0] + SigmaSqr))) / denominator;
 	 Attitude->PitchRate = Attitude->PitchRate   + (RollDelta*(-Attitude->P[1][0]*Attitude->P[3][1] + Attitude->P[3][0]*(Attitude->P[1][1] + SigmaSqr)) + PitchDelta*(-Attitude->P[0][1]*Attitude->P[3][0] + Attitude->P[3][1]*(Attitude->P[0][0] + SigmaSqr))) / denominator;

 	 Attitude->P[0][0] = Attitude->P[0][0]*((Attitude->P[0][1]*Attitude->P[1][0] - Attitude->P[0][0]*(Attitude->P[1][1] + SigmaSqr))/denominator + 1) + Attitude->P[1][0]*((Attitude->P[0][0]*Attitude->P[0][1] - Attitude->P[0][1]*(Attitude->P[0][0] + SigmaSqr))/denominator);
 	 Attitude->P[0][1] = Attitude->P[0][1]*((Attitude->P[0][1]*Attitude->P[1][0] - Attitude->P[0][0]*(Attitude->P[1][1] + SigmaSqr))/denominator + 1) + Attitude->P[1][1]*((Attitude->P[0][0]*Attitude->P[0][1] - Attitude->P[0][1]*(Attitude->P[0][0] + SigmaSqr))/denominator);
 	 Attitude->P[0][2] = Attitude->P[0][2]*((Attitude->P[0][1]*Attitude->P[1][0] - Attitude->P[0][0]*(Attitude->P[1][1] + SigmaSqr))/denominator + 1) + Attitude->P[1][2]*((Attitude->P[0][0]*Attitude->P[0][1] - Attitude->P[0][1]*(Attitude->P[0][0] + SigmaSqr))/denominator);
 	 Attitude->P[0][3] = Attitude->P[0][3]*((Attitude->P[0][1]*Attitude->P[1][0] - Attitude->P[0][0]*(Attitude->P[1][1] + SigmaSqr))/denominator + 1) + Attitude->P[1][3]*((Attitude->P[0][0]*Attitude->P[0][1] - Attitude->P[0][1]*(Attitude->P[0][0] + SigmaSqr))/denominator);

 	 Attitude->P[1][0] = Attitude->P[0][0]*((Attitude->P[1][0]*Attitude->P[1][1] - Attitude->P[1][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][0]*((Attitude->P[0][1]*Attitude->P[1][0] - Attitude->P[1][1]*(Attitude->P[0][0] + SigmaSqr))/denominator + 1);
 	 Attitude->P[1][1] = Attitude->P[0][1]*((Attitude->P[1][0]*Attitude->P[1][1] - Attitude->P[1][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][1]*((Attitude->P[0][1]*Attitude->P[1][0] - Attitude->P[1][1]*(Attitude->P[0][0] + SigmaSqr))/denominator + 1);
 	 Attitude->P[1][2] = Attitude->P[0][2]*((Attitude->P[1][0]*Attitude->P[1][1] - Attitude->P[1][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][2]*((Attitude->P[0][1]*Attitude->P[1][0] - Attitude->P[1][1]*(Attitude->P[0][0] + SigmaSqr))/denominator + 1);
 	 Attitude->P[1][3] = Attitude->P[0][3]*((Attitude->P[1][0]*Attitude->P[1][1] - Attitude->P[1][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][3]*((Attitude->P[0][1]*Attitude->P[1][0] - Attitude->P[1][1]*(Attitude->P[0][0] + SigmaSqr))/denominator + 1);

 	 Attitude->P[2][0] = Attitude->P[0][0]*((Attitude->P[1][0]*Attitude->P[2][1] - Attitude->P[2][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][0]*((Attitude->P[0][1]*Attitude->P[2][0] - Attitude->P[2][1]*(Attitude->P[0][0] + SigmaSqr))/denominator) + Attitude->P[2][0];
 	 Attitude->P[2][1] = Attitude->P[0][1]*((Attitude->P[1][0]*Attitude->P[2][1] - Attitude->P[2][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][1]*((Attitude->P[0][1]*Attitude->P[2][0] - Attitude->P[2][1]*(Attitude->P[0][0] + SigmaSqr))/denominator) + Attitude->P[2][1];
 	 Attitude->P[2][2] = Attitude->P[0][2]*((Attitude->P[1][0]*Attitude->P[2][1] - Attitude->P[2][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][2]*((Attitude->P[0][1]*Attitude->P[2][0] - Attitude->P[2][1]*(Attitude->P[0][0] + SigmaSqr))/denominator) + Attitude->P[2][2];
 	 Attitude->P[2][3] = Attitude->P[0][3]*((Attitude->P[1][0]*Attitude->P[2][1] - Attitude->P[2][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][3]*((Attitude->P[0][1]*Attitude->P[2][0] - Attitude->P[2][1]*(Attitude->P[0][0] + SigmaSqr))/denominator) + Attitude->P[2][3];

 	 Attitude->P[3][0] = Attitude->P[0][0]*((Attitude->P[1][0]*Attitude->P[3][1] - Attitude->P[3][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][0]*((Attitude->P[0][1]*Attitude->P[3][0] - Attitude->P[3][1]*(Attitude->P[0][0] + SigmaSqr))/denominator) + Attitude->P[3][0];
 	 Attitude->P[3][1] = Attitude->P[0][1]*((Attitude->P[1][0]*Attitude->P[3][1] - Attitude->P[3][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][1]*((Attitude->P[0][1]*Attitude->P[3][0] - Attitude->P[3][1]*(Attitude->P[0][0] + SigmaSqr))/denominator) + Attitude->P[3][1];
 	 Attitude->P[3][2] = Attitude->P[0][2]*((Attitude->P[1][0]*Attitude->P[3][1] - Attitude->P[3][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][2]*((Attitude->P[0][1]*Attitude->P[3][0] - Attitude->P[3][1]*(Attitude->P[0][0] + SigmaSqr))/denominator) + Attitude->P[3][2];
 	 Attitude->P[3][3] = Attitude->P[0][3]*((Attitude->P[1][0]*Attitude->P[3][1] - Attitude->P[3][0]*(Attitude->P[1][1] + SigmaSqr))/denominator)+ Attitude->P[1][3]*((Attitude->P[0][1]*Attitude->P[3][0] - Attitude->P[3][1]*(Attitude->P[0][0] + SigmaSqr))/denominator) + Attitude->P[3][3];

}
