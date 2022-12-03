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

/* Private Variables (Sensor Handle) */
static ISM330DHCX_Handle_t MotionSensor;

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
			.HPCF_XL = ISM330DHCX_ODRDiv45,
			.USR_OFF_ON_OUT = ISM330DHCX_XL_UserOffsetOutputDisabled,
			.USR_OFF_W = ISM330DHCX_DefaultSetting,
			.X_OFS_USR = ISM330DHCX_DefaultSetting,
			.Y_OFS_USR = ISM330DHCX_DefaultSetting,
			.Z_OFS_USR = ISM330DHCX_DefaultSetting,
			.HP_REF_MODE_XL = ISM330DHCX_XL_HighPassRefDisabled,
			.FASTSETTL_MODE_XL = ISM330DHCX_DefaultSetting,

			.FS_XL = ISM330DHCX_XL_8g,

			.ODR_G = ISM330DHCX_G_52Hz,
			.HPM_G = ISM330DHCX_DefaultSetting,
			.HP_EN_G = ISM330DHCX_G_HP_Disabled,
			.FTYPE = ISM330DHCX_DefaultSetting,
			.LPF1_SEL_G = ISM330DHCX_LPF_Disabled,
			.G_HM_MODE = ISM330DHCX_G_HPModeEnabled,

			.FS_G = ISM330DHCX_G_1000DPS,

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
        if(BSP_ReadAccelXYZ(&JunkData[0],&JunkData[1],&JunkData[2]) ==ISM330DHCX_DataReady)
        {
        	DiscardedXLSamples++;
        }

        if(BSP_ReadGyroXYZ(&JunkData[0],&JunkData[1],&JunkData[2]) ==ISM330DHCX_DataReady)
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
