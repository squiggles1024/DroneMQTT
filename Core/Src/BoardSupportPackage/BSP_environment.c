/*
 * BSP_environment.c
 *
 *  Created on: Sep 16, 2022
 *      Author: evanl
 */
/* Includes */
#include "../ExternalHardware/HTS221.h"
#include "../ExternalHardware/LPS22HH.h"
#include "../ExternalHardware/IIS2MDC.h"
#include "../ExternalHardware/VEML6030.h"
#include "BSP_IOBus.h"
#include "BSP_environment.h"
#include <stddef.h>
#include <stdint.h>

/* Sensor Device Handles */
static HTS221_Handle_t TempSensor_Handle = {0};
static LPS22HH_Handle_t PressureSensor = {0};
static IIS2MDC_Handle_t Magnetometer = {0};
static VEML6030_Handle_t AmbientLightSensor = {0};

/*****************************************************************************************************************************************
 *@Brief: Initializes Temperature and Humidity Sensor handle and hardware
 *@Param: None
 *@Return: Status - Either HTS221 Status or HAL Status depending on where/if an error occured
 *@Post Condition: HTS221 Temp/Hum sensor will be initialized and ready to use
 *@Pre Condition: GPIO Pins should already be configured
 *****************************************************************************************************************************************/
int32_t BSP_TempHumSensorInit(void)
{
	int32_t ret = 0;
	HTS221_IO_t TempSensor_IO = {.Init = BSP_I2C2_Init,
			                     .DeInit = BSP_I2C2_DeInit,
								 .Read = BSP_I2C2_ReadRegHTS221,
								 .Write = BSP_I2C2_WriteRegHTS221,
								 .GetTick = BSP_GetTick,
								 .ReadPin = NULL
	};

	HTS221_Init_Struct_t Settings = { .DataReadyConfig = HTS221_DataReadyDisable,
			                          .DataReadyOutputMode = HTS221_DRDY_PushPull,
									  .DataReadySignalPolarity = HTS221_DataActiveHigh,
									  .HumidityResolution = HTS221_Hum64Samples,
									  .OutputDataRate = HTS221_1_0Hz,
									  .TemperatureResolution = HTS221_Temp32amples

	};
	ret = HTS221_Init(&TempSensor_Handle, Settings, &TempSensor_IO);
	return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Reads temperature sensor
 *@Param: Pointer to float that will store the result
 *@Return: Status - Either HTS221 Status or HAL Status depending on where/if an error occured, OR DataNotReady if no new sample is available
 *@Post Condition: If new data was available, "Result" will contain the temperature in C
 *@Pre Condition: BSP_TempHumSensorInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_ReadTemperature(float *result)
{
	return HTS221_ReadTemperature(&TempSensor_Handle, result);
}

/*****************************************************************************************************************************************
 *@Brief: Reads humidity sensor
 *@Param: Pointer to float that will store the result
 *@Return: Status - Either HTS221 Status or HAL Status depending on where/if an error occured, OR DataNotReady if no new sample is available
 *@Post Condition: If new data was available, "Result" will contain the relative humidity
 *@Pre Condition: BSP_TempHumSensorInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_ReadHumidity(float *result)
{
	return HTS221_ReadHumidity(&TempSensor_Handle, result);
}

/*****************************************************************************************************************************************
 *@Brief: Turns on HTS221 internal heater. Can be used to remove condensation
 *@Param: None
 *@Return: Status - Either HTS221 Status or HAL Status depending on where/if an error occured
 *@Post Condition: HTS221 Heater will be turned on
 *@Pre Condition: BSP_TempHumSensorInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_EnableHeater(void)
{
	return HTS221_EnableHeater(&TempSensor_Handle);
}

/*****************************************************************************************************************************************
 *@Brief: Turns off HTS221 internal heater
 *@Param: None
 *@Return: Status - Either HTS221 Status or HAL Status depending on where/if an error occured
 *@Post Condition: HTS221 Heater will be turned off
 *@Pre Condition: BSP_TempHumSensorInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_DisableHeater(void)
{
	return HTS221_DisableHeater(&TempSensor_Handle);
}

/*****************************************************************************************************************************************
 *@Brief: Initializes Pressure Sensor handle and hardware
 *@Param: None
 *@Return: Status - Either LPS22HH Status or HAL Status depending on where/if an error occured
 *@Post Condition: LPS22HH Pressure sensor will be initialized and ready to use
 *@Pre Condition: GPIO Pins should already be configured
 *****************************************************************************************************************************************/
int32_t BSP_PressureSensorInit(void)
{
	LPS22HH_IO_t PressureSensor_IO =
	{
			.Init = BSP_LPS22HH_IO_Init,
			.DeInit = NULL,
			.Read = BSP_I2C2_ReadRegLPS22HH,
			.Write = BSP_I2C2_WriteRegLPS22HH,
			.GetTick = BSP_GetTick,
			.ReadPin = BSP_ReadPinLPS22HH
	};

	LPS22HH_Init_Struct_t PressureSettings =
	{
			.INTERRUPT_CFG = LPS22HH_DefaultSetting,
			.THS_P = LPS22HH_DefaultSetting,
			.IF_CTRL = LPS22HH_DefaultSetting,
			.ODR = LPS22HH_10Hz,
			.LPFP = LPS22HH_ODRDiv9,
			.SIM = LPS22HH_DefaultSetting,
			.CTRL_REG2 = LPS22HH_IncrementAddress | LPS22HH_LowNoiseMode,
			.INT_DRDY_Control = LPS22HH_DRDY,
			.INT_S = LPS22HH_DataReady,
			.STOP_ON_WTM = LPS22HH_DefaultSetting,
			.F_MODE = LPS22HH_FifoBypass,
			.WTM = LPS22HH_DefaultSetting,
			.REF_P = LPS22HH_DefaultSetting
	};

	int32_t ret = LPS22HH_Init(&PressureSensor, PressureSettings, &PressureSensor_IO);
	return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Reads pressure sensor
 *@Param: Pointer to float that will store the result
 *@Return: Status - Either LPS22HH Status or HAL Status depending on where/if an error occured, OR DataNotReady if no new sample is available
 *@Post Condition: If new data was available, "Result" will contain the pressure in hPa
 *@Pre Condition: BSP_PressureSensorInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_ReadPressure(float *result)
{
	int32_t ret = LPS22HH_ReadPressure(&PressureSensor, result);
	return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Initializes magnetometer handle and hardware
 *@Param: None
 *@Return: Status - Either IIS2MDC Status or HAL Status depending on where/if an error occured
 *@Post Condition: IIS2MDC magnetometer sensor will be initialized and ready to use
 *@Pre Condition: GPIO Pins should already be configured
 *****************************************************************************************************************************************/
int32_t BSP_MagnetometerInit(void)
{
    IIS2MDC_IO_t MagnetometerIO =
    {
    		.Init = BSP_IIS2MDC_IO_Init,
			.Deinit = NULL,
			.Read = BSP_I2C2_ReadRegIIS2MDC,
			.Write = BSP_I2C2_WriteRegIIS2MDC,
			.GetTick = BSP_GetTick,
			.ReadPin = BSP_ReadPinIIS2MDC
    };

    IIS2MDC_InitStruct_t MagnetometerSettings =
    {
    		.ConfigRegA = IIS2MDC_ODR_10Hz | IIS2MDC_TempCompensationEnabled,
			.ConfigRegB = IIS2MDC_LPF,
			.ConfigRegC = IIS2MDC_DRDY_on_PIN | IIS2MDC_BDU,
			.IntCtrlReg = IIS2MDC_DefaultSetting,
			.IntThreshold = 0,
			.OffsetX = 0,
			.OffsetY = 0,
			.OffsetZ = 0
    };

    int32_t ret = IIS2MDC_Init(&Magnetometer, MagnetometerSettings, &MagnetometerIO);
    return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Reads all magnetometer sensor axes
 *@Param: Pointer to floats that will store the results
 *@Return: Status - Either IIS2MDC Status or HAL Status depending on where/if an error occured, OR DataNotReady if no new samples are available
 *@Post Condition: If new data was available, "Mx", "My", "Mz"  will contain the magnetic fields in milligause (UNCALIBRATED)
 *@Pre Condition: BSP_MagnetometerInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_ReadMagnetometerXYZ(float *Mx, float *My, float *Mz)
{
    int32_t ret = IIS2MDC_ReadMagnetismXYZ(&Magnetometer, Mx, My, Mz);
    return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Reads magnetometer sensor X Axis
 *@Param: Pointer to float that will store the result
 *@Return: Status - Either IIS2MDC Status or HAL Status depending on where/if an error occured, OR DataNotReady if no new sample is available
 *@Post Condition: If new data was available, "Mx"  will contain the magnetic fields in milligause (UNCALIBRATED)
 *@Pre Condition: BSP_MagnetometerInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_ReadMagnetometerX(float *Mx)
{
	int32_t ret = IIS2MDC_ReadMagnetismX(&Magnetometer, Mx);
	return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Reads magnetometer sensor Y Axis
 *@Param: Pointer to float that will store the result
 *@Return: Status - Either IIS2MDC Status or HAL Status depending on where/if an error occured, OR DataNotReady if no new sample is available
 *@Post Condition: If new data was available, "My"  will contain the magnetic fields in milligause (UNCALIBRATED)
 *@Pre Condition: BSP_MagnetometerInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_ReadMagnetometerY(float *My)
{
	int32_t ret = IIS2MDC_ReadMagnetismY(&Magnetometer, My);
	return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Reads magnetometer sensor Z Axis
 *@Param: Pointer to float that will store the result
 *@Return: Status - Either IIS2MDC Status or HAL Status depending on where/if an error occured, OR DataNotReady if no new sample is available
 *@Post Condition: If new data was available, "Mz"  will contain the magnetic fields in milligause (UNCALIBRATED)
 *@Pre Condition: BSP_MagnetometerInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_ReadMagnetometerZ(float *Mz)
{
	int32_t ret = IIS2MDC_ReadMagnetismZ(&Magnetometer, Mz);
	return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Calculates the period based on the temperatures sensor sample period (in ms). Useful for putting threads to sleep for a time period
 *@Param: Pointer to uint32_t that will store the period
 *@Return: Status - Either HTS221 Status or HAL Status depending on where/if an error occured
 *@Post Condition: Period will contain the period in ms
 *@Pre Condition: BSP_TempHumSensorInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_GetTempPeriod(uint32_t *Period)
{
    int32_t ret = HTS221_GetSamplePeriod(&TempSensor_Handle, Period);
    return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Calculates the period based on the magnetometer sensor sample period (in ms). Useful for putting threads to sleep for a time period
 *@Param: Pointer to uint32_t that will store the period
 *@Return: Status - Either IIS2MDC Status or HAL Status depending on where/if an error occured
 *@Post Condition: Period will contain the period in ms
 *@Pre Condition: BSP_MagnetometerInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_GetMagneticPeriod(uint32_t *Period)
{
    int32_t ret = IIS2MDC_GetSamplePeriod(&Magnetometer, Period);
    return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Calculates the period based on the pressure sensor sample period (in ms). Useful for putting threads to sleep for a time period
 *@Param: Pointer to uint32_t that will store the period
 *@Return: Status - Either LPS22HH Status or HAL Status depending on where/if an error occured
 *@Post Condition: Period will contain the period in ms
 *@Pre Condition: BSP_PressureSensorInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_GetPressurePeriod(uint32_t *Period){
    int32_t ret = LPS22HH_GetSamplePeriod(&PressureSensor, Period);
    return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Initializes Ambient Light Sensor handle and hardware
 *@Param: None
 *@Return: Status - Either VEML6030 Status or HAL Status depending on where/if an error occured
 *@Post Condition: VEML6030 Ambient Light sensor will be initialized and ready to use
 *@Pre Condition: GPIO Pins should already be configured
 *****************************************************************************************************************************************/
int32_t BSP_AmbientLightInit(void)
{
    VEML6030_IO_t IO = {
    		.Init = BSP_I2C2_Init,
			.Deinit = NULL,
			.GetTick = BSP_GetTick,
			.Read = BSP_I2C2_ReadRegVEML6030,
			.Write = BSP_I2C2_WriteRegVEML6030
    };

    VEML_InitSettings_t Settings =
    {
    		.ALSConfig = VEML6030_INTTIME_100 | VEML6030_ALS_GAIN_DIV4 | VEML6030_PERS_1,
			.PowerSavingMode = VEML_POWERSAVING_MODE1
    };

    int32_t ret = VEML6030_Init(&AmbientLightSensor, Settings, &IO);
    return ret;
}

/*****************************************************************************************************************************************
 *@Brief: Reads ambient light sensor
 *@Param: Pointer to float that will store the result
 *@Return: Status - Either VEML6030 Status or HAL Status depending on where/if an error occured
 *@Post Condition: If new data was available, "Light"  will contain the ambient light in lux
 *@Pre Condition: BSP_AmbientLightInit should have been called already.
 *****************************************************************************************************************************************/
int32_t BSP_ReadAmbientLight(float *Light)
{
    int32_t ret = VEML6030_ReadLight(&AmbientLightSensor, Light);
    return ret;
}


