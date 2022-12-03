/*
 * VEML6030.c
 *
 *  Created on: Oct 27, 2022
 *      Author: evanl
 */
#include "VEML6030.h"
#include <stdint.h>
#include <stddef.h>

//Used to fetch resolution according to IT and GAIN settings
//Index using [ALS_IT][ALS_GAIN]
static const float ResolutionLookUpTable[6][4] = {
		{0.0036, 0.0072, 0.0288, 0.0576},
		{0.0072, 0.0144, 0.0576, 0.1152},
		{0.0144, 0.0288, 0.1152, 0.2304},
		{0.0288, 0.0576, 0.2304, 0.4608},
		{0.0576, 0.1152, 0.4608, 0.9216},
		{0.1152, 0.2304, 0.9216, 1.8432}
};

/* Private Function Prototypes */
static void ConvertLight(VEML6030_Handle_t *Handle, uint16_t Buffer, float *Light);
static int32_t VEML6030_ReadRegWrapper(void *Handle, uint8_t Reg, uint8_t *Buffer, uint8_t Length);
static int32_t VEML6030_WriteRegWrapper(void *Handle, uint8_t Reg, uint8_t *Buffer, uint8_t Length);

/***********************************************************************************************************************************
 * @Brief: Initializes a VEML6030 Device Handle
 * @Params: VEML6030 Handle to initialize, Settings for the handle to be initialized with, Low level IO struct to communicate w/ Bus
 * @Return: Status Code
 * @Pre Condition: None
 * @Post Condition: Handle will be initialized according to given settings, Hardware initialized according to provided IO.init()
 ***********************************************************************************************************************************/
int32_t VEML6030_Init(VEML6030_Handle_t *Handle, VEML_InitSettings_t Settings, const VEML6030_IO_t *IO)
{
	int32_t ret = VEML6030_Ok;
	if(Handle == NULL)
	{
        return VEML6030_HandleError;
	}

	if(Settings.PowerSavingMode > 0x07)
	{
        return VEML6030_PSM_Err;
	}

	if(Handle->Status != VEML6030_STATUS_INITIALIZED)
	{
		if(IO->Read == NULL || IO->Write == NULL)
		{
			return VEML6030_IOError;
		}

        uint16_t ALS_IT = (Settings.ALSConfig >> 6) & 0xF;
        uint16_t ALS_GAIN = (Settings.ALSConfig >> 11) & 0x3;
        if(ALS_GAIN > 3 || (ALS_IT != 0 && ALS_IT != 1 && ALS_IT != 2 && ALS_IT != 3 && ALS_IT != 8 && ALS_IT != 0xC))
        {
        	return VEML6030_ParamError;
        }
		Handle->Resolution = ResolutionLookUpTable[ALS_IT][ALS_GAIN];

		Handle->Context.Handle = Handle;
		Handle->Context.Write = VEML6030_WriteRegWrapper;
		Handle->Context.Read = VEML6030_ReadRegWrapper;
		Handle->IO = *IO;
		if(Handle->IO.Init != NULL)
		{
            Handle->IO.Init();
		}

		ret = VEML6030_WriteReg(&Handle->Context, VEML6030_REG_ALS_CONF, (uint8_t*)&Settings.ALSConfig, 2);
		if(ret != VEML6030_Ok)
		{
			return ret;
		}

		ret = VEML6030_WriteReg(&Handle->Context, VEML6030_REG_POWERSAVING, (uint8_t*)&Settings.PowerSavingMode, 2);
		if(ret != VEML6030_Ok)
		{
			return ret;
		}
		Handle->Status = VEML6030_STATUS_INITIALIZED;
	}
	return VEML6030_Ok;
}

/***********************************************************************************************************************************
 * @Brief: Reads light from VEML6030 device
 * @Params: Device Handle, float buffer to store data in
 * @Return: Status code
 * @Pre Condition: VEML6030 handle must be initialized
 * @Post Condition: "Light" param will contain light value in Lux
 ***********************************************************************************************************************************/
int32_t VEML6030_ReadLight(VEML6030_Handle_t *Handle, float *Light)
{
	if(Handle == NULL)
	{
		return VEML6030_HandleError;
	}

	if(Handle->Status != VEML6030_STATUS_INITIALIZED)
	{
		return VEML6030_InitError;
	}
	uint16_t Buffer;
	int32_t ret = VEML6030_ReadReg(&Handle->Context, VEML6030_REG_ALS, (uint8_t*)&Buffer, 2);
	if(ret != VEML6030_Ok)
	{
		return ret;
	}
	ConvertLight(Handle,Buffer,Light);
	return VEML6030_Ok;
}

/***********************************************************************************************************************************
 * @Brief: Sets high Threshold for VEML6030 IRQ
 * @Params: Device Handle, value to set the threshold to
 * @Return: Status Code
 * @Pre Condition: VEML6030 handle must be initialized
 * @Post Condition: High Threshold register in VEML6030 device will be written to.
 ***********************************************************************************************************************************/
int32_t VEML6030_SetALSHighThreshold(VEML6030_Handle_t *Handle, float Light)
{
	if(Handle == NULL)
	{
		return VEML6030_HandleError;
	}

	if(Handle->Status != VEML6030_STATUS_INITIALIZED)
	{
		return VEML6030_InitError;
	}

	uint32_t temp = Light / Handle->Resolution;
	uint16_t buffer;
	if(temp > 0x0000FFFF)
	{
		buffer = 0xFFFF;
	} else if(Light < 0)
	{
		return VEML6030_ParamError;
	} else
	{
		buffer = temp;
	}

	int32_t ret = VEML6030_WriteReg(&Handle->Context, VEML6030_REG_ALS_WH, (uint8_t*)&buffer, 2);
	if(ret != VEML6030_Ok)
	{
		return ret;
	}
	return VEML6030_Ok;
}

/***********************************************************************************************************************************
 * @Brief: Sets low Threshold for VEML6030 IRQ
 * @Params: Device Handle, value to set the threshold to
 * @Return: Status Code
 * @Pre Condition: VEML6030 handle must be initialized
 * @Post Condition: Low Threshold register in VEML6030 device will be written to.
 ***********************************************************************************************************************************/
int32_t VEML6030_SetALSLowThreshold(VEML6030_Handle_t *Handle, float Light)
{
	if(Handle == NULL)
	{
		return VEML6030_HandleError;
	}

	if(Handle->Status != VEML6030_STATUS_INITIALIZED)
	{
		return VEML6030_InitError;
	}

	uint32_t temp = Light / Handle->Resolution;
	uint16_t buffer;
	if(temp > 0x0000FFFF)
	{
		buffer = 0xFFFF;
	} else if(Light < 0)
	{
		return VEML6030_ParamError;
	} else
	{
		buffer = temp;
	}

	int32_t ret = VEML6030_WriteReg(&Handle->Context, VEML6030_REG_ALS_WL, (uint8_t*)&buffer, 2);
	if(ret != VEML6030_Ok)
	{
		return ret;
	}
	return VEML6030_Ok;
}

/***********************************************************************************************************************************
 * @Brief: Retrieves ALS High Threshold value
 * @Params: Device Handle, float buffer to store register value
 * @Return: Status Code
 * @Pre Condition: VEML6030 handle must be initialized
 * @Post Condition: High Threshold register in VEML6030 device will be read and stored in "Light" param.
 ***********************************************************************************************************************************/
int32_t VEML6030_GetALSHighThreshold(VEML6030_Handle_t *Handle, float *Light)
{
	if(Handle == NULL)
	{
		return VEML6030_HandleError;
	}

	if(Handle->Status != VEML6030_STATUS_INITIALIZED)
	{
		return VEML6030_InitError;
	}
    uint16_t buffer = 0;
	int32_t ret = VEML6030_WriteReg(&Handle->Context, VEML6030_REG_ALS_WH, (uint8_t*)&buffer, 2);
	if(ret != VEML6030_Ok)
	{
		return ret;
	}
	ConvertLight(Handle, buffer, Light);

	return VEML6030_Ok;
}

/***********************************************************************************************************************************
 * @Brief: Retrieves ALS Low Threshold value
 * @Params: Device Handle, float buffer to store register value
 * @Return: Status Code
 * @Pre Condition: VEML6030 handle must be initialized
 * @Post Condition: Low Threshold register in VEML6030 device will be read and stored in "Light" param.
 ***********************************************************************************************************************************/
int32_t VEML6030_GetALSLowThreshold(VEML6030_Handle_t *Handle, float *Light)
{
	if(Handle == NULL)
	{
		return VEML6030_HandleError;
	}

	if(Handle->Status != VEML6030_STATUS_INITIALIZED)
	{
		return VEML6030_InitError;
	}
    uint16_t buffer = 0;
	int32_t ret = VEML6030_WriteReg(&Handle->Context, VEML6030_REG_ALS_WL, (uint8_t*)&buffer, 2);
	if(ret != VEML6030_Ok)
	{
		return ret;
	}
	ConvertLight(Handle, buffer, Light);

	return VEML6030_Ok;
}

/***********************************************************************************************************************************
 * @Brief: Reads VEML6030 Device Register
 * @Params: Address of Device Handle, Register to read, Buffer to store read data, number of bytes to read
 * @Return: Status Code
 * @Pre Condition: VEML6030 must have functional IO functions
 * @Post Condition: "Buffer" param will store read bytes
 ***********************************************************************************************************************************/
static int32_t VEML6030_ReadRegWrapper(void *Handle, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	VEML6030_Handle_t *Dev = Handle;
	int32_t ret = VEML6030_Ok;
	if(Dev->IO.Read != NULL)
	{
        ret = Dev->IO.Read(Reg,Buffer,Length);
        return ret;
	}
    return VEML6030_IOError;
}

/***********************************************************************************************************************************
 * @Brief: Writes to VEML6030 Device Register
 * @Params: Address of device handle, Reg to read from, Buffer with data to write, number of bytes to write
 * @Return: Status Code
 * @Pre Condition: VEML6030 must have functional IO functions
 * @Post Condition: VEML6030 Register "Reg" will be written to with data in "Buffer"
 ***********************************************************************************************************************************/
static int32_t VEML6030_WriteRegWrapper(void *Handle, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	VEML6030_Handle_t *Dev = Handle;
	int32_t ret = VEML6030_Ok;
	if(Dev->IO.Write != NULL)
	{
        ret = Dev->IO.Write(Reg,Buffer,Length);
        return ret;
	}
    return VEML6030_IOError;
}

/***********************************************************************************************************************************
 * @Brief:
 * @Params:
 * @Return:
 * @Pre Condition:
 * @Post Condition:
 ***********************************************************************************************************************************/
static void ConvertLight(VEML6030_Handle_t *Handle, uint16_t Buffer, float *Light)
{
	*Light = Buffer * Handle->Resolution;
}
