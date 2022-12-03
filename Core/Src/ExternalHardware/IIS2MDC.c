/*
 * IIS2MDC.c
 *
 *  Created on: Sep 15, 2022
 *      Author: evanl
 */

#include "IIS2MDC.h"
#include <stddef.h>

//Define Magic Numbers
#define IIS2MDC_ResetSignal (1 << 5)
#define IIS2MDC_RebootSignal (1 << 6)
#define IIS2MDC_StartConvSignal (1)
#define IIS2MDC_XDATA_Msk (0x04)
#define IIS2MDC_YDATA_Msk (0x02)
#define IIS2MDC_ZDATA_Msk (0x01)
#define IIS2MDC_ODR_Msk (0x0F)

//Private Function Prototypes
static int32_t IIS2MDC_SWReset(IIS2MDC_Handle_t *Handle);
static int32_t IIS2MDC_Reboot(IIS2MDC_Handle_t *Handle);
static int32_t IIS2MDC_WriteRegWrapper(void *Handle,uint8_t Reg, uint8_t *Buffer, uint8_t Length);
static int32_t IIS2MDC_ReadRegWrapper(void *Handle,uint8_t Reg, uint8_t *Buffer, uint8_t Length);
static int32_t IIS2MDC_RegisterInit(IIS2MDC_Handle_t *Handle, IIS2MDC_InitStruct_t Settings);
static void ConvertMagData(uint8_t *buffer, float *magnetism);
static void DeConvertMagData(float magnetism, uint8_t *buffer);


/*******************************************************************************************************************
 * @Brief:Initializes a IIS2MDC Device Handle
 * @Params: Device handle ptr to initialize, Settings that handle will be initialized with, IO Driver with, at a minimum a read, write, and GetTick function
 * @Return: Status Code
 * @Pre Condition: None
 * @Post Condition: Device handle and hardware will be initialized.
 *******************************************************************************************************************/
int32_t IIS2MDC_Init(IIS2MDC_Handle_t *Handle, IIS2MDC_InitStruct_t Settings, IIS2MDC_IO_t *IO)
{
    if(Handle->Status != IIS2MDC_Initialized)
    {
        if(Handle == NULL)
        {
            return IIS2MDC_HandleError;
        }

        if(IO->Read == NULL || IO->Write == NULL || IO->GetTick == NULL) //User MUST provide these functions at a minimum.
        {
        	return IIS2MDC_IOError;
        }

        if(Settings.IntThreshold > 49150.5  || Settings.OffsetX > 49150.5 || Settings.OffsetY > 49150.5  || Settings.OffsetZ > 49150.5)
        {
        	return IIS2MDC_FloatError;
        }

        if(Settings.IntThreshold < -49150.5 || Settings.OffsetX < -49150.5 || Settings.OffsetY < -49150.5  || Settings.OffsetZ < -49150.5)
        {
        	return IIS2MDC_FloatError;
        }

        Handle->IO = *IO;
        Handle->Context.Handle = Handle;
        Handle->Context.Read = IIS2MDC_ReadRegWrapper;
        Handle->Context.Write = IIS2MDC_WriteRegWrapper;
        if((Settings.ConfigRegA & IIS2MDC_ODR_Msk) == IIS2MDC_ODR_OneShot)
        {
        	Handle->DataMode = IIS2MDC_OneShotMode;
        } else
        {
        	Handle->DataMode = IIS2MDC_Continuous;
        }

        Handle->OutputDataRate = Settings.ConfigRegA & IIS2MDC_ODR_Msk;
        int32_t ret = IIS2MDC_RegisterInit(Handle, Settings);
        if(ret != IIS2MDC_Ok)
        {
        	return ret;
        }
        Handle->Status = IIS2MDC_Initialized;
    }
    return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief:Deinitializes a IIS2MDC Device Handle
 * @Params: Pointer to handle to deinitialize.
 * @Return: Status Code
 * @Pre Condition: None
 * @Post Condition: Device handle and hardware will be deinitialized.
 *******************************************************************************************************************/
int32_t IIS2MDC_DeInit(IIS2MDC_Handle_t *Handle)
{
    if(Handle == NULL)
    {
    	return IIS2MDC_HandleError;
    }

    if(Handle->IO.Deinit != NULL)
    {
    	Handle->IO.Deinit();
    }

    Handle->IO.Init = NULL;
    Handle->IO.Deinit = NULL;
    Handle->IO.Write = NULL;
    Handle->IO.Read = NULL;
    Handle->IO.GetTick = NULL;
    Handle->IO.ReadPin = NULL;
    Handle->DataMode = 0;
    Handle->Context.Handle = NULL;
    Handle->Context.Read = NULL;
    Handle->Context.Write = NULL;
    Handle->Status = IIS2MDC_Uninitialized;

    return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Resets an IIS2MDC Device Handle
 * @Params: Pointer to handle to deinitialize.
 * @Return: Status Code
 * @Pre Condition: Device Handle should be initialized.
 * @Post Condition: Device will be power cycled and registers set to factory defaults.
 *******************************************************************************************************************/
int32_t IIS2MDC_ResetDevice(IIS2MDC_Handle_t *Handle)
{
	int32_t ret = IIS2MDC_Ok;
	if(Handle == NULL)
	{
		return IIS2MDC_HandleError;
	}

    if(Handle->IO.Write == NULL || Handle->IO.Read == NULL || Handle->IO.GetTick == NULL)
    {
    	return IIS2MDC_IOError;
    }

	ret = IIS2MDC_Reboot(Handle);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}


	ret = IIS2MDC_SWReset(Handle);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}

	return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Writes to the OffsetX Device register.
 * @Params: Pointer to IIS2MDC device handle, value to write to register.
 * @Return: Status Code
 * @Pre Condition: Device Handle should be initialized.
 * @Post Condition: Device will have it's IIS2MDC_REG_OFFSET_X_REGs written to.
 *******************************************************************************************************************/
int32_t IIS2MDC_SetOffsetMagnetismX(IIS2MDC_Handle_t *Handle, float Offset)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}
	uint32_t ret = 0;
	uint8_t Buffer[2] = {0};
	DeConvertMagData(Offset, Buffer);
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_OFFSET_X_REG_L,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Retrieves the the OffsetX Device register value
 * @Params: Pointer to IIS2MDC device handle, float buffer to store offset value
 * @Return: Status Code
 * @Pre Condition: Device Handle should be initialized.
 * @Post Condition: "Offset" param will contain the value of IIS2MDC_REG_OFFSET_X_REGs
 *******************************************************************************************************************/
int32_t IIS2MDC_GetOffsetMagnetismX(IIS2MDC_Handle_t *Handle, float *Offset)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}
	uint8_t Buffer[2] = {0};
	uint32_t ret = IIS2MDC_ReadReg(&Handle->Context,IIS2MDC_REG_OFFSET_X_REG_L,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	ConvertMagData(Buffer,Offset);
    return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Writes to the OffsetY Device register.
 * @Params: Pointer to IIS2MDC device handle, value to write to register.
 * @Return: Status Code
 * @Pre Condition: Device Handle should be initialized.
 * @Post Condition: Device will have it's IIS2MDC_REG_OFFSET_Y_REGs written to.
 *******************************************************************************************************************/
int32_t IIS2MDC_SetOffsetMagnetismY(IIS2MDC_Handle_t *Handle, float Offset)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}
	uint32_t ret = 0;
	uint8_t Buffer[2] = {0};
	DeConvertMagData(Offset, Buffer);
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_OFFSET_Y_REG_L,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Retrieves the the OffsetY Device register value
 * @Params: Pointer to IIS2MDC device handle, float buffer to store offset value
 * @Return: Status Code
 * @Pre Condition: Device Handle should be initialized.
 * @Post Condition: "Offset" param will contain the value of IIS2MDC_REG_OFFSET_Y_REGs
 *******************************************************************************************************************/
int32_t IIS2MDC_GetOffsetMagnetismY(IIS2MDC_Handle_t *Handle, float *Offset)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}
	uint8_t Buffer[2] = {0};
	uint32_t ret = IIS2MDC_ReadReg(&Handle->Context,IIS2MDC_REG_OFFSET_Y_REG_L,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	ConvertMagData(Buffer,Offset);
    return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Writes to the OffsetZ Device register.
 * @Params: Pointer to IIS2MDC device handle, value to write to register.
 * @Return: Status Code
 * @Pre Condition: Device Handle should be initialized.
 * @Post Condition: Device will have it's IIS2MDC_REG_OFFSET_Z_REGs written to.
 *******************************************************************************************************************/
int32_t IIS2MDC_SetOffsetMagnetismZ(IIS2MDC_Handle_t *Handle, float Offset)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}
	uint32_t ret = 0;
	uint8_t Buffer[2] = {0};
	DeConvertMagData(Offset, Buffer);
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_OFFSET_Z_REG_L,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Retrieves the the OffsetZ Device register value
 * @Params: Pointer to IIS2MDC device handle, float buffer to store offset value
 * @Return: Status Code
 * @Pre Condition: Device Handle should be initialized.
 * @Post Condition: "Offset" param will contain the value of IIS2MDC_REG_OFFSET_Z_REGs
 *******************************************************************************************************************/
int32_t IIS2MDC_GetOffsetMagnetismZ(IIS2MDC_Handle_t *Handle, float *Offset)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}
	uint8_t Buffer[2] = {0};
	uint32_t ret = IIS2MDC_ReadReg(&Handle->Context,IIS2MDC_REG_OFFSET_Z_REG_L,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	ConvertMagData(Buffer,Offset);
    return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Starts a magnetism conversion
 * @Params: pointer to IIS2MDC Handle to begin conversion
 * @Return: Status Code,
 * @Pre Condition: Device Handle should be initialized and configured in One Shot Mode.
 * @Post Condition: IIS2MDC Device will sampling/converting magnetic field.
 *******************************************************************************************************************/
int32_t IIS2MDC_StartConversion(IIS2MDC_Handle_t *Handle)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}

	if(Handle->DataMode != IIS2MDC_OneShotMode)
	{
        return IIS2MDC_OneShotError;
	}

	uint8_t buffer = 0;
	uint32_t ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_CFG_REG_A, &buffer, 1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	buffer |= IIS2MDC_ODR_OneShot;
	ret = IIS2MDC_WriteReg(&Handle->Context, IIS2MDC_REG_CFG_REG_A, &buffer, 1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Reads X, Y, and Z magnetic field values
 * @Params: pointer to IIS2MDC Handle, and buffers for X, Y, and Z values
 * @Return: Status Code, IIS2MDC_DataNotReady if conversion isn't done, IIS2MDC_DataReady on successful read
 * @Pre Condition: Device Handle should be initialized
 * @Post Condition: Mx, My, and Mz params will contain magnetic field values in milligause if read was successful.
 *******************************************************************************************************************/
int32_t IIS2MDC_ReadMagnetismXYZ(IIS2MDC_Handle_t *Handle, float *Mx, float *My, float *Mz)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}

	uint8_t Buffer[6] = {0};
	uint32_t ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_STATUS_REG, Buffer, 1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	if(Buffer[0] & (IIS2MDC_XDATA_Msk | IIS2MDC_YDATA_Msk | IIS2MDC_ZDATA_Msk))
	{
		ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_OUTX_L_REG, Buffer, 6);
		if(ret != IIS2MDC_Ok)
		{
			return ret;
		}
		ConvertMagData(Buffer, Mx);
		ConvertMagData(&Buffer[2], My);
		ConvertMagData(&Buffer[4], Mz);
		return IIS2MDC_DataReady;
	}
	return IIS2MDC_DataNotReady;
}

/*******************************************************************************************************************
 * @Brief: Reads X magnetic field value
 * @Params: pointer to IIS2MDC Handle, and buffer for X value
 * @Return: Status Code, IIS2MDC_DataNotReady if conversion isn't done, IIS2MDC_DataReady on successful read
 * @Pre Condition: Device Handle should be initialized
 * @Post Condition: Mx param will contain magnetic field value in milligause if read was successful.
 *******************************************************************************************************************/
int32_t IIS2MDC_ReadMagnetismX(IIS2MDC_Handle_t *Handle, float *Mx)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}

	uint8_t Buffer[2] = {0};
	uint32_t ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_STATUS_REG, Buffer, 1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	if(Buffer[0] & IIS2MDC_XDATA_Msk)
	{
		ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_OUTX_L_REG, Buffer, 2);
		if(ret != IIS2MDC_Ok)
		{
			return ret;
		}
		ConvertMagData(Buffer, Mx);
		return IIS2MDC_DataReady;
	}
	return IIS2MDC_DataNotReady;
}

/*******************************************************************************************************************
 * @Brief: Reads Y magnetic field value
 * @Params: pointer to IIS2MDC Handle, and buffer for Y value
 * @Return: Status Code, IIS2MDC_DataNotReady if conversion isn't done, IIS2MDC_DataReady on successful read
 * @Pre Condition: Device Handle should be initialized
 * @Post Condition: My param will contain magnetic field value in milligause if read was successful.
 *******************************************************************************************************************/
int32_t IIS2MDC_ReadMagnetismY(IIS2MDC_Handle_t *Handle, float *My)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}

	uint8_t Buffer[2] = {0};
	uint32_t ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_STATUS_REG, Buffer, 1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	if(Buffer[0] & IIS2MDC_YDATA_Msk)
	{
		ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_OUTY_L_REG, Buffer, 2);
		if(ret != IIS2MDC_Ok)
		{
			return ret;
		}
		ConvertMagData(Buffer, My);
		return IIS2MDC_DataReady;
	}
	return IIS2MDC_DataNotReady;
}

/*******************************************************************************************************************
 * @Brief: Reads Z magnetic field value
 * @Params: pointer to IIS2MDC Handle, and buffer for Z value
 * @Return: Status Code, IIS2MDC_DataNotReady if conversion isn't done, IIS2MDC_DataReady on successful read
 * @Pre Condition: Device Handle should be initialized
 * @Post Condition: Mz param will contain magnetic field value in milligause if read was successful.
 *******************************************************************************************************************/
int32_t IIS2MDC_ReadMagnetismZ(IIS2MDC_Handle_t *Handle, float *Mz)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}

	uint8_t Buffer[2] = {0};
	uint32_t ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_STATUS_REG, Buffer, 1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}
	if(Buffer[0] & IIS2MDC_ZDATA_Msk)
	{
		ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_OUTZ_L_REG, Buffer, 2);
		if(ret != IIS2MDC_Ok)
		{
			return ret;
		}
		ConvertMagData(Buffer, Mz);
		return IIS2MDC_DataReady;
	}
	return IIS2MDC_DataNotReady;
}

/*******************************************************************************************************************
 * @Brief: Calls Low Level IO Driver to Write to device registers.
 * @Params: pointer to IIS2MDC Handle, Reg to write to, Buffer with data to write, number of bytes to write.
 * @Return: Status Code
 * @Pre Condition: Device Handle should be initialized
 * @Post Condition: Device registers will be written to/modified.
 *******************************************************************************************************************/
static int32_t IIS2MDC_WriteRegWrapper(void *Handle,uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	IIS2MDC_Handle_t *Dev= Handle;
	if(Dev->IO.Write != NULL)
	{
		return Dev->IO.Write(Reg,Buffer,Length);
	}
    return IIS2MDC_IOError;
}

/*******************************************************************************************************************
 * @Brief: Calls Low Level IO Driver to read from device registers.
 * @Params: pointer to IIS2MDC Handle, Reg to read from, Buffer to hold read data, number of bytes to read.
 * @Return: Status Code
 * @Pre Condition: Device Handle should be initialized
 * @Post Condition: "Buffer" param will contain the read data.
 *******************************************************************************************************************/
static int32_t IIS2MDC_ReadRegWrapper(void *Handle,uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	IIS2MDC_Handle_t *Dev= Handle;
	if(Dev->IO.Read != NULL)
	{
		return Dev->IO.Read(Reg,Buffer,Length);
	}
    return IIS2MDC_IOError;
}

/*******************************************************************************************************************
 * @Brief: Initializes Device handle and hardware according to provided settings. Called from IIS2MDC Init function.
 * @Params: pointer to IIS2MDC Handle, desired user settings.
 * @Return: Status Code
 * @Pre Condition: None
 * @Post Condition: Device will be reset and registers will be written to according to Settings param.
 *******************************************************************************************************************/
static int32_t IIS2MDC_RegisterInit(IIS2MDC_Handle_t *Handle, IIS2MDC_InitStruct_t Settings)
{
	int32_t ret = IIS2MDC_ResetDevice(Handle);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}

	uint8_t Buffer[2] = {0};

	DeConvertMagData(Settings.IntThreshold, Buffer);
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_INT_THS_H_REG,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}


	DeConvertMagData(Settings.OffsetX, Buffer);
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_OFFSET_X_REG_L,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}

	DeConvertMagData(Settings.OffsetY, Buffer);
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_OFFSET_Y_REG_L,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}

	DeConvertMagData(Settings.OffsetZ, Buffer);
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_OFFSET_Z_REG_L,Buffer,2);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}

	Buffer[0] = Settings.ConfigRegA;
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_CFG_REG_A,Buffer,1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}

	Buffer[0] = Settings.ConfigRegB;
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_CFG_REG_B,Buffer,1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}

	Buffer[0] = Settings.ConfigRegC;
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_CFG_REG_C,Buffer,1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}

	Buffer[0] = Settings.IntCtrlReg;
	ret = IIS2MDC_WriteReg(&Handle->Context,IIS2MDC_REG_INT_CTRL_REG,Buffer,1);
	if(ret != IIS2MDC_Ok)
	{
		return ret;
	}

    return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Resets IIS2MDC Registers to factory default.
 * @Params: pointer to IIS2MDC Handle
 * @Return: Status Code
 * @Pre Condition: IO Should be initialized.
 * @Post Condition: Device registers will be set to factory default.
 *******************************************************************************************************************/
static int32_t IIS2MDC_SWReset(IIS2MDC_Handle_t *Handle)
{
    int32_t ret = 0;
    uint8_t Buffer = IIS2MDC_ResetSignal;
    ret = IIS2MDC_WriteReg(&Handle->Context, IIS2MDC_REG_CFG_REG_A, &Buffer, 1);
    if(ret != IIS2MDC_Ok)
    {
    	return ret;
    }

    uint32_t timeout = Handle->IO.GetTick();
    while(Buffer & IIS2MDC_ResetSignal)
    {
        ret = IIS2MDC_ReadReg(&Handle->Context, IIS2MDC_REG_CFG_REG_A, &Buffer, 1);
        if(Handle->IO.GetTick() > timeout + 100)
        {
        	return IIS2MDC_Timeout;
        }
    }
    return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Reboots IIS2MDC Device
 * @Params: pointer to IIS2MDC Handle
 * @Return: Status Code
 * @Pre Condition: IO Should be initialized.
 * @Post Condition: Device will be power cycled.
 *******************************************************************************************************************/
static int32_t IIS2MDC_Reboot(IIS2MDC_Handle_t *Handle)
{
    int32_t ret = 0;
    uint8_t Buffer = IIS2MDC_RebootSignal;
    ret = IIS2MDC_WriteReg(&Handle->Context, IIS2MDC_REG_CFG_REG_A, &Buffer, 1);
    if(ret != IIS2MDC_Ok)
    {
    	return ret;
    }
    uint32_t timeout = Handle->IO.GetTick();
    while((timeout + 20) > Handle->IO.GetTick()); //Delay 20ms
    return IIS2MDC_Ok;
}

/*******************************************************************************************************************
 * @Brief: Helper function that converts magnetic data read from device
 * @Params: pointer to buffer with data read from device. buffer to store converted value
 * @Return: Status Code
 * @Pre Condition: None
 * @Post Condition: None
 *******************************************************************************************************************/
static void ConvertMagData(uint8_t *buffer, float *magnetism)
{
	int16_t result = (buffer[1] * 256) + buffer[0];
	*magnetism = result * 1.5f;
}

/*******************************************************************************************************************
 * @Brief: Converts a B-field value in milligause to int16 so that it can be written to device registers.
 * @Params: pointer b-field value, buffer to store de-converted value
 * @Return: Status Code
 * @Pre Condition: None
 * @Post Condition: None
 *******************************************************************************************************************/
static void DeConvertMagData(float magnetism, uint8_t *buffer)
{
    int16_t result = magnetism / 1.5f;
    buffer[1] = (result & 0xFF00) >> 8;
    buffer[0] = result & 0x00FF;
}

/*******************************************************************************************************************
 * @Brief: Fetches sample period based on device sample frequency.
 * @Params: Device handle, Buffer to store Period
 * @Return: Status Code, IIS2MDC_ODR_PeriodOneShot if device in one-show mode.
 * @Pre Condition: Device should be initialized
 * @Post Condition: "Period" param will contain sample period in ms.
 *******************************************************************************************************************/
int32_t IIS2MDC_GetSamplePeriod(IIS2MDC_Handle_t *Handle, uint32_t *Period)
{
	if(Handle->Status != IIS2MDC_Initialized)
	{
		return IIS2MDC_InitError;
	}

	switch(Handle->OutputDataRate)
	{
	case(IIS2MDC_ODR_OneShot):
			*Period = 0;
	        return IIS2MDC_ODR_PeriodOneShot;
	case(IIS2MDC_ODR_10Hz):
			*Period =  (1000 / 10);
	        break;
	case(IIS2MDC_ODR_20Hz):
			*Period = (1000 / 20);
	        break;
	case(IIS2MDC_ODR_50Hz):
			*Period = (1000 / 50);
	        break;
	case(IIS2MDC_ODR_100Hz):
			*Period = (1000 / 100);
	        break;
	default:
		    return IIS2MDC_ODR_Error;
	}

	return IIS2MDC_Ok;
}
