/*
 * HTS221.c
 *
 *  Created on: Sep 15, 2022
 *      Author: evanl
 */
/* Includes */
#include "HTS221.h"
#include <stdint.h>
#include <stddef.h>

/* Private Helper functions */
static int32_t HTS221_ReadRegWrapper(void *Handle, uint8_t Reg, uint8_t *Buffer, uint8_t Length);
static int32_t HTS221_WriteRegWrapper(void *Handle, uint8_t Reg, uint8_t *Buffer, uint8_t Length);
static int32_t HTS221_RegisterInit(HTS221_Handle_t *Handle, HTS221_Init_Struct_t Settings);
static int32_t HTS221_CalculateTemperature(HTS221_Handle_t *Handle, float *temperature);
static int32_t HTS221_CalculateHumidity(HTS221_Handle_t *Handle, float *humidity);

/****************************************************************************************************************************
 * @Brief: Initializes HTS221 Handle and Hardware
 * @Params: Pointer to Handle to initialize, the desired hardware settings, structure with low level IO funtionality
 * @Return: Status code
 * @Pre Condition: None
 * @Post Condition: HTS221 Handle will be initialized and ready to use.
 ****************************************************************************************************************************/
int32_t HTS221_Init(HTS221_Handle_t *Handle, HTS221_Init_Struct_t Settings, HTS221_IO_t *IO)
{
    if(Handle == NULL)
    {
    	return HTS221_Error;
    }
    Handle->IO = *IO;
    Handle->Context.Handle = Handle;
    Handle->Context.Read = HTS221_ReadRegWrapper;
    Handle->Context.Write = HTS221_WriteRegWrapper;

    if(Settings.DataReadyConfig == HTS221_DataReadyEnable && Handle->IO.ReadPin == NULL)
    {
    	return HTS221_Error; //Must provide a read pin function if using the data ready pin
    }
    Handle->DRDY_Config = Settings.DataReadyConfig;
    Handle->DRDY_Polarity = Settings.DataReadySignalPolarity;

    //Initialized Bus
    if(Handle->IO.Init != NULL)
    {
        Handle->IO.Init();
    }

    Handle->OutputDataRate = Settings.OutputDataRate;

    //Initialize Device
    return HTS221_RegisterInit(Handle, Settings);
}

/****************************************************************************************************************************
 * @Brief: Deinitializes HTS221 Handle and Hardware (if DeInit function is provided by user)
 * @Params: Pointer to Handle to deinitialize.
 * @Return: None
 * @Pre Condition: None
 * @Post Condition: HTS221 Handle will be returned to pre-init state.
 ****************************************************************************************************************************/
void HTS221_DeInit(HTS221_Handle_t *Handle)
{
    if(Handle->IO.DeInit != NULL){
    	Handle->IO.DeInit();
    }
    Handle->IO.DeInit = NULL;
    Handle->IO.GetTick = NULL;
    Handle->IO.Init = NULL;
    Handle->IO.Write = NULL;
    Handle->IO.Read = NULL;
    Handle->IO.ReadPin = NULL;
    Handle->Context.Read = NULL;
    Handle->Context.Write = NULL;
    Handle->Context.Handle = NULL;
    Handle->DRDY_Config = 0;
    Handle->DRDY_Polarity = 0;
}

/****************************************************************************************************************************
 * @Brief: Starts a conversion for a HTS221 handle configured in 1 shot mode
 * @Params: HTS221 Handle pointer
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized and configured in one-shot-mode.
 * @Post Condition: HTS221 device will begin a temperature conversion
 ****************************************************************************************************************************/
int32_t HTS221_StartConversion(HTS221_Handle_t *Handle)
{
	uint8_t temp = 0;
	int32_t ret = 0;
	ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_CTRL_REG2, &temp, 1);  //Read Reg
	if(ret != 0)
	{
		return ret;
	}
	temp |= HTS221_START_CONV_SIGNAL;                                        //Modify Reg
	return HTS221_WriteReg(&Handle->Context,HTS221_REG_CTRL_REG2, &temp, 1); //Store Reg
}

/****************************************************************************************************************************
 * @Brief: Turns on HTS221 Heater (used to remove condensation)
 * @Params: HTS221 Handle pointer
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: HTS221 device will turn on its heater.
 ****************************************************************************************************************************/
int32_t HTS221_EnableHeater(HTS221_Handle_t *Handle)
{
	uint8_t temp = 0;
	int32_t ret = 0;
	ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_CTRL_REG2, &temp, 1);  //Read Reg
	if(ret != 0)
	{
		return ret;
	}

	temp |= HTS221_HEATER_ENABLE;                                            //Modify Reg
	return HTS221_WriteReg(&Handle->Context,HTS221_REG_CTRL_REG2, &temp, 1); //Store Reg

}

/****************************************************************************************************************************
 * @Brief: Turns off HTS221 Heater (used to remove condensation)
 * @Params: HTS221 Handle pointer
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: HTS221 device will turn off its heater.
 ****************************************************************************************************************************/
int32_t HTS221_DisableHeater(HTS221_Handle_t *Handle)
{
	uint8_t temp = 0;
	int32_t ret = 0;
	ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_CTRL_REG2, &temp, 1);  //Read Reg
	if(ret != 0)
	{
		return ret;
	}

	temp &= ~HTS221_HEATER_ENABLE;                                           //Modify Reg
	return HTS221_WriteReg(&Handle->Context,HTS221_REG_CTRL_REG2, &temp, 1); //Store Reg
}

/****************************************************************************************************************************
 * @Brief: Resets registers of HTS221 device to default state
 * @Params: HTS221 Handle pointer
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: HTS221 device registers will be returned to factory default
 ****************************************************************************************************************************/
int32_t HTS221_ResetRegisters(HTS221_Handle_t *Handle)
{
	uint8_t temp = HTS221_RebootSignal;
    return HTS221_WriteReg(&Handle->Context, HTS221_REG_CTRL_REG2, &temp, 1);
}

/****************************************************************************************************************************
 * @Brief: Power cycles HTS221 device
 * @Params: HTS221 Handle pointer
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: HTS221 device will be power cycled
 ****************************************************************************************************************************/
int32_t HTS221_PowerCycle(HTS221_Handle_t *Handle)
{

    //turn off
	int32_t ret = HTS221_PowerDown(Handle);
	if(ret != HTS221_Ok)
	{
        return ret;
	}
	//delay for 1 timeslice
	uint32_t CurrentTime = Handle->IO.GetTick();
	while(Handle->IO.GetTick() <= (CurrentTime + 1));
	//turn on
	return HTS221_PowerUp(Handle);
}

/****************************************************************************************************************************
 * @Brief: Reads temperature from HTS221 device
 * @Params: HTS221 Handle pointer, float buffer to store value
 * @Return: HTS221 Status code, HTS221_DataNotReady if a new sample isnt available, HTS221_DataReady if a new sample is read.
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: "Temperature" param will store new temperature value if read was successful.
 ****************************************************************************************************************************/
int32_t HTS221_ReadTemperature(HTS221_Handle_t *Handle, float *temperature)
{
    int32_t ret = 0;
    uint8_t reg = 0;
	//Check if DRDY is used
	if(Handle->DRDY_Config == HTS221_DataReadyEnable)
	{
		//If its used, read it.
		int32_t PinState = Handle->IO.ReadPin();
		//If Its High and Polarity is Active Low OR If its Low and Polarity is Active High, return data not ready
        if((Handle->DRDY_Polarity == HTS221_DataActiveHigh && PinState == 0) || (Handle->DRDY_Polarity == HTS221_DataActiveLow && PinState != 0))
        {
            return HTS221_DataNotReady;
        }
	} else
	{
		//Otherwise, read the status reg
        ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_STATUS_REG, &reg, 1);
        if(ret != HTS221_Ok)
        {
        	return ret;
        } else if ((reg & HTS221_TEMPDATA_Msk) == 0)
        {
        	return HTS221_DataNotReady;
        }
	}
    return HTS221_CalculateTemperature(Handle, temperature);
}

/****************************************************************************************************************************
 * @Brief: Reads humidity from HTS221 device
 * @Params: HTS221 Handle pointer, float buffer to store value
 * @Return: HTS221 Status code, HTS221_DataNotReady if a new sample isnt available, HTS221_DataReady if a new sample is read.
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: "humidity" param will store new temperature value if read was successful.
 ****************************************************************************************************************************/
int32_t HTS221_ReadHumidity(HTS221_Handle_t *Handle, float *humidity)
{
    int32_t ret = 0;
    uint8_t reg = 0;
	//Check if DRDY is used
	if(Handle->DRDY_Config == HTS221_DataReadyEnable)
	{
		//If its used, read it.
		int32_t PinState = Handle->IO.ReadPin();
		//If Its High and Polarity is Active Low OR If its Low and Polarity is Active High, return data not ready
        if((Handle->DRDY_Polarity == HTS221_DataActiveHigh && PinState == 0) || (Handle->DRDY_Polarity == HTS221_DataActiveLow && PinState != 0))
        {
            return HTS221_DataNotReady;
        }
	} else
	{
		//Otherwise, read the status reg
        ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_STATUS_REG, &reg, 1);
        if(ret != HTS221_Ok)
        {
        	return ret;
        } else if ((reg & HTS221_HUMDATA_Msk) == 0)
        {
        	return HTS221_DataNotReady;
        }
	}

	return HTS221_CalculateHumidity(Handle, humidity);
}

/****************************************************************************************************************************
 * @Brief: Place HTS221 Device in power down mode
 * @Params: HTS221 Handle pointer
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: Device will be in power down mode.
 ****************************************************************************************************************************/
int32_t HTS221_PowerDown(HTS221_Handle_t *Handle)
{
	uint8_t temp = 0;
	int32_t ret = 0;
	ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_CTRL_REG1, &temp, 1);  //Read Reg
	if(ret != 0)
	{
		return ret;
	}

	temp &= ~HTS221_POWERON_SIGNAL;                                          //Modify Reg
	return HTS221_WriteReg(&Handle->Context,HTS221_REG_CTRL_REG1, &temp, 1); //Store Reg
}

/****************************************************************************************************************************
 * @Brief: Turns on HTS221 device that is in power down mode.
 * @Params: HTS221 Handle pointer
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: Device will be turned on again.
 ****************************************************************************************************************************/
int32_t HTS221_PowerUp(HTS221_Handle_t *Handle)
{
	uint8_t temp = 0;
	int32_t ret = 0;
	ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_CTRL_REG1, &temp, 1);  //Read Reg
	if(ret != 0)
	{
		return ret;
	}

	temp |= HTS221_POWERON_SIGNAL;                                           //Modify Reg
	return HTS221_WriteReg(&Handle->Context,HTS221_REG_CTRL_REG1, &temp, 1); //Store Reg
}

/****************************************************************************************************************************
 * @Brief: Calls Low level IO function to read from HTS221 device
 * @Params: address of HTS221 Handle, Register to read, Buffer to read into, Number of bytes to read.
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: Device register will contain new value
 ****************************************************************************************************************************/
static int32_t HTS221_ReadRegWrapper(void *Handle, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	HTS221_Handle_t *HTS221_Handle = Handle;
	return HTS221_Handle->IO.Read(Reg, Buffer, Length);
}

/****************************************************************************************************************************
 * @Brief: Calls Low level IO function to write to HTS221 device
 * @Params: address of HTS221 Handle, Register to write, Buffer with data to write, Number of bytes to write.
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: "Buffer" will contain read data
 ****************************************************************************************************************************/
static int32_t HTS221_WriteRegWrapper(void *Handle, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	HTS221_Handle_t *HTS221_Handle = Handle;
	return HTS221_Handle->IO.Write(Reg, Buffer, Length);
}

/****************************************************************************************************************************
 * @Brief: Initializes HTS221 device registers according to given settings
 * @Params: HTS221 Handle pointer, Init struct with desired settings.
 * @Return: HTS221 Status code
 * @Pre Condition: None
 * @Post Condition: Device will have its registers configured according to "Settings"
 ****************************************************************************************************************************/
static int32_t HTS221_RegisterInit(HTS221_Handle_t *Handle, HTS221_Init_Struct_t Settings)
{
    int32_t ret =  0;
    uint8_t temp = 0;

    //reset registers to factory default
    ret = HTS221_ResetRegisters(Handle);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }

    temp = HTS221_BDU_ENABLE | Settings.OutputDataRate;
    ret = HTS221_WriteReg(&Handle->Context, HTS221_REG_CTRL_REG1, &temp,1);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }

    temp = Settings.DataReadySignalPolarity | Settings.DataReadyOutputMode | Settings.DataReadyConfig;
    return HTS221_WriteReg(&Handle->Context, HTS221_REG_CTRL_REG3, &temp,1);
}

/****************************************************************************************************************************
 * @Brief: Performs HTS221 temperature calulation
 * @Params: HTS221 Handle pointer, float buffer to store calculated value
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: "temperature" will contain the calculated value
 * Comments: See HTS221 App note for calculation procedure
 ****************************************************************************************************************************/
static int32_t HTS221_CalculateTemperature(HTS221_Handle_t *Handle, float *temperature)
{
	int32_t ret = 0;
	uint8_t Buffer[2] = {0};
	int16_t temp = 0;
    float T0_OUT = 0, T1_OUT = 0;
    float T1_DEGC = 0,  T0_DEGC = 0;


    //Fetch T0
    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_T0_OUT, Buffer, 2);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    temp = (Buffer[1] << 8) + Buffer[0];
    T0_OUT = temp * 1.0f;

    //Fetch T1
    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_T1_OUT, Buffer, 2);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    temp = (Buffer[1] << 8) + Buffer[0];
    T1_OUT = temp * 1.0f;

    //Fetch T0DegC
    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_T0_degC_x8, Buffer, 1); //Get LSB, store in Buffer16
    if(ret != HTS221_Ok)
    {
    	return ret;
    }

    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_T1_T0_msb, &Buffer[1], 1);  //Get MSB, store in Buffer8
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    temp = ((Buffer[1] & 0x03) << 8) + Buffer[0];
    T0_DEGC = temp / 8.0f; //Calculate

    //Fetch T1DegC
    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_T1_degC_x8, Buffer, 1); //Get LSB, store in Buffer16
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    temp = ((Buffer[1] & 0x0C) << 6) + Buffer[0];
    T1_DEGC = temp / 8.0f; //Calculate

    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_TEMP_OUT_L, Buffer, 2);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    temp = (Buffer[1] << 8) + Buffer[0];
    *temperature = (((T1_DEGC - T0_DEGC) * temp) + ((T1_OUT * T0_DEGC) - (T0_OUT * T1_DEGC))) / (T1_OUT - T0_OUT); //Linear Interpolation
    return HTS221_Ok;
}

/****************************************************************************************************************************
 * @Brief: Performs HTS221 humidity calulation
 * @Params: HTS221 Handle pointer, float buffer to store calculated value
 * @Return: HTS221 Status code
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: "humidity" will contain the calculated value
 * Comments: See HTS221 App note for calculation procedure
 ****************************************************************************************************************************/
static int32_t HTS221_CalculateHumidity(HTS221_Handle_t *Handle, float *humidity)
{
	int32_t ret = 0;
	uint8_t Buffer[2] = {0};
	int16_t temp = 0;
    float H0_OUT = 0, H1_OUT = 0;
    float H1_REL = 0,  H0_REL = 0;
    float result = 0;

    //Fetch H0
    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_H0_T0_OUT, Buffer, 2);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    temp = (Buffer[1] << 8) + Buffer[0];
    H0_OUT = temp * 1.0f;

    //Fetch H1
    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_H1_T0_OUT, Buffer, 2);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    temp = (Buffer[1] << 8) + Buffer[0];
    H1_OUT = temp * 1.0f;

    //Fetch H0_Rh
    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_H0_rH_x2, Buffer, 1);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    H0_REL = Buffer[0] / 2.0f;

    //Fetch H0_Rh
    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_H1_rH_x2, Buffer, 1);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    H1_REL = Buffer[0] / 2.0f;

    //Fetch Measurement
    ret = HTS221_ReadReg(&Handle->Context, HTS221_REG_HUMIDITY_OUT_L, Buffer, 2);
    if(ret != HTS221_Ok)
    {
    	return ret;
    }
    temp = (Buffer[1] << 8) + Buffer[0];
    result = (((H1_REL - H0_REL) * temp) + ((H1_OUT * H0_REL) - (H0_OUT * H1_REL))) / (H1_OUT - H0_OUT); //Linear Interpolation
    if(result > 100.0)
    {
    	*humidity = 100.0;
    } else if (result < 0.0)
    {
    	*humidity = 0.0;
    } else
    {
    	*humidity = result;
    }

    return HTS221_Ok;
}

/****************************************************************************************************************************
 * @Brief: Returns period in ms based on HTS221 handle frequency
 * @Params: HTS221 Handle pointer, uint32 buffer to store period
 * @Return: HTS221 Status code, HTS221_ODR_Error is its configured in one shot mode.
 * @Pre Condition: HTS221 handle must be initialized
 * @Post Condition: "period" will contain the period in ms
 ****************************************************************************************************************************/
int32_t HTS221_GetSamplePeriod(HTS221_Handle_t *Handle, uint32_t *Period)
{
	switch(Handle->OutputDataRate)
	{
	case(HTS221_OneShotMode):
			*Period = 0;
	        return HTS221_PeriodOneShot;
			break;
	case(HTS221_1_0Hz):
			*Period = 1000;
			break;
	case(HTS221_7_0p0Hz):
			*Period = 143;
	        break;
	case(HTS221_12_5Hz):
			*Period = 80;
	        break;
	default:
		   return HTS221_ODR_Error;
	}
	return HTS221_Ok;
}
