/*
 * VEML6030_Registers.c
 *
 *  Created on: Oct 27, 2022
 *      Author: evanl
 */
#include "VEML6030_Registers.h"
#include <stdint.h>
#include <stddef.h>

/**************************************************************************************************************
 * @Brief: Function checks the parameters passed to the Read/Write reg functions
 * @Params: Params passed to below Read/Write reg functions
 * @Return: Status code
 * @Pre Condition: None
 * @Post Condition: None
 **************************************************************************************************************/
static int32_t VEML6030_ValidateParams(VEML6030_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	int32_t ret = VEML6030_REG_OK;
    if(Context == NULL || Context->Read == NULL || Context->Write == NULL || Context->Handle == NULL)
    {
    	ret |= VEML6030_REG_CONTEXT_ERR;
    }

    if(Reg > VEML6030_REG_ALS_INT)
    {
    	ret |= VEML6030_REG_ADDR_ERR;
    }

    if(Buffer == NULL)
    {
    	ret |= VEML6030_REG_BUFFER_ERR;
    }

    if(Length == 0)
    {
       ret |= VEML6030_REG_LENGTH_ERR;
    }

    return ret;
}

/**************************************************************************************************************
 * @Brief: Calls Read wrapper function in VEML6030.c which will call the low level IO function to read a reg.
 * @Params: Context ptr, Register to read, Buffer to store data, number of bytes to read
 * @Return: Status code
 * @Pre Condition: VEML6030 Handle in which the context param resides should be initialized.
 * @Post Condition: ReadReg wrapper will be called and "Buffer" will be given the read data.
 **************************************************************************************************************/
int32_t VEML6030_ReadReg(VEML6030_Context_t *Context,uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
    int32_t ret = VEML6030_ValidateParams(Context,Reg,Buffer,Length);
    if(ret != VEML6030_REG_OK)
    {
    	return ret;
    }
    ret = Context->Read(Context->Handle,Reg,Buffer,Length);
    return ret;
}

/**************************************************************************************************************
 * @Brief: Calls Write wrapper function in VEML6030.c which will call the low level IO function to write a reg.
 * @Params: Context ptr, Register to write, Buffer with data to write, number of bytes to write
 * @Return: Status code
 * @Pre Condition: VEML6030 Handle in which the context param resides should be initialized.
 * @Post Condition: WriteReg wrapper will be called and "Buffer" will be written into device registers
 **************************************************************************************************************/
int32_t VEML6030_WriteReg(VEML6030_Context_t *Context,uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
    int32_t ret = VEML6030_ValidateParams(Context,Reg,Buffer,Length);
    if(ret != VEML6030_REG_OK)
    {
    	return ret;
    }
    ret = Context->Write(Context->Handle,Reg,Buffer,Length);
    return ret;
}
