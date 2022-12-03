/*
 * IIS2MDC_Registers.c
 *
 *  Created on: Oct 22, 2022
 *      Author: evanl
 */

#include "IIS2MDC_Registers.h"
#include <stddef.h>

/******************************************************************************************
 * @Brief: Checks the parameters passed to Read/Write reg functions below.
 * @Param: Context pointer, Reg to read, Buffer, Buffer Length
 * @Return: IIS2MDC_REG_OK if params are OK, Error Code if theres a problem.
 * @Pre Condition: None
 * @Post Condition: None
 ******************************************************************************************/
static int32_t IIS2MDC_ValidateParams(IIS2MDC_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	int32_t ret = IIS2MDC_REG_OK;
    if(Context == NULL || Context->Read == NULL || Context->Write == NULL || Context->Handle == NULL)
    {
    	ret |= IIS2MDC_REG_CONTEXT_ERR;
    }

    if(Reg < IIS2MDC_REG_OFFSET_X_REG_L || Reg > IIS2MDC_REG_TEMP_OUT_H_REG)
    {
    	ret |= IIS2MDC_REG_ADDR_ERR;
    }

    if(Buffer == NULL)
    {
    	ret |= IIS2MDC_REG_BUFFER_ERR;
    }

    if(Length == 0)
    {
       ret |= IIS2MDC_REG_LENGTH_ERR;
    }

    return ret;

}

/******************************************************************************************
 * @Brief: Calls context function to read IIS2MDC Register
 * @Param: Context pointer, Reg to read, Buffer to store data, number of bytes of read
 * @Return: Status code
 * @Pre Condition: Low level IO should be initialized
 * @Post Condition: IIS2MDC device will be read and buffer will contain the data.
 ******************************************************************************************/
int32_t IIS2MDC_ReadReg(IIS2MDC_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	int32_t ret = IIS2MDC_ValidateParams(Context,Reg,Buffer,Length);
	if(ret != IIS2MDC_REG_OK)
	{
		return ret;
	}

	ret = Context->Read(Context->Handle, Reg, Buffer, Length);
	return ret;
}

/******************************************************************************************
 * @Brief: Calls context function to write IIS2MDC Registers
 * @Param: Context pointer, Reg to write, Buffer of data to write, number of bytes of write
 * @Return: Status code
 * @Pre Condition: Low level IO should be initialized
 * @Post Condition: Buffer data will be written to given IIS2MDC device register(s)
 ******************************************************************************************/
int32_t IIS2MDC_WriteReg(IIS2MDC_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	int32_t ret = IIS2MDC_ValidateParams(Context,Reg,Buffer,Length);
	if(ret != IIS2MDC_REG_OK)
	{
		return ret;
	}

	ret = Context->Write(Context->Handle, Reg, Buffer, Length);
	return ret;
}
