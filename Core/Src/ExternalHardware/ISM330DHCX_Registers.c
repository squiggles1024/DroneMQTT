/*
 * ISM330DHCX_Registers.c
 *
 *  Created on: Oct 4, 2022
 *      Author: evanl
 */
/* Includes */
#include "ISM330DHCX_Registers.h"
#include <stdint.h>
#include <stddef.h>

/* Private function prototype */
static int32_t ISM330DHCX_ValidateParams(uint8_t Reg, uint8_t *Buffer, uint8_t Length);

/*********************************************************************************************************
 * @Brief: Function calls the ReadWrapper function in ISM330DHCX.c which calls low level IO Read function.
 * @Params: Context pointer, Reg to read from, Buffer to store bytes, number of bytes to read.
 * @Return: Status code
 * @Pre Condition: Low level IO must be initialized.
 * @Post Condition: ReadWrapper function will be called and reads from device.
 *********************************************************************************************************/
int32_t ISM330DHCX_ReadReg(ISM330DHCX_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	if(Context->Read == NULL)
	{
		return ISM330DHCX_REG_CONTEXT_ERR;
	}

	int32_t ret = ISM330DHCX_ValidateParams(Reg,Buffer,Length);
	if(ret != ISM330DHCX_REG_NO_ERR)
	{
        return ret;
	}

	return Context->Read(Context->Handle, Reg, Buffer, Length);
}

/*********************************************************************************************************
 * @Brief: Function calls the WriteWrapper function in ISM330DHCX.c which calls low level IO Read function.
 * @Params: Context pointer, Reg to write to, Buffer with data to write, number of bytes to write.
 * @Return: Status code
 * @Pre Condition: Low level IO must be initialized.
 * @Post Condition: WriteWrapper function will be called and writes to device registers.
 *********************************************************************************************************/
int32_t ISM330DHCX_WriteReg(ISM330DHCX_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	if(Context->Write == NULL)
	{
		return ISM330DHCX_REG_CONTEXT_ERR;
	}

	int32_t ret = ISM330DHCX_ValidateParams(Reg,Buffer,Length);
	if(ret != ISM330DHCX_REG_NO_ERR)
	{
        return ret;
	}

	return Context->Write(Context->Handle, Reg, Buffer, Length);
}

/*********************************************************************************************************
 * @Brief: Helper function that checks the arguments passed to above functions.
 * @Params: Reg to read/write, write/read buffer, number of bytes to read/write.
 * @Return: Status code
 * @Pre Condition: None
 * @Post Condition: None
 *********************************************************************************************************/
static int32_t ISM330DHCX_ValidateParams(uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	int32_t ret = 0;
	if(Reg < ISM330DHCX_REG_FUNC_CFG_ACCESS || Reg > ISM330DHCX_REG_FIFO_OUT_Z_H)
	{
		ret |= ISM330DHCX_REG_PARAM_ERR;
	}

	if(Buffer == NULL)
	{
        ret |= ISM330DHCX_BUFFER_PARAM_ERR;
	}

	if( Length == 0 || Length > (ISM330DHCX_REG_FIFO_OUT_Z_H - ISM330DHCX_REG_FUNC_CFG_ACCESS))
	{
		ret |= ISM330DHCX_LENGTH_PARAM_ERR;
	}

	return ret;
}

