/*
 * HTS221_Register.c
 *
 *  Created on: Oct 3, 2022
 *      Author: evanl
 */
/* Includes */
#include "HTS221_Registers.h"
#include "stddef.h"

/* Private Function Prototype */
static int32_t HTS221_ValidateParams(uint8_t Reg, uint8_t *Buffer, uint8_t Length);

/***************************************************************************************************
 * @Brief: Reads from an HTS221 Register
 * @Param: Initialzed Context pointer, Register to read, Buffer to store data, qty of bytes to read.
 * @Return: HTS221 Status or HAL Status depending on where error occured.
 * @Post Condition: Buffer will contain returned data if read was successful.
 * @Pre Condition: IO Bus and context must be initialized.
 ***************************************************************************************************/
int32_t HTS221_ReadReg(HTS221_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	int32_t ret = HTS221_ValidateParams(Reg,Buffer,Length);
	if(ret != HTS221_REG_NO_ERR)
	{
        return ret;
	}

    if(Context->Read != NULL)
    {
        return Context->Read(Context->Handle, Reg, Buffer, Length);
    }

    return HTS221_REG_CONTEXT_ERR;
}

/***************************************************************************************************
 * @Brief: Writes to a HTS221 Register
 * @Param: Initialzed Context pointer, Register to write, Buffer of data to write, qty of bytes to write.
 * @Return: HTS221 Status or HAL Status depending on where error occured.
 * @Post Condition: HTS221 Register will be written to.
 * @Pre Condition: IO Bus and context must be initialized.
 ***************************************************************************************************/
int32_t HTS221_WriteReg(HTS221_Context_t *Context,uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	int32_t ret = HTS221_ValidateParams(Reg,Buffer,Length);
	if(ret != HTS221_REG_NO_ERR)
	{
        return ret;
	}

	if(Context->Write != NULL)
	{
	    return Context->Write(Context->Handle, Reg, Buffer, Length);
	}

	return HTS221_REG_CONTEXT_ERR;
}


/***************************************************************************************************
 * @Brief: Helper function to validate input parameters to WriteReg and ReadReg functions
 * @Param: Reg to write, Buffer of data, Length of data
 * @Return: 0 if params are good. otherwise returns an error code
 ***************************************************************************************************/
static int32_t HTS221_ValidateParams(uint8_t Reg, uint8_t *Buffer, uint8_t Length)
{
	uint32_t ret = 0;

	if(Reg < HTS221_REG_WHO_AM_I || Reg > 0x3F)
	{
		 ret |= HTS221_REG_PARAM_ERR;
	}

	if(Buffer == NULL)
	{
		ret |= HTS221_BUFFER_PARAM_ERR;
	}

	if(Length == 0 || Length > (0x3F - HTS221_REG_WHO_AM_I))
	{  //0x3F - 0x0F = 0x30 readable registers
        ret |= HTS221_LENGTH_PARAM_ERR;
	}

	return ret;
}
