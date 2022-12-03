/*
 * VEML6030_Registers.h
 *
 *  Created on: Oct 27, 2022
 *      Author: evanl
 */

#ifndef SRC_EXTERNALHARDWARE_VEML6030_REGISTERS_H_
#define SRC_EXTERNALHARDWARE_VEML6030_REGISTERS_H_
#include <stdint.h>
/* See VEML6030_Register.c for more details */

//Registers
#define VEML6030_REG_ALS_CONF (0x00)
#define VEML6030_REG_ALS_WH   (0x01)
#define VEML6030_REG_ALS_WL   (0x02)
#define VEML6030_REG_POWERSAVING (0x03)
#define VEML6030_REG_ALS      (0x04)
#define VEML6030_REG_WHITE    (0x05)
#define VEML6030_REG_ALS_INT  (0x06)

//Status Codes
#define VEML6030_REG_OK          (0)
#define VEML6030_REG_CONTEXT_ERR (1 << 31)  | (1 << 27)
#define VEML6030_REG_BUFFER_ERR  ((1 << 31) | (1 << 30))
#define VEML6030_REG_LENGTH_ERR  ((1 << 31) | (1 << 29))
#define VEML6030_REG_ADDR_ERR  ((1 << 31) | (1 << 28))

typedef struct
{
	void *Handle;                                        //Pointer to the address of the VEML6030 Handle (See VEML6030.c/.h)
	int32_t (*Write)(void *,uint8_t,uint8_t*,uint8_t);   //Points to the VEML6030_WriteReg wrapper in VEML6030.c
	int32_t (*Read)(void *, uint8_t, uint8_t*, uint8_t); //Points to the VEML6030_ReadReg wrapper in VEML6030.c
}VEML6030_Context_t;

//These functions call the Read/Write functions in the Context struct. The context struct points to the wrapper functions in VEML6030.c
int32_t VEML6030_ReadReg(VEML6030_Context_t *Context,uint8_t Reg, uint8_t *Buffer, uint8_t Length);
int32_t VEML6030_WriteReg(VEML6030_Context_t *Context,uint8_t Reg, uint8_t *Buffer, uint8_t Length);


#endif /* SRC_EXTERNALHARDWARE_VEML6030_REGISTERS_H_ */
