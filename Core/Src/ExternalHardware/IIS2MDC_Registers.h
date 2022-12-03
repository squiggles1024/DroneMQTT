/*
 * IIS2MDC_Registers.h
 *
 *  Created on: Oct 22, 2022
 *      Author: evanl
 */

#ifndef SRC_EXTERNALHARDWARE_IIS2MDC_REGISTERS_H_
#define SRC_EXTERNALHARDWARE_IIS2MDC_REGISTERS_H_
#include <stdint.h>
/*See IIS2MDC_Register.c for driver details*/
//Error Codes
#define IIS2MDC_REG_OK          (0)
#define IIS2MDC_REG_CONTEXT_ERR (1 << 31)  | (1 << 27)
#define IIS2MDC_REG_BUFFER_ERR  ((1 << 31) | (1 << 30))
#define IIS2MDC_REG_LENGTH_ERR  ((1 << 31) | (1 << 29))
#define IIS2MDC_REG_ADDR_ERR  ((1 << 31) | (1 << 28))


typedef struct
{
    void *Handle;                                     //Pointer to the address of the IIS2MDC Handle (See IIS2MDC.c/.h)
    int32_t (*Read)(void*,uint8_t,uint8_t*,uint8_t);  //Points to the IIS2MDC_ReadReg wrapper in IIS2MDC.c
    int32_t (*Write)(void*,uint8_t,uint8_t*,uint8_t); //Points to the IIS2MDC_WriteReg wrapper in IIS2MDC.c
}IIS2MDC_Context_t;

//These functions call the Read/Write functions in the Context struct. The context struct points to the wrapper functions in IIS2MDC.c
int32_t IIS2MDC_ReadReg(IIS2MDC_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length);
int32_t IIS2MDC_WriteReg(IIS2MDC_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length);


//Reg Addresses
#define IIS2MDC_REG_OFFSET_X_REG_L (0x45U)
#define IIS2MDC_REG_OFFSET_X_REG_H (0x46U)
#define IIS2MDC_REG_OFFSET_Y_REG_L (0x47U)
#define IIS2MDC_REG_OFFSET_Y_REG_H (0x48U)
#define IIS2MDC_REG_OFFSET_Z_REG_L (0x49U)
#define IIS2MDC_REG_OFFSET_Z_REG_H (0x4AU)

#define IIS2MDC_REG_WHO_AM_I (0x4FU)

#define IIS2MDC_REG_CFG_REG_A (0x60U)
#define IIS2MDC_REG_CFG_REG_B (0x61U)
#define IIS2MDC_REG_CFG_REG_C (0x62U)

#define IIS2MDC_REG_INT_CTRL_REG (0x63U)
#define IIS2MDC_REG_INT_SOURCE_REG (0x64U)
#define IIS2MDC_REG_INT_THS_L_REG (0x65U)
#define IIS2MDC_REG_INT_THS_H_REG (0x66U)

#define IIS2MDC_REG_STATUS_REG (0x67U)

#define IIS2MDC_REG_OUTX_L_REG (0x68U)
#define IIS2MDC_REG_OUTX_H_REG (0x69U)
#define IIS2MDC_REG_OUTY_L_REG (0x6AU)
#define IIS2MDC_REG_OUTY_H_REG (0x6BU)
#define IIS2MDC_REG_OUTZ_L_REG (0x6CU)
#define IIS2MDC_REG_OUTZ_H_REG (0x6DU)

#define IIS2MDC_REG_TEMP_OUT_L_REG (0x6EU)
#define IIS2MDC_REG_TEMP_OUT_H_REG (0x6FU)


#endif /* SRC_EXTERNALHARDWARE_IIS2MDC_REGISTERS_H_ */
