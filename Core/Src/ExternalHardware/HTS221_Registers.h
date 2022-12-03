/*
 * HTS221_Registers.h
 *
 *  Created on: Oct 3, 2022
 *      Author: evanl
 */

#ifndef SRC_EXTERNALHARDWARE_HTS221_REGISTERS_H_
#define SRC_EXTERNALHARDWARE_HTS221_REGISTERS_H_
#include <stdint.h>
/* See HTS221_Reg.c for more details */

//Should be returned from "WHO AM I" register
#define HTS221_DEVICE_ID            (0xBC)

//Register list
#define HTS221_REG_WHO_AM_I         (0x0F)
#define HTS221_REG_AV_CONF          (0x10)
#define HTS221_REG_CTRL_REG1        (0x20)
#define HTS221_REG_CTRL_REG2        (0x21)
#define HTS221_REG_CTRL_REG3        (0x22)
#define HTS221_REG_STATUS_REG       (0x27)
#define HTS221_REG_HUMIDITY_OUT_L   (0x28)
#define HTS221_REG_HUMIDITY_OUT_H   (0x29)
#define HTS221_REG_TEMP_OUT_L       (0x2A)
#define HTS221_REG_TEMP_OUT_H       (0x2B)
#define HTS221_REG_H0_rH_x2         (0x30)
#define HTS221_REG_H1_rH_x2         (0x31)
#define HTS221_REG_T0_degC_x8       (0x32)
#define HTS221_REG_T1_degC_x8       (0x33)
#define HTS221_REG_T1_T0_msb        (0x35)
#define HTS221_REG_H0_T0_OUT        (0x36)
#define HTS221_REG_H1_T0_OUT        (0x3A)
#define HTS221_REG_T0_OUT           (0x3C)
#define HTS221_REG_T1_OUT           (0x3E)

//Error status codes
#define HTS221_REG_NO_ERR (0)
#define HTS221_REG_PARAM_ERR (1)
#define HTS221_BUFFER_PARAM_ERR (2)
#define HTS221_LENGTH_PARAM_ERR (4)
#define HTS221_REG_CONTEXT_ERR (8)


typedef struct{
    void *Handle;                                     //Pointer to the address of the HTS221 Handle (See HTS221.c/.h)
    int32_t (*Read)(void*,uint8_t,uint8_t*,uint8_t);  //Points to the HTS221_ReadReg wrapper in HTS221.c
    int32_t (*Write)(void*,uint8_t,uint8_t*,uint8_t); //Points to the HTS221_WriteReg wrapper in HTS221.c
}HTS221_Context_t;

//These functions call the Read/Write functions in the Context struct. The context struct points to the wrapper functions in HTS221.c
int32_t HTS221_ReadReg(HTS221_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length);
int32_t HTS221_WriteReg(HTS221_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length);

#endif /* SRC_EXTERNALHARDWARE_HTS221_REGISTERS_H_ */
