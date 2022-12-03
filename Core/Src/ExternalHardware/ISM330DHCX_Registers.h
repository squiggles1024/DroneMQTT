/*
 * ISM330DHCX_Registers.h
 *
 *  Created on: Oct 4, 2022
 *      Author: evanl
 */

#ifndef SRC_EXTERNALHARDWARE_ISM330DHCX_REGISTERS_H_
#define SRC_EXTERNALHARDWARE_ISM330DHCX_REGISTERS_H_
#include <stdint.h>
/* See ISM330DHCX_Registers.c for function details */

//Status Codes
#define ISM330DHCX_REG_NO_ERR (0)
#define ISM330DHCX_REG_PARAM_ERR (1)
#define ISM330DHCX_BUFFER_PARAM_ERR (2)
#define ISM330DHCX_LENGTH_PARAM_ERR (4)
#define ISM330DHCX_REG_CONTEXT_ERR (8)

/* Device ID (See "WHO_AM_I" reg) */
#define ISM330DHCX_DEVICE_ID (0x6BU)

/* Embedded functions configuration register */
#define ISM330DHCX_REG_FUNC_CFG_ACCESS (0x01U)
#define ISM330DHCX_REG_PIN_CTRL (0x02)

/* Embedded functions configuration register */
#define ISM330DHCX_REG_FIFO_CTRL1 (0x07U)
#define ISM330DHCX_REG_FIFO_CTRL2 (0x08U)
#define ISM330DHCX_REG_FIFO_CTRL3 (0x09U)
#define ISM330DHCX_REG_FIFO_CTRL4 (0x0AU)

/* Counter batch data rate register */
#define ISM330DHCX_REG_COUNTER_BDR_REG1 (0x0BU)
#define ISM330DHCX_REG_COUNTER_BDR_REG2 (0x0CU)

/*  INT1 pin control */
#define ISM330DHCX_REG_INT1_CTRL (0x0DU)

/*  INT2 pin control */
#define ISM330DHCX_REG_INT2_CTRL (0x0EU)

/*  Who I am ID */
#define ISM330DHCX_REG_WHO_AM_I (0x0FU)

/* Accelerometer and gyroscope control registers */
#define ISM330DHCX_REG_CTRL1_XL (0x10U)
#define ISM330DHCX_REG_CTRL2_G (0x11U)
#define ISM330DHCX_REG_CTRL3_C (0x12U)
#define ISM330DHCX_REG_CTRL4_C (0x13U)
#define ISM330DHCX_REG_CTRL5_C (0x14U)
#define ISM330DHCX_REG_CTRL6_C (0x15U)
#define ISM330DHCX_REG_CTRL7_G (0x16U)
#define ISM330DHCX_REG_CTRL8_XL (0x17U)
#define ISM330DHCX_REG_CTRL9_XL (0x18U)
#define ISM330DHCX_REG_CTRL10_C (0x19U)

/* Interrupt registers */
#define ISM330DHCX_REG_ALL_INT_SRC (0x1AU)
#define ISM330DHCX_REG_WAKE_UP_SRC (0x1BU)
#define ISM330DHCX_REG_TAP_SRC (0x1CU)
#define ISM330DHCX_REG_D6D_SRC (0x1DU)

/* Status data register for user interface */
#define ISM330DHCX_REG_STATUS_REG (0x1EU)

/* Temperature output data registers */
#define ISM330DHCX_REG_OUT_TEMP_L (0x20U)
#define ISM330DHCX_REG_OUT_TEMP_H (0x21U)

/* Gyroscope output registers for user interface */
#define ISM330DHCX_REG_OUTX_L_G (0x22U)
#define ISM330DHCX_REG_OUTX_H_G (0x23U)
#define ISM330DHCX_REG_OUTY_L_G (0x24U)
#define ISM330DHCX_REG_OUTY_H_G (0x25U)
#define ISM330DHCX_REG_OUTZ_L_G (0x26U)
#define ISM330DHCX_REG_OUTZ_H_G (0x27U)

/* Accelerometer output registers */
#define ISM330DHCX_REG_OUTX_L_A (0x28U)
#define ISM330DHCX_REG_OUTX_H_A (0x29U)
#define ISM330DHCX_REG_OUTY_L_A (0x2AU)
#define ISM330DHCX_REG_OUTY_H_A (0x2BU)
#define ISM330DHCX_REG_OUTZ_L_A (0x2CU)
#define ISM330DHCX_REG_OUTZ_H_A (0x2DU)

/* FSM Registers */
#define ISM330DHCX_REG_EMB_FUNC_STATUS_MAINPAGE (0x35U)
#define ISM330DHCX_REG_FSM_STATUS_A_MAINPAGE (0x36U)
#define ISM330DHCX_REG_FSM_STATUS_B_MAINPAGE (0x37U)

/* Machine Learning Core Register */
#define ISM330DHCX_REG_MLC_STATUS_MAINPAGE (0x38U)

/* Sensor Hub source register */
#define ISM330DHCX_REG_STATUS_MASTER_MAINPAGE (0x39U)

/* FIFO status registers */
#define ISM330DHCX_REG_FIFO_STATUS1 (0x3AU)
#define ISM330DHCX_REG_FIFO_STATUS2 (0x3BU)

/* Timestamp output registers */
#define ISM330DHCX_REG_TIMESTAMP0_REG (0x40U)
#define ISM330DHCX_REG_TIMESTAMP1_REG (0x41U)
#define ISM330DHCX_REG_TIMESTAMP2_REG (0x42U)
#define ISM330DHCX_REG_TIMESTAMP3_REG (0x43U)

/* Interrupt registers */
#define ISM330DHCX_REG_TAP_CFG0 (0x56U)
#define ISM330DHCX_REG_TAP_CFG1 (0x57U)
#define ISM330DHCX_REG_TAP_CFG2 (0x58U)
#define ISM330DHCX_REG_TAP_THS_6D (0x59U)
#define ISM330DHCX_REG_INT_DUR2 (0x5AU)
#define ISM330DHCX_REG_WAKE_UP_THS (0x5BU)
#define ISM330DHCX_REG_WAKE_UP_DUR (0x5CU)
#define ISM330DHCX_REG_FREE_FALL (0x5DU)
#define ISM330DHCX_REG_MD1_CFG (0x5EU)
#define ISM330DHCX_REG_MD2_CFG (0x5FU)

/* Internal frequency register */
#define ISM330DHCX_REG_INTERNAL_FREQ_FINE (0x63U)

/* Optical Image Stabilization Registers */
#define ISM330DHCX_REG_INT_OIS (0x6FU)
#define ISM330DHCX_REG_CTRL1_OIS (0x70U)
#define ISM330DHCX_REG_CTRL2_OIS (0x71U)
#define ISM330DHCX_REG_CTRL3_OIS (0x72U)

/* Accelerometer user offset correction */
#define ISM330DHCX_REG_X_OFS_USR (0x73U)
#define ISM330DHCX_REG_Y_OFS_USR (0x74U)
#define ISM330DHCX_REG_Z_OFS_USR (0x75U)

/* Fifo data out registers, Tag = Sensor which produced the data in FIFO */
#define ISM330DHCX_REG_FIFO_DATA_OUT_TAG (0x78U)
#define ISM330DHCX_REG_FIFO_OUT_X_L (0x79U)
#define ISM330DHCX_REG_FIFO_OUT_X_H (0x7AU)
#define ISM330DHCX_REG_FIFO_OUT_Y_L (0x7BU)
#define ISM330DHCX_REG_FIFO_OUT_Y_H (0x7CU)
#define ISM330DHCX_REG_FIFO_OUT_Z_L (0x7DU)
#define ISM330DHCX_REG_FIFO_OUT_Z_H (0x7EU)


typedef struct{
    void (*Handle);                                      //Pointer to the address of the ISM330DHCX Handle (See ISM330DHCX.c/.h)
    int32_t (*Write)(void*, uint8_t, uint8_t*, uint8_t); //Points to the ISM330DHCX_WriteReg wrapper in ISM330DHCX.c
    int32_t (*Read)(void*, uint8_t, uint8_t*, uint8_t);  //Points to the ISM330DHCX_ReadReg wrapper in ISM330DHCX.c
}ISM330DHCX_Context_t;

//These functions call the Read/Write functions in the Context struct. The context struct points to the wrapper functions in ISM330DHCX.c
int32_t ISM330DHCX_ReadReg(ISM330DHCX_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length);
int32_t ISM330DHCX_WriteReg(ISM330DHCX_Context_t *Context, uint8_t Reg, uint8_t *Buffer, uint8_t Length);


#endif /* SRC_EXTERNALHARDWARE_ISM330DHCX_REGISTERS_H_ */
