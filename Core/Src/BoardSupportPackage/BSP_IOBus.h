/*
 * BSP_IOBus.h
 *
 *  Created on: Sep 19, 2022
 *      Author: evanl
 */

#ifndef SRC_BOARDSUPPORTPACKAGE_BSP_IOBUS_H_
#define SRC_BOARDSUPPORTPACKAGE_BSP_IOBUS_H_
#include <stdint.h>
/* See BSP_IOBus.c for detailed descriptions*/
/* Public Function prototypes */
void BSP_I2C1_Init(void);
void BSP_I2C1_DeInit(void);
void BSP_I2C2_Init(void);
void BSP_I2C2_DeInit(void);
void BSP_ISM330DHCX_IO_Init(void);
void BSP_LPS22HH_IO_Init(void);
void BSP_IIS2MDC_IO_Init(void);

int32_t BSP_I2C1_WriteReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_WriteReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_ReadReg16(uint16_t DevAddr, uint16_t Reg, uint8_t *pData, uint16_t Length);

int32_t BSP_I2C2_ReadRegHTS221(uint8_t Reg, uint8_t *pData, uint8_t Length);
int32_t BSP_I2C2_WriteRegHTS221(uint8_t Reg, uint8_t *pData, uint8_t Length);

int32_t BSP_I2C2_ReadRegISM330DHCX(uint8_t Reg, uint8_t *pData, uint8_t Length);
int32_t BSP_I2C2_WriteRegISM330DHCX(uint8_t Reg, uint8_t *pData, uint8_t Length);
int32_t BSP_ReadPinISM330DHCX(void);

int32_t BSP_I2C2_ReadRegLPS22HH(uint8_t Reg, uint8_t *pData, uint8_t Length);
int32_t BSP_I2C2_WriteRegLPS22HH(uint8_t Reg, uint8_t *pData, uint8_t Length);
int32_t BSP_ReadPinLPS22HH(void);

int32_t BSP_I2C2_ReadRegIIS2MDC(uint8_t Reg, uint8_t *pData, uint8_t Length);
int32_t BSP_I2C2_WriteRegIIS2MDC(uint8_t Reg, uint8_t *pData, uint8_t Length);
int32_t BSP_ReadPinIIS2MDC(void);

int32_t BSP_I2C2_ReadRegVEML6030(uint8_t Reg, uint8_t *pData, uint8_t Length);
int32_t BSP_I2C2_WriteRegVEML6030(uint8_t Reg, uint8_t *pData, uint8_t Length);

uint32_t BSP_GetTick(void);

#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_IOBUS_H_ */
