/*
 * BSP_IO_Bus.c
 *
 *  Created on: Sep 19, 2022
 *      Author: evanl
 */

#ifndef SRC_BOARDSUPPORTPACKAGE_BSP_IO_BUS_C_
#define SRC_BOARDSUPPORTPACKAGE_BSP_IO_BUS_C_
/* Includes */
#include "i2c.h"
#include "gpio.h"
#include <stdint.h>
#include "BSP_IOBus.h"

/* Private Defines */
#define HTS221_I2C_ADDR (0xBEU)            //I2C Address of temp sensor
#define ISM330DHCX_I2C_ADDR (0xD6U)        //I2C Address of accel/gyro
#define LPS22HH_I2C_ADDR (0xBAU)           //I2C Address of pressure sensor
#define IIS2MDC_I2C_ADDR (0x3CU)           //I2C Address of magnetometer
#define VEML6030_I2C_ADDR (0x20U)          //I2C Address of light sensor

/**************************************************************************************************************************************
 * @Brief: Initializes I2C1 Bus if it hasn't been initialized yet
 * @Param: None
 * @Return: None
 * @Post Condition: I2C1 bus will be ready to use
 * @Pre Condition: None
 **************************************************************************************************************************************/
void BSP_I2C1_Init(void)
{
	if(hi2c1.State == HAL_I2C_STATE_RESET)
	{
        I2C1_Init();
	}
}

/**************************************************************************************************************************************
 * @Brief: Deinitializes I2C1 Bus
 * @Param: None
 * @Return: None
 * @Post Condition: I2C1 bus will be reset to uninitialized state
 * @Pre Condition: None
 **************************************************************************************************************************************/
void BSP_I2C1_DeInit(void)
{
	HAL_I2C_DeInit(&hi2c1);
}

/**************************************************************************************************************************************
 * @Brief: Initializes I2C2 Bus if it hasn't been initialized yet
 * @Param: None
 * @Return: None
 * @Post Condition: I2C2 bus will be ready to use
 * @Pre Condition: None
 **************************************************************************************************************************************/
void BSP_I2C2_Init(void)
{
	if(hi2c2.State == HAL_I2C_STATE_RESET)
	{
        I2C2_Init();
	}
}

/**************************************************************************************************************************************
 * @Brief: Deinitializes I2C2 Bus
 * @Param: None
 * @Return: None
 * @Post Condition: I2C2 bus will be reset to uninitialized state
 * @Pre Condition: None
 **************************************************************************************************************************************/
void BSP_I2C2_DeInit(void)
{
	HAL_I2C_DeInit(&hi2c2);
}

/**************************************************************************************************************************************
 * @Brief: Writes to device on I2C1 Bus with 8 bit registers
 * @Param: Address of receiving device, Register to write to, data to write, number of bytes to write
 * @Return: HAL Status, See I2C1 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be written to.
 * @Pre Condition: I2C1 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C1_WriteReg(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length){
    int32_t ret = HAL_I2C_Mem_Write(&hi2c1, Address, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 1000);
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Reads from device on I2C1 bus with 8 bit registers
 * @Param: Address of receiving device, Register to read from, pointer to data buffer, number of bytes to read
 * @Return: HAL Status, See I2C1 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be read from, and read bytes stored in pData.
 * @Pre Condition: I2C1 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C1_ReadReg(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length){
    int32_t ret = HAL_I2C_Mem_Read(&hi2c1, Address, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 1000);
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Writes to device on I2C1 Bus with 16 bit registers
 * @Param: Address of receiving device, Register to write to, data to write, number of bytes to write
 * @Return: HAL Status, See I2C1 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be written to.
 * @Pre Condition: I2C1 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C1_WriteReg16(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length){
    int32_t ret = HAL_I2C_Mem_Write(&hi2c1, Address, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, 1000);
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Reads from device on I2C1 Bus with 16 bit registers
 * @Param: Address of receiving device, Register to write to, data buffer to read into, number of bytes to read
 * @Return: HAL Status, See I2C1 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be read from, and read bytes stored in pData.
 * @Pre Condition: I2C1 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C1_ReadReg16(uint16_t Address, uint16_t Reg, uint8_t *pData, uint16_t Length){
    int32_t ret = HAL_I2C_Mem_Read(&hi2c1, Address, Reg, I2C_MEMADD_SIZE_16BIT, pData, Length, 1000);
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Writes to HT221 device using I2C2 Bus
 * @Param: Reg to write to, pointer to data to write, number of bytes to write
 * @Return: HAL Status, See I2C2 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be written to.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_WriteRegHTS221(uint8_t Reg, uint8_t *pData, uint8_t Length){
    int32_t ret = HAL_I2C_Mem_Write(&hi2c2, HTS221_I2C_ADDR, Reg | 0x80, I2C_MEMADD_SIZE_8BIT, pData, Length, 100); //Set 0x80 bit to enable multibyte write/read
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Reads from HT221 device using I2C2 Bus
 * @Param: Reg to read from, pointer to data to data buffer to fill, number of bytes to read
 * @Return: HAL Status, See I2C2 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be read from and data stored in pData.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_ReadRegHTS221(uint8_t Reg, uint8_t *pData, uint8_t Length){
    int32_t ret = HAL_I2C_Mem_Read(&hi2c2, HTS221_I2C_ADDR, Reg | 0x80, I2C_MEMADD_SIZE_8BIT, pData, Length, 100); //Set 0x80 bit to enable multibyte write/read
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Reads from ISM330DHCX device using I2C2 Bus
 * @Param: Reg to read from, pointer to data to data buffer to fill, number of bytes to read
 * @Return: HAL Status, See I2C2 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be read from and data stored in pData.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_ReadRegISM330DHCX(uint8_t Reg, uint8_t *pData, uint8_t Length)
{
	int32_t ret = HAL_I2C_Mem_Read(&hi2c2, ISM330DHCX_I2C_ADDR, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100);
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Writes to ISM330DHCX device using I2C2 Bus
 * @Param: Reg to write to, pointer to data to write, number of bytes to write
 * @Return: HAL Status, See I2C2 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be written to.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_WriteRegISM330DHCX(uint8_t Reg, uint8_t *pData, uint8_t Length)
{
	int32_t ret = HAL_I2C_Mem_Write(&hi2c2, ISM330DHCX_I2C_ADDR, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100);
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Reads ISM330DHCX device INT1 Pin. Can be used instead of polling the device status register for seeing if new data is available.
 * @Param: None
 * @Return: Pin State (gpio pin set or reset)
 * @Post Condition: None
 * @Pre Condition: GPIO Pin should be configured as input
 **************************************************************************************************************************************/
int32_t BSP_ReadPinISM330DHCX(void)
{
    return HAL_GPIO_ReadPin(Mems_ISM330DLC_INT1_GPIO_Port, Mems_ISM330DLC_INT1_Pin);
}

/**************************************************************************************************************************************
 * @Brief: Initializes ISM330DHCX Gpio pins and I2C Bus
 * @Param: None
 * @Return: None
 * @Post Condition: GPIO and I2C Bus for ISM330DHCX will be ready to use
 * @Pre Condition: None
 **************************************************************************************************************************************/
void BSP_ISM330DHCX_IO_Init(void)
{
    MotionSensor_GPIO_Init();
	BSP_I2C2_Init();
}

/**************************************************************************************************************************************
 * @Brief: Initializes LPS22HH Gpio pins and I2C Bus
 * @Param: None
 * @Return: None
 * @Post Condition: GPIO and I2C Bus for LPS22HH will be ready to use
 * @Pre Condition: None
 **************************************************************************************************************************************/
void BSP_LPS22HH_IO_Init(void)
{
	PressureSensor_GPIO_Init();
	BSP_I2C2_Init();
}

/**************************************************************************************************************************************
 * @Brief: Reads from LPS22HH device using I2C2 Bus
 * @Param: Reg to read from, pointer to data to data buffer to fill, number of bytes to read
 * @Return: HAL Status, See I2C2 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be read from and data stored in pData.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_ReadRegLPS22HH(uint8_t Reg, uint8_t *pData, uint8_t Length)
{
	int32_t ret = HAL_I2C_Mem_Read(&hi2c2, LPS22HH_I2C_ADDR, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100);
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Writes to LPS22HH device using I2C2 Bus
 * @Param: Reg to write to, pointer to data to write, number of bytes to write
 * @Return: HAL Status, See I2C2 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be written to.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_WriteRegLPS22HH(uint8_t Reg, uint8_t *pData, uint8_t Length)
{
	int32_t ret = HAL_I2C_Mem_Write(&hi2c2, LPS22HH_I2C_ADDR, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100);
	if(ret != HAL_OK && hi2c2.ErrorCode == HAL_I2C_ERROR_AF)
	{
		ret = HAL_I2C_Mem_Write(&hi2c2, LPS22HH_I2C_ADDR, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100); //Retry...fix bug that occurs during initialization.
	}

	if(ret != HAL_OK)
	{
		return hi2c2.ErrorCode;
	}
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Reads LPS22HH device INT Pin. Can be used instead of polling the device status register for seeing if new data is available.
 * @Param: None
 * @Return: Pin State (gpio pin set or reset)
 * @Post Condition: None
 * @Pre Condition: GPIO Pin should be configured as input
 **************************************************************************************************************************************/
int32_t BSP_ReadPinLPS22HH(void)
{
	return HAL_GPIO_ReadPin(Mems_INT_LPS22HH_GPIO_Port, Mems_INT_LPS22HH_Pin);
}

/**************************************************************************************************************************************
 * @Brief: Initializes IIS2MDC Gpio pins and I2C Bus
 * @Param: None
 * @Return: None
 * @Post Condition: GPIO and I2C Bus for IIS2MDC will be ready to use
 * @Pre Condition: None
 **************************************************************************************************************************************/
void BSP_IIS2MDC_IO_Init(void)
{
	MagSensor_GPIO_Init();
	BSP_I2C2_Init();
}

/**************************************************************************************************************************************
 * @Brief: Reads from IIS2MDC device using I2C2 Bus
 * @Param: Reg to read from, pointer to data to data buffer to fill, number of bytes to read
 * @Return: HAL Status, See I2C2 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be read from and data stored in pData.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_ReadRegIIS2MDC(uint8_t Reg, uint8_t *pData, uint8_t Length)
{
	int32_t ret = HAL_I2C_Mem_Read(&hi2c2, IIS2MDC_I2C_ADDR, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100);
	if(ret != HAL_OK)
	{
		return hi2c2.ErrorCode;
	}
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Writes to IIS2MDC device using I2C2 Bus
 * @Param: Reg to write to, pointer to data to write, number of bytes to write
 * @Return: HAL Status, See I2C2 Handle Error code if HAL_Ok not returned.
 * @Post Condition: Device with given address will be written to.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_WriteRegIIS2MDC(uint8_t Reg, uint8_t *pData, uint8_t Length)
{
	int32_t ret = HAL_I2C_Mem_Write(&hi2c2, IIS2MDC_I2C_ADDR, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100);
	if(ret != HAL_OK)
	{
		return hi2c2.ErrorCode;
	}
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Reads IIS2MDC device INT Pin. Can be used instead of polling the device status register for seeing if new data is available.
 * @Param: None
 * @Return: Pin State (gpio pin set or reset)
 * @Post Condition: None
 * @Pre Condition: GPIO Pin should be configured as input
 **************************************************************************************************************************************/
int32_t BSP_ReadPinIIS2MDC(void)
{
	return HAL_GPIO_ReadPin(Mems_INT_IIS2MDC_GPIO_Port, Mems_INT_IIS2MDC_Pin);
}

/**************************************************************************************************************************************
 * @Brief: Reads Ticks from system time base timer. Can be used to initiate delays.
 * @Param: None
 * @Return: Current ticks from system time base timer
 * @Post Condition: None
 * @Pre Condition: Timer should be intialized and running.
 **************************************************************************************************************************************/
uint32_t BSP_GetTick(void)
{
    return HAL_GetTick();
}

/**************************************************************************************************************************************
 * @Brief: Reads from VEML6030 device using I2C2 Bus
 * @Param: Reg to read from, pointer to data to data buffer to fill, number of bytes to read
 * @Return: I2C2 Handle Error Code or HAL_Ok
 * @Post Condition: Device with given address will be read from and data stored in pData.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_ReadRegVEML6030(uint8_t Reg, uint8_t *pData, uint8_t Length)
{
	int32_t ret = HAL_I2C_Mem_Read(&hi2c2, VEML6030_I2C_ADDR, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100);
	if(ret != HAL_OK)
	{
		return hi2c2.ErrorCode;
	}
    return ret;
}

/**************************************************************************************************************************************
 * @Brief: Writes to VEML6030 device using I2C2 Bus
 * @Param: Reg to write to, pointer to data to write, number of bytes to write
 * @Return: I2C2 Handle Error Code or HAL_Ok
 * @Post Condition: Device with given address will be written to.
 * @Pre Condition: I2C2 bus should be initialized
 **************************************************************************************************************************************/
int32_t BSP_I2C2_WriteRegVEML6030(uint8_t Reg, uint8_t *pData, uint8_t Length)
{
	int32_t ret = HAL_I2C_Mem_Write(&hi2c2, VEML6030_I2C_ADDR, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100);
	if(ret != HAL_OK)
	{
		return hi2c2.ErrorCode;
	}
    return ret;
}


#endif /* SRC_BOARDSUPPORTPACKAGE_BSP_IO_BUS_C_ */
