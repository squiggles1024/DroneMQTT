/*
 * HTS221.h
 *
 *  Created on: Sep 15, 2022
 *      Author: evanl
 */

#ifndef SRC_EXTERNALHARDWARE_HTS221_H_
#define SRC_EXTERNALHARDWARE_HTS221_H_
#include "HTS221_Registers.h"
#include <stdint.h>
/*See HTS221.c for driver details*/

//Status Codes
#define HTS221_ODR_Error (-2)
#define HTS221_Error (-1)
#define HTS221_PeriodOneShot (1)
#define HTS221_Ok (0)

/*Magic Number defines*/
#define HTS221_RebootSignal (1 << 7)
#define HTS221_BDU_ENABLE (1 << 2)
#define HTS221_START_CONV_SIGNAL (1)
#define HTS221_HEATER_ENABLE (1 << 1)
#define HTS221_POWERON_SIGNAL (1 << 7)
#define HTS221_TEMPDATA_Msk (1)
#define HTS221_HUMDATA_Msk (2)

/* Low Level IO Struct to be provided by user */
typedef struct{
    void    (*Init)(void);
    void    (*DeInit)(void);
    int32_t (*Write)(uint8_t, uint8_t*, uint8_t); //Reg, Buffer, Length
    int32_t (*Read)(uint8_t, uint8_t*, uint8_t);  //Reg, Buffer, Length
    uint32_t (*GetTick)(void);
    int32_t (*ReadPin)(void);
}HTS221_IO_t;

typedef enum{
 HTS221_DataReady,
 HTS221_DataNotReady = 0xBEEF
}HTS221_DataReadyStatus_t;

//Selects Number of Averaged Samples for Humidity
typedef enum{
    HTS221_Hum4Samples = 0,
	HTS221_Hum8Samples,
	HTS221_Hum16Samples,
	HTS221_Hum32amples,
    HTS221_Hum64Samples,
	HTS221_Hum128Samples,
	HTS221_Hum256Samples,
	HTS221_Hum512Samples
}HTS221_HumidityResMode_t;

//Selects Number of Averaged Samples for Temperature
typedef enum{
	HTS221_Temp2Samples = 0,
    HTS221_Temp4Samples = 1 << 3,
	HTS221_Temp8Samples = 2 << 3,
	HTS221_Temp16Samples = 3 << 3,
	HTS221_Temp32amples = 4 << 3,
    HTS221_Temp64Samples = 5 << 3,
	HTS221_Temp128Samples = 6 << 3,
	HTS221_Temp256Samples = 7 << 3
}HTS221_TemperatureResMode_t;

//Set BDU to 1

//Effects both Temp and Humidity
typedef enum{
    HTS221_OneShotMode = 0,
	HTS221_1_0Hz,
	HTS221_7_0p0Hz,
	HTS221_12_5Hz
}HTS221_OutputDataRate_t;

//Configure DRDY Pin to be active high or active low
typedef enum{
    HTS221_DataActiveHigh = 0,
	HTS221_DataActiveLow = 1 << 7
}HTS221_DataReadySignalPolarity_t;

//Configures the DRDY Pin as Open Drain or push pull
typedef enum{
   HTS221_DRDY_PushPull = 0,
   HTS221_DRDY_OpenDrain = 1 << 6
}HTS221_DataReadyPPOD_t;

//Whether or not the DRDY Pin is used.
typedef enum{
   HTS221_DataReadyDisable = 0,
   HTS221_DataReadyEnable = 1 << 2
}HTS221_DataReadyConfig_t;

typedef struct{
	HTS221_HumidityResMode_t HumidityResolution;
	HTS221_TemperatureResMode_t TemperatureResolution;
	HTS221_OutputDataRate_t OutputDataRate;
	HTS221_DataReadySignalPolarity_t DataReadySignalPolarity;
	HTS221_DataReadyPPOD_t DataReadyOutputMode;
	HTS221_DataReadyConfig_t DataReadyConfig;
}HTS221_Init_Struct_t;

typedef struct{
	HTS221_IO_t IO;
	HTS221_Context_t Context;
	HTS221_DataReadyConfig_t DRDY_Config;
	HTS221_DataReadySignalPolarity_t DRDY_Polarity;
	HTS221_OutputDataRate_t OutputDataRate;
}HTS221_Handle_t;

int32_t HTS221_Init(HTS221_Handle_t *Handle, HTS221_Init_Struct_t Settings, HTS221_IO_t *IO);
void HTS221_DeInit(HTS221_Handle_t *Handle);
int32_t HTS221_StartConversion(HTS221_Handle_t *Handle);//One shot Bit CTRL2
int32_t HTS221_EnableHeater(HTS221_Handle_t *Handle);   //Heater Bit CTRL1
int32_t HTS221_DisableHeater(HTS221_Handle_t *Handle);
int32_t HTS221_ResetRegisters(HTS221_Handle_t *Handle); //BOOT Bit CTRL2
int32_t HTS221_PowerCycle(HTS221_Handle_t *Handle);     //PD Bit CTRL1
int32_t HTS221_ReadTemperature(HTS221_Handle_t *Handle, float *temperature);
int32_t HTS221_ReadHumidity(HTS221_Handle_t *Handle, float *humidity);
int32_t HTS221_PowerDown(HTS221_Handle_t *Handle);
int32_t HTS221_PowerUp(HTS221_Handle_t *Handle);

int32_t HTS221_GetSamplePeriod(HTS221_Handle_t *Handle, uint32_t *Period);

#endif /* SRC_EXTERNALHARDWARE_HTS221_H_ */
