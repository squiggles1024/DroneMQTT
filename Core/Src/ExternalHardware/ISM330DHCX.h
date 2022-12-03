/*
 * ISM330DHCX.h
 *
 *  Created on: Sep 15, 2022
 *      Author: evanl
 */

#ifndef SRC_EXTERNALHARDWARE_ISM330DHCX_H_
#define SRC_EXTERNALHARDWARE_ISM330DHCX_H_
#include "ISM330DHCX_Registers.h"
#include <stdint.h>

//Status Codes
#define ISM330DHCX_Error (-1)
#define ISM330DHCX_ReadRegErr (-2)
#define ISM330DHCX_ParamError (-3)
#define ISM330DHCX_Ok (0)
#define ISM330DHCX_InitError (-4)

//Magic Numbers
#define ISM330DHCX_ResetSignal (0x01)
#define ISM330DHCX_DefaultSetting (0x00)

#define ISM330DHCX_SubMilliPeriod (1)
#define ISM330DHCX_PeriodOneShot (2)

//Low Level IO Struct
typedef struct{
    void (*Init)(void);
    void (*DeInit)(void);
    int32_t (*Read)(uint8_t, uint8_t*, uint8_t);
    int32_t (*Write)(uint8_t, uint8_t*, uint8_t);
    int32_t (*ReadInt1Pin)(void);
    int32_t (*ReadInt2Pin)(void);
    uint32_t (*GetTick)(void);
}ISM330DHCX_IO_t;

//Status codes returned by Read Data functions
typedef enum{
	ISM330DHCX_DataReady,
	ISM330DHCX_DataNotReady = 0xBEEF
}ISM330DHCX_DataReadyStatus_t;

//Enum used to keep track of whether or not a device handle is initialized.
typedef enum
{
	ISM330DHCX_Uninitialized = 0,
	ISM330DHCX_Initialized
}ISM330DHCX_InitStatus_t;

/*PIN_CTRL REG, set bits 0-5 to 1*/
typedef enum{
	ISM330DHCX_SDOPullupDisconnected = 0,
	ISM330DHCX_SDOPulledUp = 1 << 6
}ISM330DHCX_SDOPullUp_t;

/*FIFO CTRL 1 Reg*/
typedef uint16_t ISM330DHCX_WatermarkThreshold; //9 Bit Number, 1 LSB = 7 Bytes (6 Data, 1 Tag (Tag = Sensor ID, TimeStamp, Parity)

/*FIFO CTRL 2 Reg*/
typedef enum{
	ISM330DHCX_FifoDepthUnlimited = 0,
	ISM330DHCX_FifoDepthWaterMark = 1 << 7 //Fifo Depth limited to Water Mark
}ISM330DHCX_FifoDepth_t;

/*FIFO CTRL 3 Reg*/
typedef enum{
	ISM330DHCX_GyroNotBatched= 0,
	ISM330DHCX_GyroBatched_12_5Hz = 1 << 4,
	ISM330DHCX_GyroBatched_26Hz = 2 << 4,
	ISM330DHCX_GyroBatched_52Hz = 3 << 4,
	ISM330DHCX_GyroBatched_104Hz = 4 << 4,
	ISM330DHCX_GyroBatched_208Hz = 5 << 4,
	ISM330DHCX_GyroBatched_417Hz = 6 << 4,
	ISM330DHCX_Gyroatched_833Hz = 7 << 4,
	ISM330DHCX_GyroBatched_1667Hz = 8 << 4,
	ISM330DHCX_GyroBatched_3333Hz = 9 << 4,
	ISM330DHCX_GyroBatched_6667Hz = 10 << 4,
	ISM330DHCX_GyroBatched_6_5Hz = 11 << 4,
}ISM330DHCX_GyroBatchDataRate_t;

typedef enum{
	ISM330DHCX_AccelNotBatched= 0,
	ISM330DHCX_AccelBatched_12_5Hz = 1,
	ISM330DHCX_AccelBatched_26Hz = 2,
	ISM330DHCX_AccelBatched_52Hz = 3,
	ISM330DHCX_AccelBatched_104Hz = 4,
	ISM330DHCX_AccelBatched_208Hz = 5,
	ISM330DHCX_AccelBatched_417Hz = 6,
	ISM330DHCX_Accelatched_833Hz = 7,
	ISM330DHCX_AccelBatched_1667Hz = 8,
	ISM330DHCX_AccelBatched_3333Hz = 9,
	ISM330DHCX_AccelBatched_6667Hz = 10,
	ISM330DHCX_AccelBatched_1_6Hz = 11,
}ISM330DHCX_AccelBatchDataRate_t;


/*FIFO CTRL 4 Reg*/
typedef enum{
	ISM330DHCX_NoDecimation = 0,
	ISM330DHCX_DecimationFreqDiv1 = 1 << 6,
	ISM330DHCX_DecimationFreqDiv8 = 2 << 6,
	ISM330DHCX_DecimationFreqDiv32 = 3 << 6
}ISM330DHCX_BatchDecimationRate_t;

typedef enum{
	ISM330DHCX_TempNotBatched= 0,
	ISM330DHCX_TempBatched_1_6Hz = 1 << 4,
	ISM330DHCX_TempBatched_12_5Hz = 2 << 4,
	ISM330DHCX_TempBatched_52Hz = 3 << 4
}ISM330DHCX_TempBatchDataRate_t;

typedef enum{
	ISM330DHCX_FifoBypassed = 0,
	ISM330DHCX_FifoStopOnFull = 1,
	ISM330DHCX_ContinuousToFifo = 3,
	ISM330DHCX_BypassToContinuous = 4,
	ISM330DHCX_Continuous = 6,
	ISM330DHCX_BypassToFifo = 7
}ISM330DHCX_FifoMode_t;

/*Counter BDR Reg 1*/
typedef enum{
	ISM330DHCX_DataReadyLatched = 0,
	ISM330DHCX_DataReadyPulsed = 1 << 7
}ISM330DHCX_DataReadyMode_t;

typedef enum{
	ISM330DHCX_BatchEvtOnXL = 0,
	ISM330DHCX_BatchEvtOnGyro = 1 << 5
}ISM330DHCX_BatchEvtTrigger_t;

/*Counter BDR Reg 1 and 2*/
typedef uint16_t ISM330DHCX_BatchDataEvtThreshold;

/*Int1 Ctrl Reg*/
typedef enum{
	ISM330DHCX_INT_DEN_DRDY_flag = 1 << 7,
	ISM330DHCX_INT1_CNT_BDR = 1 << 6,
	ISM330DHCX_INT1_FIFO_FULL = 1 << 5,
	ISM330DHCX_INT1_FIFO_OVR = 1 << 4,
	ISM330DHCX_INT1_FIFO_TH = 1 << 3,
	ISM330DHCX_INT1_BOOT = 1 << 2,
	ISM330DHCX_INT1_DRDY_G = 1 << 1,
	ISM330DHCX_INT1_DRDY_XL = 1 << 0
}ISM330DHCX_Int1trigger_t;

/*Int 2 Ctrl Reg*/
typedef enum{
	ISM330DHCX_INT2_CNT_BDR = 1 << 6,
	ISM330DHCX_INT2_FIFO_FULL = 1 << 5,
	ISM330DHCX_INT2_FIFO_OVR = 1 << 4,
	ISM330DHCX_INT2_FIFO_TH = 1 << 3,
	ISM330DHCX_INT2_TEMP = 1 << 2,
	ISM330DHCX_INT2_DRDY_G = 1 << 1,
	ISM330DHCX_INT2_DRDY_XL = 1 << 0
}ISM330DHCX_Int2trigger_t;

/*Ctrl1 XL Reg*/
typedef enum{
	ISM330DHCX_XL_PowerDown = 0,
	ISM330DHCX_XL_1_6Hz = 11 << 4,
	ISM330DHCX_XL_12_5Hz = 1 << 4,
	ISM330DHCX_XL_26Hz = 2 << 4,
	ISM330DHCX_XL_52Hz = 3 << 4,
	ISM330DHCX_XL_104Hz = 4 << 4,
	ISM330DHCX_XL_208Hz = 5 << 4,
	ISM330DHCX_XL_416Hz = 6 << 4,
	ISM330DHCX_XL_833Hz = 7 << 4,
	ISM330DHCX_XL_1666Hz = 8 << 4,
	ISM330DHCX_XL_3333Hz = 9 << 4,
	ISM330DHCX_XL_6666Hz = 10 << 4
}ISM330DHCX_AccelODR_t;

typedef enum{
	ISM330DHCX_XL_2g = 0,
	ISM330DHCX_XL_16g = 1 << 2,
	ISM330DHCX_XL_4g = 2 << 2,
	ISM330DHCX_XL_8g = 3 << 2
}ISM330DHCX_AccelFullscale_t;

typedef enum{
	ISM330DHCX_XL_SSF_Disabled = 0,
	ISM330DHCX_XL_SSF_Enabled = 1 << 1
}ISM330DHCX_XL_SecondStageLPF_t;

/*CTRL2 G Reg*/
typedef enum{
	ISM330DHCX_GyroPowerDown = 0,
	ISM330DHCX_G_12_5Hz = 1 << 4,
	ISM330DHCX_G_26Hz = 2 << 4,
	ISM330DHCX_G_52Hz = 3 << 4,
	ISM330DHCX_G_104Hz = 4 << 4,
	ISM330DHCX_G_208Hz = 5 << 4,
	ISM330DHCX_G_416Hz = 6 << 4,
	ISM330DHCX_G_833Hz = 7 << 4,
	ISM330DHCX_G_1666Hz = 8 << 4,
	ISM330DHCX_G_3333Hz = 9 << 4,
	ISM330DHCX_G_6666Hz = 10 << 4
}ISM330DHCX_GyroODR_t;

typedef enum{
	ISM330DHCX_G_125DPS = 1 << 1,
	ISM330DHCX_G_250DPS = 0,
	ISM330DHCX_G_500DPS = 1 << 2,
	ISM330DHCX_G_1000DPS = 2 << 2,
	ISM330DHCX_G_2000DPS = 3 << 2,
	ISM330DHCX_G_4000DPS = 1
}ISM330DHCX_GyroFullScale_t;

/*CTRL3 C Reg*/
typedef enum{
	ISM330DHCX_IRQ_ActiveHigh = 0,
	ISM330DHCX_IRQ_ActiveLow = 1 << 5,
}ISM330DHCX_IRQPolarity_t;

typedef enum{
	ISM330DHCX_IRQ_PushPull = 0,
	ISM330DHCX_IRQ_OpenDrain = 1 << 4
}ISM330DHCX_IRQOutputType_t;

typedef enum{
	ISM330DHCX_SPI4Wire = 0,
	ISM330DHCX_SPI3Wire = 1 << 3
}ISM330DHCX_SerialIFMode_t;

typedef enum{
	ISM330DHCX_AutoIncrementEnabled = 1 << 2,
	ISM330DHCX_AutoIncrementDisable = 0
}ISM330DHCX_AddressAutoIncrement_t;

/*CTRL4 C Reg*/
typedef enum{
	ISM330DHCX_IRQOnBothPins = 1 << 5,
	ISM330DHCX_IRQPinsSeparate = 0
}ISM330DHCX_IRQPinMux_t;

typedef enum{
	ISM330DHCX_DRDY_Msk_Disabled,
	ISM330DHCX_DRDY_Msk_Enabled = 1 << 3
}ISM330DHCX_DRDY_Msk_t;

typedef enum{
	ISM330DHCX_LPF_Disabled = 0,
	ISM330DHCX_LPF_Enabled = 1 << 1
}ISM330DHCX_GyroLPF_t;

/*CTRL5 C Reg Skipped*/
/*CTRL6 C Reg*/
typedef enum{
	ISM330DHCX_DataEnableOff = 0,
	ISM330DHCX_EdgeTriggerMode = 4 << 5,
	ISM330DHCX_LevelTriggerMode = 2 << 5,
	ISM330DHCX_LevelLatchedMode = 3 << 5,
	ISM330DHCX_LevelFIFOMode = 6 << 5
}ISM330DHCX_TriggerMode_t;

typedef enum{
	ISM330DHCX_XL_HPModeDisabled = 1 << 4,
	ISM330DHCX_XL_HPModeEnabled = 0
}ISM330DHCX_AccelHighPerfMode_t;

typedef enum{
	ISM330DHCX_XL_LSBDiv1024 = 0,
	ISM330DHCX_XL_LSBDiv64 = 1 << 3
}ISM330DHCX_AccelOffsetWeight_t;

typedef enum{
	ISM330DHCX_LPF1_000 = 0,
	ISM330DHCX_LPF1_001,
	ISM330DHCX_LPF1_010,
	ISM330DHCX_LPF1_011,
	ISM330DHCX_LPF1_100,
	ISM330DHCX_LPF1_101,
	ISM330DHCX_LPF1_110,
	ISM330DHCX_LPF1_111
}ISM330DHCX_LPF1Bandwidth_t;

/*CTRL 7 G*/
typedef enum{
	ISM330DHCX_G_HPModeEnabled = 0,
	ISM330DHCX_G_HPModeDisabled = 1 << 7
}ISM330DHCX_GyroHighPerfMode_t;

typedef enum{
	ISM330DHCX_G_HP_Disabled = 0,
	ISM330DHCX_G_HP_Enabled = 1 << 6
}ISM330DHCX_GyroHighPass_t;

typedef enum{
	ISM330DHCX_GHP_16mHz = 0,
	ISM330DHCX_GHP_65mHz = 1 << 4,
	ISM330DHCX_GHP_260mHz = 2 << 4,
	ISM330DHCX_GHP_1040mHz = 3 << 4
}ISM330DHCX_GyroHighPassCutoff_t;

typedef enum{
	ISM330DHCX_XL_UserOffsetOutputDisabled = 0,
	ISM330DHCX_XL_UserOffsetOutput = 1 << 1
}ISM330DHCX_AccelUserOffsetOuput_t;

/*CTRL 8 XL*/
typedef enum{
	ISM330DHCX_ODRDiv4 = 0,
	ISM330DHCX_ODRDiv10 = 1 << 5,
	ISM330DHCX_ODRDiv20 = 2 << 5,
	ISM330DHCX_ODRDiv45 = 3 << 5,
	ISM330DHCX_ODRDiv100 = 4 << 5,
	ISM330DHCX_ODRDiv200 = 5 << 5,
	ISM330DHCX_ODRDiv400 = 6 << 5,
	ISM330DHCX_ODRDiv800 = 7 << 5
}ISM330DHCX_LPF2andHPF_BW_t;

typedef enum{
	ISM330DHCX_XL_HighPassRefDisabled = 0,
	ISM330DHCX_XL_HighPassRefEnabled = 1 << 4
}ISM330DHCX_AccelHighPassRefMode_t;

typedef enum{
	ISM330DHCX_XL_FastSettleModeDisabled = 0,
	ISM330DHCX_XL_FastSettleModeEnabled = 1 << 3
}ISM330DHCX_AccelFastSettleMode_t;

typedef enum{
	ISM330DHCX_XL_HighPassOrSlope = 1 << 2,
	ISM330DHCX_XL_LowPass = 0
}ISM330DHCX_AccelFilterSelect_t;

/*CTRL 9 XL Reg*/
typedef enum{
	ISM330DHCX_DEN_On_X_Enabled = 1 << 7,
	ISM330DHCX_DEN_On_X_Disabled = 0
}ISM330DHCX_DataEnableOnX_t;

typedef enum{
	ISM330DHCX_DEN_On_Y_Enabled = 1 << 6,
	ISM330DHCX_DEN_On_Y_Disabled = 0
}ISM330DHCX_DataEnableOnY_t;

typedef enum{
	ISM330DHCX_DEN_On_Z_Enabled = 1 << 5,
	ISM330DHCX_DEN_On_Z_Disabled = 0
}ISM330DHCX_DataEnableOnZ_t;

typedef enum{
	ISM330DHCX_DEN_GyroStamped = 0,
	ISM330DHCX_DEN_XLStamped = 1 << 4
}ISM330DHCX_DataEnablePinStamping_t;

typedef enum{
	ISM330DHCX_DEN_XL_Disabled = 0,
	ISM330DHCX_DEN_XL_Enabled = 1 << 3
}ISM330DHCX_DataEnableForXL_t;

typedef enum{
	ISM330DHCX_DEN_ActiveLow = 0,
	ISM330DHCX_DEN_ActiveHigh = 1 << 2
}ISM330DHCX_DataEnablePolarity_t;

/*CTRL 10 C*/
typedef enum{
	ISM330DHCX_TimeStampEnabled = 1 << 5,
	ISM330DHCX_TimeStampDisabled = 0
}ISM330DHCX_TimeStamp_t;

/* Settings Struct to be provided by the user */
typedef struct{
	/*****Physical Interface Settings*****/
	ISM330DHCX_SDOPullUp_t SDO_PU_EN;                        //PIN_CTRL Reg
	ISM330DHCX_SerialIFMode_t SIM;                           //CTRL3
	ISM330DHCX_AddressAutoIncrement_t IF_INC;                //CTRL3

    /*****Data Enable (DEN) Settings *****/
	ISM330DHCX_TriggerMode_t TriggerMode;                    //CTRL6
	ISM330DHCX_DataEnableOnX_t DEN_X;                        //CTRL9
	ISM330DHCX_DataEnableOnY_t DEN_Y;                        //CTRL9
	ISM330DHCX_DataEnableOnZ_t DEN_Z;                        //CTRL9
	ISM330DHCX_DataEnablePinStamping_t DEN_XL_G;             //CTRL9
	ISM330DHCX_DataEnableForXL_t DEN_XL_EN;                  //CTRL9
	ISM330DHCX_DataEnablePolarity_t DEN_LH;                  //CTRL9

	/*****FIFO Settings*****/
	ISM330DHCX_FifoMode_t FIFO_MODE;                        //FIFO_CTRL4
	ISM330DHCX_WatermarkThreshold WTM;                      //FIFO_CTRL1 and FIFO_CTRL2
	ISM330DHCX_FifoDepth_t STOP_ON_WTM;                     //FIFO_CTRL2
	ISM330DHCX_GyroBatchDataRate_t BDR_GY;                  //FIFO_CTRL3
	ISM330DHCX_AccelBatchDataRate_t BDR_XL;                 //FIFO_CTRL3
	ISM330DHCX_TempBatchDataRate_t ODR_T_BATCH;             //FIFO_CTRL4
	ISM330DHCX_BatchDecimationRate_t DEC_TS_BATCH;          //FIFO_CTRL4
	ISM330DHCX_DataReadyMode_t dataready_pulsed;            //Counter BDR Reg1
	ISM330DHCX_BatchEvtTrigger_t TRIG_COUNTER_BDR;          //Counter BDR Reg1
	ISM330DHCX_BatchDataEvtThreshold CNT_BDR_TH;            //Counter BDR Reg1 + 2

	/*****XL Filter Settings: See Filter Block Diagram on pg 16 and 17 of AN5398*****/
	//Main Filter Settings
	ISM330DHCX_AccelODR_t ODR_XL;                           //CTRL1
	ISM330DHCX_AccelHighPerfMode_t XL_HM_MODE;              //CTRL6
	ISM330DHCX_AccelFilterSelect_t HP_SLOPE_XL_EN;          //CTRL8
	ISM330DHCX_XL_SecondStageLPF_t LPF2_XL_EN;              //CTRL1
	ISM330DHCX_LPF2andHPF_BW_t HPCF_XL;                     //CTRL8
	ISM330DHCX_AccelUserOffsetOuput_t USR_OFF_ON_OUT;       //CTRL7
	ISM330DHCX_AccelOffsetWeight_t USR_OFF_W;               //CTRL6
	int8_t X_OFS_USR;
	int8_t Y_OFS_USR;
	int8_t Z_OFS_USR;
	//Other Filter Settings
	ISM330DHCX_AccelHighPassRefMode_t HP_REF_MODE_XL;       //CTRL8
	ISM330DHCX_AccelFastSettleMode_t FASTSETTL_MODE_XL;     //CTRL8

	/*****XL Sensitivity Setting*****/
	ISM330DHCX_AccelFullscale_t FS_XL;                       //CTRL1

	/*****Gyro Filter Settings: See Filter Block Diagram on pg 20 of AN5398*****/
	ISM330DHCX_GyroHighPassCutoff_t HPM_G;                   //CTRL7
	ISM330DHCX_GyroHighPass_t HP_EN_G;                       //CTRL7
	ISM330DHCX_LPF1Bandwidth_t FTYPE;                        //CTRL6
	ISM330DHCX_GyroLPF_t LPF1_SEL_G;                         //CTRL4
	ISM330DHCX_GyroODR_t ODR_G;                              //CTRL2
	ISM330DHCX_GyroHighPerfMode_t G_HM_MODE;                 //CTRL7

	/*****G Sensitivity Setting*****/
	ISM330DHCX_GyroFullScale_t FS_G;                         //CTRL2

	/*****Sensor Filter Settling Mode (See 4.3.1 of AN5398*****/
	ISM330DHCX_DRDY_Msk_t DRDY_MASK;                         //CTRL4

	/*****IRQ Settings*****/
	ISM330DHCX_IRQPolarity_t H_LACTIVE;                      //CTRL3
	ISM330DHCX_IRQOutputType_t PP_OD;                        //CTRL3
	ISM330DHCX_Int1trigger_t INT1_CTRL;                      //Int1 Ctrl Reg
	ISM330DHCX_Int2trigger_t INT2_CTRL;                      //Int2 Ctrl Reg
	ISM330DHCX_IRQPinMux_t INT2_on_INT1;                     //CTRL4

	/*****Time Stamp Counter Settings*****/
	ISM330DHCX_TimeStamp_t TIMESTAMP_EN;                     //CTRL10

}ISM330DHCX_Init_Struct_t;

//Device Handle definition.
typedef struct{
	ISM330DHCX_IO_t IO;
	ISM330DHCX_Context_t Context;
	ISM330DHCX_GyroFullScale_t GyroResolution;
	ISM330DHCX_AccelFullscale_t AccelResolution;
	ISM330DHCX_Int1trigger_t Int1Mode;
	ISM330DHCX_Int2trigger_t Int2Mode;
	ISM330DHCX_IRQPinMux_t IntPinMux;
	ISM330DHCX_IRQPolarity_t IntPolarity;
	ISM330DHCX_GyroODR_t GyroDataRate;
	ISM330DHCX_AccelODR_t AccelDataRate;
	ISM330DHCX_InitStatus_t Status;
}ISM330DHCX_Handle_t;

int32_t ISM330DHCX_Init(ISM330DHCX_Handle_t *Handle, ISM330DHCX_Init_Struct_t Settings, ISM330DHCX_IO_t *IO);
void ISM330DHCX_DeInit(ISM330DHCX_Handle_t *Handle);
int32_t ISM330DHCX_ResetDevice(ISM330DHCX_Handle_t *Handle, ISM330DHCX_Init_Struct_t Settings, ISM330DHCX_IO_t *IO);
int32_t ISM330DHCX_RegisterIOBus(ISM330DHCX_Handle_t *Handle, ISM330DHCX_IO_t *IO);
int32_t ISM330DHCX_ReadAccel(ISM330DHCX_Handle_t *Handle, float *Ax, float *Ay, float *Az);
int32_t ISM330DHCX_ReadGyro(ISM330DHCX_Handle_t *Handle, float *Wx, float *Wy, float *Wz);
int32_t ISM330DHCX_ReadTemp(ISM330DHCX_Handle_t *Handle, float *Data);
int32_t ISM330DHCX_Reboot(ISM330DHCX_Handle_t *Handle);
int32_t ISM330DHCX_Reset(ISM330DHCX_Handle_t *Handle);

int32_t ISM330DHCX_GetAccelPeriod(ISM330DHCX_Handle_t *Handle, uint32_t *Period);
int32_t ISM330DHCX_GetGyroPeriod(ISM330DHCX_Handle_t *Handle, uint32_t *Period);

#endif /* SRC_EXTERNALHARDWARE_ISM330DHCX_H_ */
