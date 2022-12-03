################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ExternalHardware/HTS221.c \
../Core/Src/ExternalHardware/HTS221_Register.c \
../Core/Src/ExternalHardware/IIS2MDC.c \
../Core/Src/ExternalHardware/IIS2MDC_Registers.c \
../Core/Src/ExternalHardware/ISM330DHCX.c \
../Core/Src/ExternalHardware/ISM330DHCX_Registers.c \
../Core/Src/ExternalHardware/LPS22HH.c \
../Core/Src/ExternalHardware/LPS22HH_Registers.c \
../Core/Src/ExternalHardware/OV5640.c \
../Core/Src/ExternalHardware/VEML6030.c \
../Core/Src/ExternalHardware/VEML6030_Registers.c 

OBJS += \
./Core/Src/ExternalHardware/HTS221.o \
./Core/Src/ExternalHardware/HTS221_Register.o \
./Core/Src/ExternalHardware/IIS2MDC.o \
./Core/Src/ExternalHardware/IIS2MDC_Registers.o \
./Core/Src/ExternalHardware/ISM330DHCX.o \
./Core/Src/ExternalHardware/ISM330DHCX_Registers.o \
./Core/Src/ExternalHardware/LPS22HH.o \
./Core/Src/ExternalHardware/LPS22HH_Registers.o \
./Core/Src/ExternalHardware/OV5640.o \
./Core/Src/ExternalHardware/VEML6030.o \
./Core/Src/ExternalHardware/VEML6030_Registers.o 

C_DEPS += \
./Core/Src/ExternalHardware/HTS221.d \
./Core/Src/ExternalHardware/HTS221_Register.d \
./Core/Src/ExternalHardware/IIS2MDC.d \
./Core/Src/ExternalHardware/IIS2MDC_Registers.d \
./Core/Src/ExternalHardware/ISM330DHCX.d \
./Core/Src/ExternalHardware/ISM330DHCX_Registers.d \
./Core/Src/ExternalHardware/LPS22HH.d \
./Core/Src/ExternalHardware/LPS22HH_Registers.d \
./Core/Src/ExternalHardware/OV5640.d \
./Core/Src/ExternalHardware/VEML6030.d \
./Core/Src/ExternalHardware/VEML6030_Registers.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ExternalHardware/%.o Core/Src/ExternalHardware/%.su: ../Core/Src/ExternalHardware/%.c Core/Src/ExternalHardware/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -DNX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I"C:/Users/evanl/OneDrive/Documents/Embedded_Software_Workspaces/ExampleCode/DroneMQTT/Core/Src/BoardSupportPackage" -IC:/certs/ -I"C:/Users/evanl/OneDrive/Documents/Embedded_Software_Workspaces/ExampleCode/DroneMQTT/Core/Src/ExternalHardware/mx_wifi" -I"C:/Users/evanl/OneDrive/Documents/Embedded_Software_Workspaces/ExampleCode/DroneMQTT/Core/Src/ExternalHardware" -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../NetXDuo/App -I../NetXDuo/Target -I../Middlewares/ST/netxduo/addons/dhcp -I../Middlewares/ST/netxduo/common/inc -I../Middlewares/ST/netxduo/ports/cortex_m33/gnu/inc -I../Middlewares/ST/netxduo/addons/dns -I../Middlewares/ST/netxduo/addons/mqtt -I../Middlewares/ST/netxduo/addons/sntp -I../Middlewares/ST/netxduo/nx_secure/inc -I../Middlewares/ST/netxduo/nx_secure/ports -I../Middlewares/ST/netxduo/crypto_libraries/inc -I../Middlewares/ST/netxduo/crypto_libraries/ports/cortex_m4/gnu/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ExternalHardware

clean-Core-2f-Src-2f-ExternalHardware:
	-$(RM) ./Core/Src/ExternalHardware/HTS221.d ./Core/Src/ExternalHardware/HTS221.o ./Core/Src/ExternalHardware/HTS221.su ./Core/Src/ExternalHardware/HTS221_Register.d ./Core/Src/ExternalHardware/HTS221_Register.o ./Core/Src/ExternalHardware/HTS221_Register.su ./Core/Src/ExternalHardware/IIS2MDC.d ./Core/Src/ExternalHardware/IIS2MDC.o ./Core/Src/ExternalHardware/IIS2MDC.su ./Core/Src/ExternalHardware/IIS2MDC_Registers.d ./Core/Src/ExternalHardware/IIS2MDC_Registers.o ./Core/Src/ExternalHardware/IIS2MDC_Registers.su ./Core/Src/ExternalHardware/ISM330DHCX.d ./Core/Src/ExternalHardware/ISM330DHCX.o ./Core/Src/ExternalHardware/ISM330DHCX.su ./Core/Src/ExternalHardware/ISM330DHCX_Registers.d ./Core/Src/ExternalHardware/ISM330DHCX_Registers.o ./Core/Src/ExternalHardware/ISM330DHCX_Registers.su ./Core/Src/ExternalHardware/LPS22HH.d ./Core/Src/ExternalHardware/LPS22HH.o ./Core/Src/ExternalHardware/LPS22HH.su ./Core/Src/ExternalHardware/LPS22HH_Registers.d ./Core/Src/ExternalHardware/LPS22HH_Registers.o ./Core/Src/ExternalHardware/LPS22HH_Registers.su ./Core/Src/ExternalHardware/OV5640.d ./Core/Src/ExternalHardware/OV5640.o ./Core/Src/ExternalHardware/OV5640.su ./Core/Src/ExternalHardware/VEML6030.d ./Core/Src/ExternalHardware/VEML6030.o ./Core/Src/ExternalHardware/VEML6030.su ./Core/Src/ExternalHardware/VEML6030_Registers.d ./Core/Src/ExternalHardware/VEML6030_Registers.o ./Core/Src/ExternalHardware/VEML6030_Registers.su

.PHONY: clean-Core-2f-Src-2f-ExternalHardware

