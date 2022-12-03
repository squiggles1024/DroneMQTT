################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ExternalHardware/mx_wifi/mx_wifi.c 

OBJS += \
./Core/Src/ExternalHardware/mx_wifi/mx_wifi.o 

C_DEPS += \
./Core/Src/ExternalHardware/mx_wifi/mx_wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ExternalHardware/mx_wifi/%.o Core/Src/ExternalHardware/mx_wifi/%.su: ../Core/Src/ExternalHardware/mx_wifi/%.c Core/Src/ExternalHardware/mx_wifi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -DNX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I"C:/Users/evanl/OneDrive/Documents/Embedded_Software_Workspaces/ExampleCode/DroneMQTT/Core/Src/BoardSupportPackage" -IC:/certs/ -I"C:/Users/evanl/OneDrive/Documents/Embedded_Software_Workspaces/ExampleCode/DroneMQTT/Core/Src/ExternalHardware/mx_wifi" -I"C:/Users/evanl/OneDrive/Documents/Embedded_Software_Workspaces/ExampleCode/DroneMQTT/Core/Src/ExternalHardware" -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../NetXDuo/App -I../NetXDuo/Target -I../Middlewares/ST/netxduo/addons/dhcp -I../Middlewares/ST/netxduo/common/inc -I../Middlewares/ST/netxduo/ports/cortex_m33/gnu/inc -I../Middlewares/ST/netxduo/addons/dns -I../Middlewares/ST/netxduo/addons/mqtt -I../Middlewares/ST/netxduo/addons/sntp -I../Middlewares/ST/netxduo/nx_secure/inc -I../Middlewares/ST/netxduo/nx_secure/ports -I../Middlewares/ST/netxduo/crypto_libraries/inc -I../Middlewares/ST/netxduo/crypto_libraries/ports/cortex_m4/gnu/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ExternalHardware-2f-mx_wifi

clean-Core-2f-Src-2f-ExternalHardware-2f-mx_wifi:
	-$(RM) ./Core/Src/ExternalHardware/mx_wifi/mx_wifi.d ./Core/Src/ExternalHardware/mx_wifi/mx_wifi.o ./Core/Src/ExternalHardware/mx_wifi/mx_wifi.su

.PHONY: clean-Core-2f-Src-2f-ExternalHardware-2f-mx_wifi

