################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/netxduo/addons/sntp/nxd_sntp_client.c 

OBJS += \
./Middlewares/ST/netxduo/addons/sntp/nxd_sntp_client.o 

C_DEPS += \
./Middlewares/ST/netxduo/addons/sntp/nxd_sntp_client.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/netxduo/addons/sntp/%.o Middlewares/ST/netxduo/addons/sntp/%.su: ../Middlewares/ST/netxduo/addons/sntp/%.c Middlewares/ST/netxduo/addons/sntp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U585xx -DTX_INCLUDE_USER_DEFINE_FILE -DTX_SINGLE_MODE_NON_SECURE=1 -DNX_INCLUDE_USER_DEFINE_FILE -c -I../Core/Inc -I"C:/Users/evanl/OneDrive/Documents/Embedded_Software_Workspaces/ExampleCode/DroneMQTT/Core/Src/BoardSupportPackage" -IC:/certs/ -I"C:/Users/evanl/OneDrive/Documents/Embedded_Software_Workspaces/ExampleCode/DroneMQTT/Core/Src/ExternalHardware/mx_wifi" -I"C:/Users/evanl/OneDrive/Documents/Embedded_Software_Workspaces/ExampleCode/DroneMQTT/Core/Src/ExternalHardware" -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -I../AZURE_RTOS/App -I../Middlewares/ST/threadx/common/inc -I../Middlewares/ST/threadx/ports/cortex_m33/gnu/inc -I../NetXDuo/App -I../NetXDuo/Target -I../Middlewares/ST/netxduo/addons/dhcp -I../Middlewares/ST/netxduo/common/inc -I../Middlewares/ST/netxduo/ports/cortex_m33/gnu/inc -I../Middlewares/ST/netxduo/addons/dns -I../Middlewares/ST/netxduo/addons/mqtt -I../Middlewares/ST/netxduo/addons/sntp -I../Middlewares/ST/netxduo/nx_secure/inc -I../Middlewares/ST/netxduo/nx_secure/ports -I../Middlewares/ST/netxduo/crypto_libraries/inc -I../Middlewares/ST/netxduo/crypto_libraries/ports/cortex_m4/gnu/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-netxduo-2f-addons-2f-sntp

clean-Middlewares-2f-ST-2f-netxduo-2f-addons-2f-sntp:
	-$(RM) ./Middlewares/ST/netxduo/addons/sntp/nxd_sntp_client.d ./Middlewares/ST/netxduo/addons/sntp/nxd_sntp_client.o ./Middlewares/ST/netxduo/addons/sntp/nxd_sntp_client.su

.PHONY: clean-Middlewares-2f-ST-2f-netxduo-2f-addons-2f-sntp

