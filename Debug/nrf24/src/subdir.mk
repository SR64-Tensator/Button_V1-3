################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../nrf24/src/nrf24_Button.c 

OBJS += \
./nrf24/src/nrf24_Button.o 

C_DEPS += \
./nrf24/src/nrf24_Button.d 


# Each subdirectory must supply rules for building sources it contributes
nrf24/src/%.o nrf24/src/%.su nrf24/src/%.cyclo: ../nrf24/src/%.c nrf24/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L051xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/sajad.rahmanPour/STM32CubeIDE/Tensator_workspace/Button_V1-3/nrf24/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-nrf24-2f-src

clean-nrf24-2f-src:
	-$(RM) ./nrf24/src/nrf24_Button.cyclo ./nrf24/src/nrf24_Button.d ./nrf24/src/nrf24_Button.o ./nrf24/src/nrf24_Button.su

.PHONY: clean-nrf24-2f-src

