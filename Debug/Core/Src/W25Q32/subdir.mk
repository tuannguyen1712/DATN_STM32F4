################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/W25Q32/W25Q32.c 

OBJS += \
./Core/Src/W25Q32/W25Q32.o 

C_DEPS += \
./Core/Src/W25Q32/W25Q32.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/W25Q32/%.o Core/Src/W25Q32/%.su Core/Src/W25Q32/%.cyclo: ../Core/Src/W25Q32/%.c Core/Src/W25Q32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-W25Q32

clean-Core-2f-Src-2f-W25Q32:
	-$(RM) ./Core/Src/W25Q32/W25Q32.cyclo ./Core/Src/W25Q32/W25Q32.d ./Core/Src/W25Q32/W25Q32.o ./Core/Src/W25Q32/W25Q32.su

.PHONY: clean-Core-2f-Src-2f-W25Q32

