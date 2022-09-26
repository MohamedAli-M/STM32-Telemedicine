################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MAX30102/MAX30102.c \
../Core/Src/MAX30102/algorithm.c 

OBJS += \
./Core/Src/MAX30102/MAX30102.o \
./Core/Src/MAX30102/algorithm.o 

C_DEPS += \
./Core/Src/MAX30102/MAX30102.d \
./Core/Src/MAX30102/algorithm.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MAX30102/%.o Core/Src/MAX30102/%.su: ../Core/Src/MAX30102/%.c Core/Src/MAX30102/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-MAX30102

clean-Core-2f-Src-2f-MAX30102:
	-$(RM) ./Core/Src/MAX30102/MAX30102.d ./Core/Src/MAX30102/MAX30102.o ./Core/Src/MAX30102/MAX30102.su ./Core/Src/MAX30102/algorithm.d ./Core/Src/MAX30102/algorithm.o ./Core/Src/MAX30102/algorithm.su

.PHONY: clean-Core-2f-Src-2f-MAX30102

