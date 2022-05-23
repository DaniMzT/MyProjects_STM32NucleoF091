################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../myDrivers/SrcDrivers/gpio.c 

OBJS += \
./myDrivers/SrcDrivers/gpio.o 

C_DEPS += \
./myDrivers/SrcDrivers/gpio.d 


# Each subdirectory must supply rules for building sources it contributes
myDrivers/SrcDrivers/%.o myDrivers/SrcDrivers/%.su: ../myDrivers/SrcDrivers/%.c myDrivers/SrcDrivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F091RCTx -c -I../Inc -I"C:/Users/danim/STM32CubeIDE/workspace_1.9.0/Train_ButtonsLeds/myDrivers/IncDrivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-myDrivers-2f-SrcDrivers

clean-myDrivers-2f-SrcDrivers:
	-$(RM) ./myDrivers/SrcDrivers/gpio.d ./myDrivers/SrcDrivers/gpio.o ./myDrivers/SrcDrivers/gpio.su

.PHONY: clean-myDrivers-2f-SrcDrivers

