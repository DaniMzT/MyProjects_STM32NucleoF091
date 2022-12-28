################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/sysmem.c \
../Src/train_LCD_timers.c 

OBJS += \
./Src/sysmem.o \
./Src/train_LCD_timers.o 

C_DEPS += \
./Src/sysmem.d \
./Src/train_LCD_timers.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F091RCTx -c -I../Inc -I"/home/danimt/MyProjects_STM32NucleoF091/Train/myDrivers/IncDrivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/train_LCD_timers.d ./Src/train_LCD_timers.o ./Src/train_LCD_timers.su

.PHONY: clean-Src

