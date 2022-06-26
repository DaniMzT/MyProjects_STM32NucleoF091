################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/trainSPI_Arduino.c 

OBJS += \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/trainSPI_Arduino.o 

C_DEPS += \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/trainSPI_Arduino.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F091RCTx -c -I../Inc -I"C:/Users/danim/STM32CubeIDE/workspace_1.9.0/Train/myDrivers/IncDrivers" -O0 -ffunction-sections -fdata-sections -Wall -specs=rdimon.specs -lc -lrdimon -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/trainSPI_Arduino.d ./Src/trainSPI_Arduino.o ./Src/trainSPI_Arduino.su

.PHONY: clean-Src

