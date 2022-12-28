################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../myDrivers/SrcDrivers/basicTimers.c \
../myDrivers/SrcDrivers/gpio.c \
../myDrivers/SrcDrivers/i2c.c \
../myDrivers/SrcDrivers/spi.c 

OBJS += \
./myDrivers/SrcDrivers/basicTimers.o \
./myDrivers/SrcDrivers/gpio.o \
./myDrivers/SrcDrivers/i2c.o \
./myDrivers/SrcDrivers/spi.o 

C_DEPS += \
./myDrivers/SrcDrivers/basicTimers.d \
./myDrivers/SrcDrivers/gpio.d \
./myDrivers/SrcDrivers/i2c.d \
./myDrivers/SrcDrivers/spi.d 


# Each subdirectory must supply rules for building sources it contributes
myDrivers/SrcDrivers/%.o myDrivers/SrcDrivers/%.su: ../myDrivers/SrcDrivers/%.c myDrivers/SrcDrivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F091RCTx -c -I../Inc -I"/home/danimt/MyProjects_STM32NucleoF091/Train/myDrivers/IncDrivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-myDrivers-2f-SrcDrivers

clean-myDrivers-2f-SrcDrivers:
	-$(RM) ./myDrivers/SrcDrivers/basicTimers.d ./myDrivers/SrcDrivers/basicTimers.o ./myDrivers/SrcDrivers/basicTimers.su ./myDrivers/SrcDrivers/gpio.d ./myDrivers/SrcDrivers/gpio.o ./myDrivers/SrcDrivers/gpio.su ./myDrivers/SrcDrivers/i2c.d ./myDrivers/SrcDrivers/i2c.o ./myDrivers/SrcDrivers/i2c.su ./myDrivers/SrcDrivers/spi.d ./myDrivers/SrcDrivers/spi.o ./myDrivers/SrcDrivers/spi.su

.PHONY: clean-myDrivers-2f-SrcDrivers

