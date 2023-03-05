################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/port.c 

OBJS += \
./ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/port.o 

C_DEPS += \
./ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/port.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/%.o ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/%.su: ../ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/%.c ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F091xC -c -I../Core/Inc -I"/home/danimt/MyProjects_STM32NucleoF091/Train_FreeRTOS_STM32cube/ThirdParty/FreeRTOS/include" -I"/home/danimt/MyProjects_STM32NucleoF091/Train_FreeRTOS_STM32cube/ThirdParty/FreeRTOS" -I"/home/danimt/MyProjects_STM32NucleoF091/Train_FreeRTOS_STM32cube/ThirdParty/FreeRTOS/portable/GCC/ARM_CM0" -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM0

clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM0:
	-$(RM) ./ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/port.d ./ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/port.o ./ThirdParty/FreeRTOS/portable/GCC/ARM_CM0/port.su

.PHONY: clean-ThirdParty-2f-FreeRTOS-2f-portable-2f-GCC-2f-ARM_CM0

