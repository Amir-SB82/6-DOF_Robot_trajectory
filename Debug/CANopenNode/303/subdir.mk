################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CANopenNode/303/CO_LEDs.c 

OBJS += \
./CANopenNode/303/CO_LEDs.o 

C_DEPS += \
./CANopenNode/303/CO_LEDs.d 


# Each subdirectory must supply rules for building sources it contributes
CANopenNode/303/%.o CANopenNode/303/%.su CANopenNode/303/%.cyclo: ../CANopenNode/303/%.c CANopenNode/303/subdir.mk
	arm-none-eabi-gcc -gdwarf-4 "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../CANopenNode -I"../Core/CANopenNode_STM32" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CANopenNode-2f-303

clean-CANopenNode-2f-303:
	-$(RM) ./CANopenNode/303/CO_LEDs.cyclo ./CANopenNode/303/CO_LEDs.d ./CANopenNode/303/CO_LEDs.o ./CANopenNode/303/CO_LEDs.su

.PHONY: clean-CANopenNode-2f-303
