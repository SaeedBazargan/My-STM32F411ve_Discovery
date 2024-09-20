################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/Inc/fonts.c \
../Library/Inc/ssd1306.c 

OBJS += \
./Library/Inc/fonts.o \
./Library/Inc/ssd1306.o 

C_DEPS += \
./Library/Inc/fonts.d \
./Library/Inc/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Library/Inc/%.o Library/Inc/%.su Library/Inc/%.cyclo: ../Library/Inc/%.c Library/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Bazar/Desktop/STM32/STM32F411Discovery/My-STM32F411ve_Discovery/STM32F411Discovery_OLED_ssd1306/OLED_ssd1306/Library/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Library-2f-Inc

clean-Library-2f-Inc:
	-$(RM) ./Library/Inc/fonts.cyclo ./Library/Inc/fonts.d ./Library/Inc/fonts.o ./Library/Inc/fonts.su ./Library/Inc/ssd1306.cyclo ./Library/Inc/ssd1306.d ./Library/Inc/ssd1306.o ./Library/Inc/ssd1306.su

.PHONY: clean-Library-2f-Inc

