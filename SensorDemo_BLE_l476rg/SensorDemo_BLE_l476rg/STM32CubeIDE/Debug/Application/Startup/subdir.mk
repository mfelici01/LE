################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Application/Startup/startup_stm32l476rgtx.s 

OBJS += \
./Application/Startup/startup_stm32l476rgtx.o 

S_DEPS += \
./Application/Startup/startup_stm32l476rgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Application/Startup/%.o: ../Application/Startup/%.s Application/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/taouf/OneDrive/Documents/LE/SensorDemo_BLE_l476rg/SensorDemo_BLE_l476rg/STM32CubeIDE/Drivers/HTS221" -I"C:/Users/taouf/OneDrive/Documents/LE/SensorDemo_BLE_l476rg/SensorDemo_BLE_l476rg/STM32CubeIDE/Drivers/LPS22HH" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Application-2f-Startup

clean-Application-2f-Startup:
	-$(RM) ./Application/Startup/startup_stm32l476rgtx.d ./Application/Startup/startup_stm32l476rgtx.o

.PHONY: clean-Application-2f-Startup

