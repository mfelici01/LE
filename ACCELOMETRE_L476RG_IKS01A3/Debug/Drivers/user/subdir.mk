################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/user/lis2dw12_reg.c 

OBJS += \
./Drivers/user/lis2dw12_reg.o 

C_DEPS += \
./Drivers/user/lis2dw12_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/user/%.o Drivers/user/%.su Drivers/user/%.cyclo: ../Drivers/user/%.c Drivers/user/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/taouf/OneDrive/Documents/ACCELOMETRE_L476RG_IKS01A3/Drivers/user" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-user

clean-Drivers-2f-user:
	-$(RM) ./Drivers/user/lis2dw12_reg.cyclo ./Drivers/user/lis2dw12_reg.d ./Drivers/user/lis2dw12_reg.o ./Drivers/user/lis2dw12_reg.su

.PHONY: clean-Drivers-2f-user

