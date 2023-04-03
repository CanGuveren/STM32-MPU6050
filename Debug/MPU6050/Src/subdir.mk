################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MPU6050/Src/mpu6050.c 

OBJS += \
./MPU6050/Src/mpu6050.o 

C_DEPS += \
./MPU6050/Src/mpu6050.d 


# Each subdirectory must supply rules for building sources it contributes
MPU6050/Src/%.o MPU6050/Src/%.su: ../MPU6050/Src/%.c MPU6050/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/EmbeddedDrivers/MPU6050/MPU6050/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MPU6050-2f-Src

clean-MPU6050-2f-Src:
	-$(RM) ./MPU6050/Src/mpu6050.d ./MPU6050/Src/mpu6050.o ./MPU6050/Src/mpu6050.su

.PHONY: clean-MPU6050-2f-Src

