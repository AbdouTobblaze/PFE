################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MPU9250_DRV/Src/MPU9250.c 

OBJS += \
./MPU9250_DRV/Src/MPU9250.o 

C_DEPS += \
./MPU9250_DRV/Src/MPU9250.d 


# Each subdirectory must supply rules for building sources it contributes
MPU9250_DRV/Src/MPU9250.o: ../MPU9250_DRV/Src/MPU9250.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/abdou/Documents/M2 SEM/PFE/Analyse vibratoire/Programmes_mcu/Reading_accel/MPU9250_DRV/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MPU9250_DRV/Src/MPU9250.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

