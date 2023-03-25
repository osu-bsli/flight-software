################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../custom/accelerometer_1.c \
../custom/accelerometer_2.c \
../custom/arming.c \
../custom/barometer.c \
../custom/data.c \
../custom/magnetometer.c \
../custom/process.c \
../custom/recovery.c \
../custom/telemetry.c 

OBJS += \
./custom/accelerometer_1.o \
./custom/accelerometer_2.o \
./custom/arming.o \
./custom/barometer.o \
./custom/data.o \
./custom/magnetometer.o \
./custom/process.o \
./custom/recovery.o \
./custom/telemetry.o 

C_DEPS += \
./custom/accelerometer_1.d \
./custom/accelerometer_2.d \
./custom/arming.d \
./custom/barometer.d \
./custom/data.d \
./custom/magnetometer.d \
./custom/process.d \
./custom/recovery.d \
./custom/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
custom/%.o custom/%.su custom/%.cyclo: ../custom/%.c custom/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../custom -I../lib -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-custom

clean-custom:
	-$(RM) ./custom/accelerometer_1.cyclo ./custom/accelerometer_1.d ./custom/accelerometer_1.o ./custom/accelerometer_1.su ./custom/accelerometer_2.cyclo ./custom/accelerometer_2.d ./custom/accelerometer_2.o ./custom/accelerometer_2.su ./custom/arming.cyclo ./custom/arming.d ./custom/arming.o ./custom/arming.su ./custom/barometer.cyclo ./custom/barometer.d ./custom/barometer.o ./custom/barometer.su ./custom/data.cyclo ./custom/data.d ./custom/data.o ./custom/data.su ./custom/magnetometer.cyclo ./custom/magnetometer.d ./custom/magnetometer.o ./custom/magnetometer.su ./custom/process.cyclo ./custom/process.d ./custom/process.o ./custom/process.su ./custom/recovery.cyclo ./custom/recovery.d ./custom/recovery.o ./custom/recovery.su ./custom/telemetry.cyclo ./custom/telemetry.d ./custom/telemetry.o ./custom/telemetry.su

.PHONY: clean-custom

