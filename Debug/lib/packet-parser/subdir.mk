################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/packet-parser/buffer.c \
../lib/packet-parser/crc.c \
../lib/packet-parser/packet.c \
../lib/packet-parser/parser.c 

OBJS += \
./lib/packet-parser/buffer.o \
./lib/packet-parser/crc.o \
./lib/packet-parser/packet.o \
./lib/packet-parser/parser.o 

C_DEPS += \
./lib/packet-parser/buffer.d \
./lib/packet-parser/crc.d \
./lib/packet-parser/packet.d \
./lib/packet-parser/parser.d 


# Each subdirectory must supply rules for building sources it contributes
lib/packet-parser/%.o lib/packet-parser/%.su lib/packet-parser/%.cyclo: ../lib/packet-parser/%.c lib/packet-parser/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4P5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../custom -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-packet-2d-parser

clean-lib-2f-packet-2d-parser:
	-$(RM) ./lib/packet-parser/buffer.cyclo ./lib/packet-parser/buffer.d ./lib/packet-parser/buffer.o ./lib/packet-parser/buffer.su ./lib/packet-parser/crc.cyclo ./lib/packet-parser/crc.d ./lib/packet-parser/crc.o ./lib/packet-parser/crc.su ./lib/packet-parser/packet.cyclo ./lib/packet-parser/packet.d ./lib/packet-parser/packet.o ./lib/packet-parser/packet.su ./lib/packet-parser/parser.cyclo ./lib/packet-parser/parser.d ./lib/packet-parser/parser.o ./lib/packet-parser/parser.su

.PHONY: clean-lib-2f-packet-2d-parser

