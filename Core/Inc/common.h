/**
 * common.h
 */

#pragma once

#include <stdint.h>
#include <stm32h7xx_hal.h>

#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1

uint16_t u8_to_u16(uint8_t lsb, uint8_t msb);
int16_t u8_to_i16(uint8_t lsb, uint8_t msb);

#define I2C_TIMEOUT_TICKS 100

#define HAL_I2C_Mem_Read_IT_Semaphore(binary_semaphore, hi2c, DevAddress, MemAddress, MemAddSize, pData, Size)  \
    ({                                                                                                          \
        HAL_StatusTypeDef _status = HAL_I2C_Mem_Read_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size); \
        if (xSemaphoreTake(*binary_semaphore, 100) != true)                                                \
        {                                                                                                       \
            SEGGER_RTT_printf(0, SENSOR_NAME ": HAL_I2C_Mem_Read_IT_Semaphore timeout\n");                      \
            _status = HAL_TIMEOUT;                                                                              \
        }                                                                                                       \
        _status;                                                                                                \
    })

#define HAL_I2C_Mem_Write_IT_Semaphore(binary_semaphore, hi2c, DevAddress, MemAddress, MemAddSize, pData, Size)  \
    ({                                                                                                           \
        HAL_StatusTypeDef _status = HAL_I2C_Mem_Write_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size); \
        if (xSemaphoreTake(*binary_semaphore, 100) != true)                                                 \
        {                                                                                                        \
            SEGGER_RTT_printf(0, SENSOR_NAME ": HAL_I2C_Mem_Write_IT_Semaphore timeout\n");                      \
            _status = HAL_TIMEOUT;                                                                               \
        }                                                                                                        \
        _status;                                                                                                 \
    })
