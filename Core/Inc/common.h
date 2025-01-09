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