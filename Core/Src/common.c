/*
 * common.c
 *
 *  Created on: Mar 24, 2024
 *      Author: bsli
 */

#include "common.h"

uint16_t u8_to_u16(uint8_t lsb, uint8_t msb) {
	union {
		uint8_t bytes[2]; /* little-endian system, bytes[0] is least significant byte */
		uint16_t value;
	} conversion;
	conversion.bytes[0] = lsb;
	conversion.bytes[1] = msb;
	return conversion.value;
}

int16_t u8_to_i16(uint8_t lsb, uint8_t msb) {
	return (msb << 8) | lsb;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	
}