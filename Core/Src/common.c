/*
 * common.c
 *
 *  Created on: Mar 24, 2024
 *      Author: bsli
 */

#include "common.h"

uint16_t u8_to_u16(uint8_t lsb, uint8_t msb) {
	return (msb << 8) | lsb;
}

int16_t u8_to_i16(uint8_t lsb, uint8_t msb) {
	return (msb << 8) | lsb;
}

