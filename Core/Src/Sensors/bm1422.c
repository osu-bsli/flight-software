/*
 * fc_bm1422.c
 *
 *  Created on: Jan 22, 2025
 *      Author: bsli
 */

#include "common.h"
#include "stm32h7xx_hal.h"
#include "Sensors/bm1422.h"
#include <SEGGER_RTT.h>
#include <stdio.h>

/*
 * Header files are for sharing things that other C files need.
 * Register addresses should go HERE and not bm1422.h because other C files do not need to see them.
 */

/* I2C constants (Pg. 10) */
#define FC_BM1422_I2C_DEVICE_ID			 (0x0Eu << 1)	// There is a low and high address

// Register constants (pg. 10)
#define REGISTER_INFO		   0x0D   // LSB
#define REGISTER_WIA		   0x0F
#define REGISTER_DATAX		   0x10	  // LSB
#define REGISTER_DATAY		   0x12	  // LSB
#define REGISTER_DATAZ		   0x14	  // LSB
#define REGISTER_STA1	  	   0x18
#define REGISTER_CNTL1		   0x1B
#define REGISTER_CNTL2		   0x1C
#define REGISTER_CNTL3		   0x1D
#define REGISTER_AVE_A		   0x40
#define REGISTER_CNTL4		   0x5C
#define REGISTER_TEMP		   0x60   // LSB
#define REGISTER_OFF_X		   0x6C
#define REGISTER_OFF_Y	  	   0x72
#define REGISTER_OFF_Z		   0x78
#define REGISTER_FINEOUTPUTX	   0x90   // LSB
#define REGISTER_FINEOUTPUTY	   0x92   // LSB
#define REGISTER_FINEOUTPUTZ	   0x94   // LSB
#define REGISTER_GAIN_PARA_X	   0x9C   // LSB
#define REGISTER_GAIN_PARA_Y	   0x9E   // LSB

/*
 * Private functions.
 *
 * Note how these functions are marked static.
 * That means they are inaccessible to other C files. "static" tells the compiler
 * to not export the function as a public symbol.
 *
 * These functions are not prefixed with fc_adxl375_ because they are private
 * and it is obvious what they do.
 */

static HAL_StatusTypeDef read_register(struct fc_bm1422 *device, uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Read_IT(device->i2c_handle, FC_BM1422_I2C_DEVICE_ID, reg, sizeof(reg), data, sizeof(data));
}

static HAL_StatusTypeDef read_registers(struct fc_bm1422 *device, uint8_t reg, uint8_t *data, uint8_t length) {
	return HAL_I2C_Mem_Read_IT(device->i2c_handle, FC_BM1422_I2C_DEVICE_ID, reg, sizeof(reg), data, length);
}

static HAL_StatusTypeDef write_register(struct fc_bm1422 *device, uint8_t reg, uint8_t *data) {
	return HAL_I2C_Mem_Write_IT(device->i2c_handle, FC_BM1422_I2C_DEVICE_ID, reg, sizeof(reg), data, sizeof(data));
}

/*
 * Public functions.
 */

int fc_bm1422_initialize(struct fc_bm1422 *device, I2C_HandleTypeDef *i2c_handle) {

	/* reset struct */
	device->i2c_handle					= i2c_handle;
	device->magnetic_strength_x			= 0.0f;
	device->magnetic_strength_y			= 0.0f;
	device->magnetic_strength_z			= 0.0f;

	/* =================================== */
	/* check that the device id is correct */
	/* =================================== */

	HAL_StatusTypeDef status;
	uint8_t data;

	/* start I2C read with a 2-byte Batch Read */
	/* Use the FIRST Device Address for the batch read */
	status = fc_bm1422_readregisters(device, FC_BM1422_REGISTER_INFORMATION, &data, 2);
	if (status != HAL_OK) {
		return 42;
	}
	if (data != FC_BM1422_I2C_DEVICE_ID) {
		return 255;
	}

	/* wait until i2c read is complete */
	while (HAL_I2C_GetState(i2c_handle) != HAL_I2C_STATE_READY) osDelay(1);

	/* Set the Power Control Bit for Magnometer
	 * Default Setting: 0x22 -> 00100010 (pg.12)*/
	data = 0b01101100u;

	/* Start i2c write */
	status = fc_bm1422_writeregister(device, FC_BM1422_REGISTER_CONTROL1, &data);
	if (status != HAL_OK) {
		return 42;
	}

	/* wait until i2c read is complete */
	while (HAL_I2C_GetState(i2c_handle) != HAL_I2C_STATE_READY) osDelay(1);

	return 0;
}

int fc_bm1422_process(struct fc_bm1422 *device) {

	/* Array for six output data registers (Pg. 12) */
	uint8_t data[6];

	/* Begin i2c read */
	HAL_StatusTypeDef status = fc_bm1422_readregisters(device, FC_BM1422_REGISTER_DATA_X, data, sizeof(data));
	if (status != HAL_OK) {
		return 255;
	}

	/* Wait until i2c read is complete */
	while (HAL_I2C_GetState(device->i2c_handle) != HAL_I2C_STATE_READY) osDelay(1);

	/* CONVERT DATA BYTES TO SIGNED 16-BIT VALUES */
	union {
		uint8_t bytes[2];
		int16_t value;
	} converter;

	converter.bytes[0] = data[0];
	converter.bytes[1] = data[1];
	int16_t raw_magnetic_strength_X = converter.value;

	converter.bytes[0] = data[2];
	converter.bytes[1] = data[3];
	int16_t raw_magnetic_strength_Y = converter.value;

	converter.bytes[0] = data[4];
	converter.bytes[1] = data[5];
	int16_t raw_magnetic_strength_Z = converter.value;

	/* Process Raw Data */
	device->magnetic_strength_x = (float) raw_magnetic_strength_X;
	device->magnetic_strength_y = (float) raw_magnetic_strength_Y;
	device->magnetic_strength_z = (float) raw_magnetic_strength_Z;

	return 0;
}
