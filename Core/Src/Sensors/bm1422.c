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
#define I2C_ADDRESS (0x0Eu << 1) // There is a low and high address
#define WHO_AM_I 0x41

// Register constants (pg. 10)
#define REGISTER_INFO 0x0D // LSB
#define REGISTER_WIA 0x0F
#define REGISTER_DATAX 0x10 // LSB
#define REGISTER_DATAY 0x12 // LSB
#define REGISTER_DATAZ 0x14 // LSB
#define REGISTER_STA1 0x18
#define REGISTER_CNTL1 0x1B
#define REGISTER_CNTL2 0x1C
#define REGISTER_CNTL3 0x1D
#define REGISTER_AVE_A 0x40
#define REGISTER_CNTL4 0x5C
#define REGISTER_TEMP 0x60 // LSB
#define REGISTER_OFF_X 0x6C
#define REGISTER_OFF_Y 0x72
#define REGISTER_OFF_Z 0x78
#define REGISTER_FINEOUTPUTX 0x90 // LSB
#define REGISTER_FINEOUTPUTY 0x92 // LSB
#define REGISTER_FINEOUTPUTZ 0x94 // LSB
#define REGISTER_GAIN_PARA_X 0x9C // LSB
#define REGISTER_GAIN_PARA_Y 0x9E // LSB

#define SENSOR_NAME "bm1422"

static HAL_StatusTypeDef read_registers(struct fc_bm1422 *device, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read_IT_Semaphore(device->i2c_semaphore, device->i2c_handle, I2C_ADDRESS, reg, sizeof(reg), data, length);
}

static HAL_StatusTypeDef write_registers(struct fc_bm1422 *device, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Write_IT_Semaphore(device->i2c_semaphore, device->i2c_handle, I2C_ADDRESS, reg, sizeof(reg), data, length);
}

/*
 * Public functions.
 */

HAL_StatusTypeDef fc_bm1422_initialize(struct fc_bm1422 *device, I2C_HandleTypeDef *i2c_handle, SemaphoreHandle_t *i2c_semaphore)
{
	device->i2c_handle = i2c_handle;
	device->i2c_semaphore = i2c_semaphore;
	device->isInDegradedState = false;

	/* =================================== */
	/* check that the device id is correct */
	/* =================================== */

	HAL_StatusTypeDef status;
	uint8_t data;

	status = read_registers(device, REGISTER_WIA, &data, 1);
	if (status != HAL_OK)
	{
		goto error;
	}
	if (data != WHO_AM_I)
	{
		SEGGER_RTT_printf(0, SENSOR_NAME ": WHO_AM_I mismatch: %d\n", data);
		status = HAL_ERROR;
		goto error;
	}

	/* Set the Power Control Bit for Magnometer
	 * Default Setting: 0x22 -> 00100010 (pg.12)*/
	data = 0b01101100u;

	/* Start i2c write */
	status = write_registers(device, REGISTER_CNTL1, &data, 1);
	if (status != HAL_OK)
	{
		SEGGER_RTT_printf(0, SENSOR_NAME ": control write failed\n");
		goto error;
	}

	return HAL_OK;

error:
	device->isInDegradedState = true;
	return status;
}

HAL_StatusTypeDef fc_bm1422_process(struct fc_bm1422 *device, struct fc_bm1422_data *data)
{
	/* Array for six output data registers (Pg. 12) */
	uint8_t raw_data[6];

	/* Begin i2c read */
	HAL_StatusTypeDef status = read_registers(device, REGISTER_DATAX, raw_data, sizeof(raw_data));
	if (status != HAL_OK)
	{
		SEGGER_RTT_printf(0, SENSOR_NAME ": read failure\n");
		goto error;
	}

	int16_t raw_magnetic_strength_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
	int16_t raw_magnetic_strength_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
	int16_t raw_magnetic_strength_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

	/* Process Raw Data */
	data->magnetic_strength_x = (float)raw_magnetic_strength_x;
	data->magnetic_strength_y = (float)raw_magnetic_strength_y;
	data->magnetic_strength_z = (float)raw_magnetic_strength_z;

	char buf[64];
	snprintf(buf, 64, SENSOR_NAME ": mag x: %f\n", data->magnetic_strength_x);
	SEGGER_RTT_WriteString(0, buf);
	snprintf(buf, 64, SENSOR_NAME ": mag y: %f\n", data->magnetic_strength_y);
	SEGGER_RTT_WriteString(0, buf);
	snprintf(buf, 64, SENSOR_NAME ": mag z: %f\n", data->magnetic_strength_z);
	SEGGER_RTT_WriteString(0, buf);

	return HAL_OK;

error:
	device->isInDegradedState = true;
	return status;
}
