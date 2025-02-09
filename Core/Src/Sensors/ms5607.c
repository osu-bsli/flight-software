/*
 * fc_ms5607.c
 *
 *  Created on: Nov 5, 2023
 *      Author: bsli
 */

#include "Sensors/ms5607.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"

/* i2c constants */
/* THE COMPLEMENT OF THE CSB PIN IS THE LSB OF THE I2C ADDRESS */
#define I2C_ADDRESS 0x76u /* 7 bits, CSB pulled low */

/* TEMPORARY Timeout */
#define I2C_TIMEOUT 100

/* Command Size */
#define COMMAND_SIZE_8BIT 8u

/* constants (pg. 10)
 * These are updated for ms5067*/
#define CONSTANT_RESET 0x1Eu
#define CONSTANT_CONVERTD1_OSR256 0x40u
#define CONSTANT_CONVERTD1_OSR512 0x42u
#define CONSTANT_CONVERTD1_OSR1024 0x44u
#define CONSTANT_CONVERTD1_OSR2048 0x46u
#define CONSTANT_CONVERTD1_OSR4096 0x48u
#define CONSTANT_CONVERTD2_OSR256 0x50u
#define CONSTANT_CONVERTD2_OSR512 0x52u
#define CONSTANT_CONVERTD2_OSR1024 0x54u
#define CONSTANT_CONVERTD2_OSR2048 0x56u
#define CONSTANT_CONVERTD2_OSR4096 0x58u
#define CONSTANT_ADC_READ 0x00u
#define CONSTANT_PROM_READ 0xA0u

/* PROM addresses for conversion constants C1-C6 */
#define CONSTANT_PROM_READC1 0xA2u // Pressure sensitivity
#define CONSTANT_PROM_READC2 0xA4u // Pressure offset
#define CONSTANT_PROM_READC3 0xA6u // Temperature coefficient of pressure sensitivity
#define CONSTANT_PROM_READC4 0xA8u // Temperature coefficient of pressure offset
#define CONSTANT_PROM_READC5 0xAAu // Reference Temperature
#define CONSTANT_PROM_READC6 0xACu // Temperature coefficient of the temperature

/* Max and min reading values to check validity of data collected */
#define CONSTANT_PRESSURE_MIN 10.0f     // minimun pressure is 10mbar
#define CONSTANT_PRESSURE_MAX 1200.0f   // minimun pressure is 10mbar
#define CONSTANT_TEMPERATURE_MIN -40.0f // minimun temperature is -40 degrees Celsius
#define CONSTANT_TEMPERATURE_MAX 85.0f  // maximum temperature is 85 degrees Celsius

/* Double check all data + data lengths */
static HAL_StatusTypeDef write_registers(struct fc_ms5607 *device, uint8_t *data, uint16_t size)
{
    // TODO: Use interrupt mode
    SEGGER_RTT_printf(0, "ms5607 write: size: %d\n", size);
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(device->i2c_handle, (0x76 << 1), data, size, 100);
    return status;
}

static HAL_StatusTypeDef read_registers(struct fc_ms5607 *device, uint8_t *data, uint16_t size)
{
    // TODO: Use interrupt mode
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(device->i2c_handle, I2C_ADDRESS << 1, data, size, 100);
    return status;
}

/* Prom read sequence. Reads C1-C6 */
static HAL_StatusTypeDef fc_ms5607_read_prom(struct fc_ms5607 *device)
{
    HAL_StatusTypeDef status;
    uint8_t data[2];
    uint8_t command;

    /* Send command then read constant data for C1 */
    command = CONSTANT_PROM_READC1;
    status = write_registers(device, &command, sizeof(command));
    if (status != HAL_OK)
    {
        return status;
    }

    /* Read C1 and store */
    status = read_registers(device, data, 2);
    if (status != HAL_OK)
    {
        return status;
    }
    device->C1 = u8_to_u16(data[1], data[0]);

    /* Send command then read constant data for C2 */
    command = CONSTANT_PROM_READC2;
    status = write_registers(device, &command, sizeof(command));
    if (status != HAL_OK)
    {
        return status;
    }

    /* Read C2 and store */
    status = read_registers(device, data, 2);
    if (status != HAL_OK)
    {
        return status;
    }
    device->C2 = u8_to_u16(data[1], data[0]);

    /* Send command then read constant data for C3 */
    command = CONSTANT_PROM_READC3;
    status = write_registers(device, &command, sizeof(command));
    if (status != HAL_OK)
    {
        return status;
    }

    /* Read C3 and store */
    status = read_registers(device, data, 2);
    if (status != HAL_OK)
    {
        return status;
    }
    device->C3 = u8_to_u16(data[1], data[0]);

    /* Send command then read constant data for C4 */
    command = CONSTANT_PROM_READC4;
    status = write_registers(device, &command, sizeof(command));
    if (status != HAL_OK)
    {
        return status;
    }

    /* Read C4 and store */
    status = read_registers(device, data, 2);
    if (status != HAL_OK)
    {
        return status;
    }
    device->C4 = u8_to_u16(data[1], data[0]);

    /* Send command then read constant data for C5 */
    command = CONSTANT_PROM_READC5;
    status = write_registers(device, &command, sizeof(command));
    if (status != HAL_OK)
    {
        return status;
    }

    /* Read C5 and store */
    status = read_registers(device, data, 2);
    if (status != HAL_OK)
    {
        return status;
    }
    device->C5 = u8_to_u16(data[1], data[0]);

    /* Send command then read constant data for C6 */
    command = CONSTANT_PROM_READC6;
    status = write_registers(device, &command, sizeof(command));
    if (status != HAL_OK)
    {
        return status;
    }

    /* Read C6 and store. Return final status */
    status = read_registers(device, data, 2);
    if (status != HAL_OK)
    {
        return status;
    }
    device->C6 = u8_to_u16(data[1], data[0]);

    return HAL_OK;
}

/* Reset command for barometer */
/* Return Status */
static HAL_StatusTypeDef fc_ms5607_reset(struct fc_ms5607 *device)
{
    /* Send reset command */

    // Write reset command
    //	uint8_t data = FC_MS5607_CONSTANT_RESET;
    //	HAL_StatusTypeDef status = fc_ms5607_transmit(device, &data, FC_8BIT_COMMAND_SIZE);
    //	if(status != HAL_OK){
    //		return status;
    //	}

    /* Must read PROM once after a reset occurs */
    return fc_ms5607_read_prom(device);
}

/* Initialize MS5607 barometer I2C device */
HAL_StatusTypeDef fc_ms5607_initialize(struct fc_ms5607 *device, I2C_HandleTypeDef *i2c_handle)
{

    /* reset struct */
    device->i2c_handle = i2c_handle;
    device->C1 = 0;
    device->C2 = 0;
    device->C3 = 0;
    device->C4 = 0;
    device->C5 = 0;
    device->C6 = 0;

    /* Initiate reset sequence to calibrate PROM */
    HAL_StatusTypeDef reset_status = fc_ms5607_reset(device);
    if (reset_status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "ms5607: reset failed\n");
        return reset_status;
    }

    // Initialization succeeded
    SEGGER_RTT_printf(0, "ms5607: init succeeded\n");
    return HAL_OK;
}

/* Process to read and convert pressure and temperature */
HAL_StatusTypeDef fc_ms5607_process(struct fc_ms5607 *device, struct fc_ms5607_data *data)
{
    HAL_StatusTypeDef status;
    uint8_t command;
    uint8_t zero = 0x00u;
    uint32_t delay = 3; /* freertos ticks */

    /* ===================== */
    /* read temperature data */
    /* ===================== */

    /* Start temperature conversion */
    command = CONSTANT_CONVERTD2_OSR1024; // use highest OSR for now
    status = write_registers(device, &command, 1);
    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "ms5607: error: 1: %d\n", status);
        return status;
    }

    osDelay(delay);

    /* begin reading temperature */
    status = write_registers(device, &zero, 1);
    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "ms5607: error 2\n");
        return status;
    }

    /* Read and store digital temperature data */
    uint8_t temp_bytes[3]; // Big-endian byte 0 = 23-16 byte 1 = 8-15 byte 2 = 7-0
    status = read_registers(device, temp_bytes, 3);
    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "ms5607: error 3\n");
        return status;
    }

    // Convert temperature bytes into temperature digital data
    uint32_t D2 = (temp_bytes[0] << 16) | (temp_bytes[1] << 8) | (temp_bytes[2]);
    int32_t dT = D2 - ((uint32_t)device->C5 * 256);  // D2 - T_ref
                                                     //	device->dT = device->D2 - (device->C5 << 8);					// D2 - T_ref
                                                     //    device->dT = 68; // TODO: remove
    int32_t TEMP = 2000 + ((dT * device->C6) >> 23); // 20.0 C + dT * TEMPSENS (2000+dT*C6/2^23)
    data->temperature_c = TEMP / 100.0f;             // Convert to Celsius

    /* ===================== */
    /* read pressure data */
    /* ===================== */

    /* start pressure conversion */
    command = CONSTANT_CONVERTD1_OSR1024; // use highest OSR for now
    status = write_registers(device, &command, 1);
    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "ms5607: error 4\n");
        return status;
    }

    // TODO: Find a way to get rid of this osDelay :vomit:
    osDelay(delay);

    /* begin reading temperature */
    status = write_registers(device, &zero, 1);
    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "ms5607: error 5\n");
        return status;
    }

    /* Access conversion data by sending a read command. 24 SCLK cycles to receive all bits. */
    uint8_t pressure_bytes[3]; // Big-endian byte 0 = 23-16 byte 1 = 8-15 byte 2 = 7-0
    status = read_registers(device, pressure_bytes, 3);
    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "ms5607: error 6\n");
        return status;
    }

    // Convert temperature bytes into temperature digital data
    uint32_t D1 = (pressure_bytes[0] << 16) | (pressure_bytes[1] << 8) | (pressure_bytes[2]);

    // Determine Constants based on temperature
    // High Temperature
    int32_t OFF = (((int64_t)device->C2) << 17) + (((int64_t)device->C4 * (int64_t)dT) >> 6);
    int32_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    int32_t SENS =
        (((int64_t)device->C1) << 16) +
        ((
             (int64_t)device->C3 *
             (int64_t)dT) >>
         7);
    //    device->SENS = 3039050829; // TODO: remove

    // Low Temperature
    if (TEMP < 2000)
    {
        T2 = (int32_t)(((int64_t)dT * (int64_t)dT) >> 31);
        OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) >> 4;
        SENS2 = 2 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000);

        // Very low temperature
        if (TEMP < -1500)
        {
            OFF2 += 15 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
            SENS2 += 8 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
        }
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }
    else
    { // high temperature
        T2 = 0;
        OFF2 = 0;
        SENS2 = 0;
    }

    int64_t P = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;
    data->pressure_mbar = (float)P / 100.0f;

    // Check validity of conversions - values must be between min and max values on data sheet

    if (data->pressure_mbar > 10.0f || data->pressure_mbar < 1200.0f)
    {
        SEGGER_RTT_printf(0, "ms5607: pressure_mbar out of range\n");
        return HAL_ERROR;
    }
    if (data->temperature_c > 85.0f || data->temperature_c < 40.0f)
    {
        SEGGER_RTT_printf(0, "ms5607: temperature_c out of range\n");
        return HAL_ERROR;
    }

    char buf[64];
    SEGGER_RTT_printf(0, "ms5607: process\n");
    sprintf(buf, "%f", data->pressure_mbar);
    SEGGER_RTT_printf(0, "ms5607: pressure_mbar: %s\n", buf);
    sprintf(buf, "%f", data->temperature_c);
    SEGGER_RTT_printf(0, "ms5607: temperature_c: %s\n", buf);

    return HAL_OK;
}
