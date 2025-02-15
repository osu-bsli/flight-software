/**
 * ms5607.c
 *
 * MS5607 barometer driver.
 *
 * @authors
 * - Dawn Goorskey
 * - Brian Jia
 * - Amber Phillips
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
 * These are updated for ms5067
 * OSR = oversampling ratio
 */
#define COMMAND_RESET 0x1Eu
#define COMMAND_CONVERTD1_OSR256 0x40u
#define COMMAND_CONVERTD1_OSR512 0x42u
#define COMMAND_CONVERTD1_OSR1024 0x44u
#define COMMAND_CONVERTD1_OSR2048 0x46u
#define COMMAND_CONVERTD1_OSR4096 0x48u
#define COMMAND_CONVERTD2_OSR256 0x50u
#define COMMAND_CONVERTD2_OSR512 0x52u
#define COMMAND_CONVERTD2_OSR1024 0x54u
#define COMMAND_CONVERTD2_OSR2048 0x56u
#define COMMAND_CONVERTD2_OSR4096 0x58u
#define COMMAND_ADC_READ 0x00u
#define COMMAND_PROM_READ 0xA0u

/* Max and min reading values to check validity of data collected */
#define CONSTANT_PRESSURE_MIN 10.0f     // minimun pressure is 10mbar
#define CONSTANT_PRESSURE_MAX 1200.0f   // minimun pressure is 10mbar
#define CONSTANT_TEMPERATURE_MIN -40.0f // minimun temperature is -40 degrees Celsius
#define CONSTANT_TEMPERATURE_MAX 85.0f  // maximum temperature is 85 degrees Celsius

/* Double check all data + data lengths */
static HAL_StatusTypeDef write_registers(struct fc_ms5607 *device, uint8_t *data, uint16_t size)
{
    // TODO: Use interrupt mode
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(device->i2c_handle, (0x76 << 1), data, size, 100);
    if (status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "ms5607 write_registers HAL error: %d\n", status);
        SEGGER_RTT_printf(0, "ms5607 write_registers I2C error: %d\n", HAL_I2C_GetError(device->i2c_handle));
    }
    return status;
}

static HAL_StatusTypeDef read_registers(struct fc_ms5607 *device, uint8_t *data, uint16_t size)
{
    // TODO: Use interrupt mode
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(device->i2c_handle, I2C_ADDRESS << 1, data, size, 100);
    return status;
}

/* PROM read sequence. Reads C1-C6. */
static HAL_StatusTypeDef fc_ms5607_read_prom(struct fc_ms5607 *device)
{
    HAL_StatusTypeDef status;
    
    for (int i = 0; i <= 6; i++) {
        uint8_t command = COMMAND_PROM_READ | (i << 1);
        status = write_registers(device, &command, 1);
        if (status != HAL_OK) {
            return status;
        }
        
        uint8_t data[2];
        status = read_registers(device, data, 2);
        if (status != HAL_OK) {
            return status;
        }

        device->C[i] = u8_to_u16(data[1], data[0]);
    }

    // SEGGER_RTT_printf(0, "ms5607: prom C1: %d\n", device->C[1]);
    // SEGGER_RTT_printf(0, "ms5607: prom C2: %d\n", device->C[2]);
    // SEGGER_RTT_printf(0, "ms5607: prom C3: %d\n", device->C[3]);
    // SEGGER_RTT_printf(0, "ms5607: prom C4: %d\n", device->C[4]);
    // SEGGER_RTT_printf(0, "ms5607: prom C5: %d\n", device->C[5]);
    // SEGGER_RTT_printf(0, "ms5607: prom C6: %d\n", device->C[6]);

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
    device->C[0] = 0;
    device->C[1] = 0;
    device->C[2] = 0;
    device->C[3] = 0;
    device->C[4] = 0;
    device->C[5] = 0;
    device->C[6] = 0;

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

    /* ===================== */
    /* read temperature data */
    /* ===================== */

    /* Start temperature conversion */
    command = COMMAND_CONVERTD2_OSR4096; // use highest OSR for now
    status = write_registers(device, &command, 1);
    if (status != HAL_OK)
    {
        return status;
    }

    // TODO: Find a way to get rid of this osDelay :vomit:
    osDelay(20);

    /* begin reading temperature */
    command = COMMAND_ADC_READ;
    status = write_registers(device, &command, 1);
    if (status != HAL_OK)
    {
        return status;
    }


    /* Read and store digital temperature data */
    uint8_t temp_bytes[3]; // Big-endian byte 0 = 23-16 byte 1 = 8-15 byte 2 = 7-0
    status = read_registers(device, temp_bytes, 3);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 
     * TEMPERATURE CALCULATION (p. 8)
     */

    uint32_t D2 = (temp_bytes[0] << 16) | (temp_bytes[1] << 8) | (temp_bytes[2]);
    int32_t dT = D2 - ((int32_t)device->C[5] * 256);  // D2 - T_ref
    int32_t TEMP = 2000 + (((int64_t)dT * device->C[6]) >> 23); // 20.0 C + dT * TEMPSENS (2000+dT*C6/2^23)

    /* start pressure conversion */
    command = COMMAND_CONVERTD1_OSR4096; // use highest OSR for now
    status = write_registers(device, &command, 1);
    if (status != HAL_OK)
    {
        return status;
    }

    // TODO: Find a way to get rid of this osDelay :vomit:
    osDelay(20);

    /* begin reading pressure */
    command = COMMAND_ADC_READ;
    status = write_registers(device, &command, 1);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Access conversion data by sending a read command. 24 SCLK cycles to receive all bits. */
    uint8_t pressure_bytes[3]; // Big-endian byte 0 = 23-16 byte 1 = 8-15 byte 2 = 7-0
    status = read_registers(device, pressure_bytes, 3);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 
     * PRESSURE CALCULATION (p. 8)
     */

    uint32_t D1 = (pressure_bytes[0] << 16) | (pressure_bytes[1] << 8) | (pressure_bytes[2]);
    int64_t OFF = (((int64_t)device->C[2]) << 17) + (((int64_t)device->C[4] * (int64_t)dT) >> 6);
    int64_t SENS = (((int64_t)device->C[1]) << 16) + (((int64_t)device->C[3] * (int64_t)dT) >> 7);

    /*
     * SECOND ORDER TEMPERATURE COMPENSATION (p. 9)
     */

    int64_t T2, OFF2, SENS2;
    // Low Temperature
    if (TEMP < 2000)
    {
        T2 = ((int64_t)dT * (int64_t)dT) >> 31;
        OFF2 = 61 * ((int64_t)(TEMP - 2000) * (int64_t)(TEMP - 2000)) >> 4;
        SENS2 = 2 * ((int64_t)(TEMP - 2000) * (int64_t)(TEMP - 2000));

        // Very low temperature
        if (TEMP < -1500)
        {
            OFF2 += 15 * ((int64_t)(TEMP + 1500)) * ((int64_t)(TEMP + 1500));
            SENS2 += 8 * ((int64_t)(TEMP + 1500)) * ((int64_t)(TEMP + 1500));
        }

        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    int64_t P = ((((int64_t)D1 * SENS) >> 21) - OFF) >> 15;
    data->pressure_mbar = (float)P / 100.0f;
    data->temperature_c = TEMP / 100.0f;             // Convert from centiCelcius to Celsius

    // Check validity of conversions - values must be between min and max values on data sheet

    char buf[64];
    if (data->pressure_mbar < 10.0f || data->pressure_mbar > 1200.0f)
    {
        sprintf(buf, "%f", data->pressure_mbar);
        SEGGER_RTT_printf(0, "ms5607: pressure_mbar out of range: %s\n", buf);
        return HAL_ERROR;
    }

    if (data->temperature_c < -40.0f || data->temperature_c > 85.0f)
    {
        sprintf(buf, "%f", data->temperature_c);
        SEGGER_RTT_printf(0, "ms5607: temperature_c out of range: %s\n", buf);
        return HAL_ERROR;
    }

    // SEGGER_RTT_printf(0, "ms5607: process\n");
    // sprintf(buf, "%f", data->pressure_mbar);
    // SEGGER_RTT_printf(0, "ms5607: pressure_mbar: %s\n", buf);
    // sprintf(buf, "%f", data->temperature_c);
    // SEGGER_RTT_printf(0, "ms5607: temperature_c: %s\n", buf);

    return HAL_OK;
}
