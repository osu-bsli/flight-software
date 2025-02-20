/**
 * adxl375.c
 *
 * ADXL375 accelerometer driver.
 *
 * @authors
 * - Dawn Goorskey
 * - Hana Winchester
 * - Brian Jia
 */

#include "Sensors/adxl375.h"
#include <SEGGER_RTT.h>
#include "common.h"
#include "stm32h7xx_hal.h"
#include <cmsis_os.h>
#include <stdio.h>

/*
 * Header files are for sharing things that other C files need.
 * Register addresses should go HERE and not adxl375.h because other C files do not need to see them.
 */

/* I2C constants */
#define DEVICE_ID 0xE5u          /* fixed value (datasheet pg. 21) */
#define I2C_ADDRESS (0x53u << 1) /* (pg. 18) ALT_ADDRESS pin is low */

/* Register constants (pg. 20) */
#define REGISTER_DEVID 0x00u
#define REGISTER_THRESH_SHOCK 0x1Du
#define REGISTER_OFSX 0x1Eu
#define REGISTER_OFSY 0x1Fu
#define REGISTER_OFSZ 0x20u
#define REGISTER_DUR 0x21u
#define REGISTER_LATENT 0x22u
#define REGISTER_WINDOW 0x23u
#define REGISTER_THRESH_ACT 0x24u
#define REGISTER_THRESH_INACT 0x25u
#define REGISTER_TIME_INACT 0x26u
#define REGISTER_ACT_INACT_CTL 0x27u
#define REGISTER_SHOCK_AXES 0x2Au
#define REGISTER_ACT_SHOCK_STATUS 0x2Bu
#define REGISTER_BW_RATE 0x2Cu
#define REGISTER_BW_RATE_LOW_POWER (1 << 4)
#define REGISTER_BW_RATE_100HZ 0b1010
#define REGISTER_POWER_CTL 0x2Du
#define REGISTER_INT_ENABLE 0x2Eu
#define REGISTER_INT_MAP 0x2Fu
#define REGISTER_INT_SOURCE 0x30u
#define REGISTER_DATA_FORMAT 0x31u
#define REGISTER_DATA_FORMAT_SELF_TEST (1 << 7)
#define REGISTER_DATAX0 0x32u
#define REGISTER_DATAX1 0x33u
#define REGISTER_DATAY0 0x34u
#define REGISTER_DATAY1 0x35u
#define REGISTER_DATAZ0 0x36u
#define REGISTER_DATAZ1 0x37u
#define REGISTER_FIFO_CTL 0x38u
#define REGISTER_FIFO_STATUS 0x39u

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

static HAL_StatusTypeDef read_registers(struct fc_adxl375 *device, uint8_t reg,
                                        uint8_t *data, uint8_t length)
{
  // TODO: Use interrupt mode
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(
      device->i2c_handle, I2C_ADDRESS, reg, sizeof(reg), data, length);
  if (xSemaphoreTake(*device->i2c_semaphore, 100) != pdTRUE)
  {
    SEGGER_RTT_printf(0, "adxl375: read_registers timeout\n");
    return HAL_TIMEOUT;
  }
  return status;
}

static HAL_StatusTypeDef write_registers(struct fc_adxl375 *device, uint8_t reg, uint8_t *data, uint8_t length)
{
  // TODO: Use interrupt mode
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(
      device->i2c_handle, I2C_ADDRESS, reg, sizeof(reg), data, length);
  if (xSemaphoreTake(*device->i2c_semaphore, 100) != pdTRUE)
  {
    SEGGER_RTT_printf(0, "adxl375: write_registers timeout\n");
    return HAL_TIMEOUT;
  }
  return status;
}

static HAL_StatusTypeDef is_data_ready(struct fc_adxl375 *device, int *isready)
{
  uint8_t interrupt_data;

  /* read INT_SOURCE bits (pg. 23) */
  HAL_StatusTypeDef status = read_registers(
      device, REGISTER_INT_SOURCE, &interrupt_data, sizeof(interrupt_data));
  if (status != HAL_OK)
  {
    return status;
  }

  /* ============================= */
  /* DATA_READY is bit D7 (pg. 23) */
  /* ============================= */

  // Shift to ready D7 bit
  // TODO (Brian Jia): Mask off the 7th bit instead of shifting here
  interrupt_data >>= 7;

  // if DATA_READY bit is 1, an interrupt triggered indicating data is ready
  if (interrupt_data == 1)
  {
    *isready = 1;
  }
  else
  {
    *isready = 0;
  }

  return HAL_OK;
}

/*
 * Public functions.
 */

HAL_StatusTypeDef fc_adxl375_initialize(struct fc_adxl375 *device,
                                        I2C_HandleTypeDef *i2c_handle,
                                        SemaphoreHandle_t *i2c_semaphore)
{
  /* reset struct */
  device->i2c_handle = i2c_handle;
  device->i2c_semaphore = i2c_semaphore;

  HAL_StatusTypeDef status;
  uint8_t data;

  /* Check that device ID is correct */
  status = read_registers(device, REGISTER_DEVID, &data, sizeof(data));
  if (status != HAL_OK)
  {
    return status;
  }
  if (data != DEVICE_ID)
  {
    SEGGER_RTT_printf(0, "adxl375: device ID does not match expected\n");
    return HAL_ERROR;
  }

  /* Set measure bit in POWER_CTL register (pg. 22) */
  data = 0b00001000;
  status = write_registers(device, REGISTER_POWER_CTL, &data, sizeof(data));
  if (status != HAL_OK)
  {
    return status;
  }

  data = 0b00001011;
  status = write_registers(device, REGISTER_DATA_FORMAT, &data, sizeof(data));
  if (status != HAL_OK)
  {
    return status;
  }

  data = REGISTER_BW_RATE_100HZ; // disable low power, 100 Hz
  status = write_registers(device, REGISTER_BW_RATE, &data, sizeof(data));
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}

HAL_StatusTypeDef fc_adxl375_process(struct fc_adxl375 *device, struct fc_adxl375_data *data)
{
  HAL_StatusTypeDef status;

  /* ================================ */
  /* read raw acceleration data bytes */
  /* ================================ */

  uint8_t raw_accel_data[6]; /* DATAX0, X1, Y0, Y1, Z0, and Z1 registers (pg. 24) */

  /* start i2c read */
  status = read_registers(device, REGISTER_DATAX0, raw_accel_data, sizeof(raw_accel_data));
  if (status != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* ===================================== */
  /* convert bytes to signed 16-bit values */
  /* ===================================== */

  /* Little endian (pg. 24) */
  int16_t raw_acceleration_x = u8_to_i16(raw_accel_data[0], raw_accel_data[1]);
  int16_t raw_acceleration_y = u8_to_i16(raw_accel_data[2], raw_accel_data[3]);
  int16_t raw_acceleration_z = u8_to_i16(raw_accel_data[4], raw_accel_data[5]);

  /* ============================================ */
  /* convert raw data to actual acceleration data */
  /* ============================================ */

  float scale = 0.049; // (pg. 3) 49 mg/LSB
  data->acceleration_x = scale * (float)raw_acceleration_x;
  data->acceleration_y = scale * (float)raw_acceleration_y;
  data->acceleration_z = scale * (float)raw_acceleration_z;

  /* TODO: Is the ADXL375 on the 24-F01-001 FC damaged????? Readings seem VERY off */
  /* TODO: Maybe I just need to calibrate the accel lmao */
  /* TODO: Yeah it's a high-G accel it needs careful calibration */
  /* TODO: Calibrate the ADXL375 and add code to write the calibration values to the sensor on startup */
  char buf[64];
  // SEGGER_RTT_printf(0, "adxl375: process\n");
  // sprintf(buf, "%f", device->acceleration_x);
  // SEGGER_RTT_printf(0, "adxl375: accel x: %s\n", buf);
  // sprintf(buf, "%f", device->acceleration_y);
  // SEGGER_RTT_printf(0, "adxl375: accel y: %s\n", buf);
  // sprintf(buf, "%f", device->acceleration_z);
  // SEGGER_RTT_printf(0, "adxl375: accel z: %s\n", buf);

  return HAL_OK;
}
