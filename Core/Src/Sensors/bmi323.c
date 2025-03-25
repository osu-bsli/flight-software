/**
 * bmi323.c
 *
 * ADXL375 accelerometer driver.
 *
 * @authors
 * - Dawn Goorskey
 * - Hana Winchester
 * - Brian Jia
 */
#include "Sensors/bmi323.h"
#include <assert.h>
#include "SEGGER_RTT.h"

/* sensor configuration
 * - Accelerometer range: +/- 4 g       (max).
 * - Gyroscope range:     +/- 180 deg/s (min, we only care about off-axis rotation which is slow).
 * - Temperature range is fixed.
 * - Power mode: normal (datasheet pg. 22).
 * Ranges are +/-, see datasheet pg. 1.
 */

/* TODO:
 * - Check saturation flags for each axis of each sensor during sensor read.
 */

/* i2c constants */
#define I2C_ADDRESS (0x68u << 1) /* depends on how "SDO" pin is wired (datasheet pg. 217) */

/* register constants (datasheet pg. 62) */
#define REGISTER_CHIP_ID 0x00u
#define REGISTER_ERR_REG 0x01u
#define REGISTER_STATUS 0x02u
#define REGISTER_ACC_DATA_X 0x03u
#define REGISTER_ACC_DATA_Y 0x04u
#define REGISTER_ACC_DATA_Z 0x05u
#define REGISTER_GYR_DATA_X 0x06u
#define REGISTER_GYR_DATA_Y 0x07u
#define REGISTER_GYR_DATA_Z 0x08u
#define REGISTER_TEMP_DATA 0x09u
#define REGISTER_SENSOR_TIME_0 0x0Au
#define REGISTER_SENSOR_TIME_1 0x0Bu
#define REGISTER_SAT_FLAGS 0x0Cu

#define REGISTER_INT_STATUS_INT1 0x0Du
#define REGISTER_INT_STATUS_INT1_TEMP_DRDY (1 << 11)
#define REGISTER_INT_STATUS_INT1_GYR_DRDY (1 << 12)
#define REGISTER_INT_STATUS_INT1_ACC_DRDY (1 << 13)

#define REGISTER_INT_STATUS_INT2 0x0Eu
#define REGISTER_INT_STATUS_IBI 0x0Fu
#define REGISTER_FEATURE_IO0 0x10u
#define REGISTER_FEATURE_IO1 0x11u
#define REGISTER_FEATURE_IO2 0x12u
#define REGISTER_FEATURE_IO3 0x13u
#define REGISTER_FEATURE_IO_STATUS 0x14u
#define REGISTER_FIFO_FILL_LEVEL 0x15u
#define REGISTER_FIFO_DATA 0x16u
#define REGISTER_ACC_CONF 0x20u
#define REGISTER_GYR_CONF 0x21u
#define REGISTER_ALT_ACC_CONF 0x28u
#define REGISTER_ALT_GYR_CONF 0x29u
#define REGISTER_ALT_CONF 0x2Au
/* (datasheet pg. 63) */
#define REGISTER_ALT_STATUS 0x2Bu
#define REGISTER_FIFO_WATERMARK 0x35u
#define REGISTER_FIFO_CONF 0x36u
#define REGISTER_FIFO_CTRL 0x37u
#define REGISTER_IO_INT_CTRL 0x38u
#define REGISTER_INT_CONF 0x39u
#define REGISTER_INT_MAP1 0x3Au
#define REGISTER_INT_MAP2 0x3Bu
#define REGISTER_FEATURE_CTRL 0x40u
#define REGISTER_FEATURE_DATA_ADDR 0x41u
#define REGISTER_FEATURE_DATA_TX 0x42u
#define REGISTER_FEATURE_DATA_STATUS 0x43u
#define REGISTER_FEATURE_ENGINE_STATUS 0x45u
#define REGISTER_FEATURE_EVENT_EXT 0x47u
#define REGISTER_IO_PDN_CTRL 0x4Fu
#define REGISTER_IO_SPI_IF 0x50u
#define REGISTER_IO_PAD_STRENGTH 0x51u
#define REGISTER_IO_I2C_IF 0x52u
#define REGISTER_IO_ODR_DEVIATION 0x53u
#define REGISTER_ACC_DP_OFF_X 0x60u
#define REGISTER_ACC_DP_DGAIN_X 0x61u
#define REGISTER_ACC_DP_OFF_Y 0x62u
#define REGISTER_ACC_DP_DGAIN_Y 0x63u
#define REGISTER_ACC_DP_OFF_Z 0x64u
#define REGISTER_ACC_DP_DGAIN_Z 0x65u
#define REGISTER_GYR_DP_OFF_X 0x66u
#define REGISTER_GYR_DP_DGAIN_X 0x67u
#define REGISTER_GYR_DP_OFF_Y 0x68u
#define REGISTER_GYR_DP_DGAIN_Y 0x69u
#define REGISTER_GYR_DP_OFF_Z 0x6Au
/* (datasheet pg. 64) */
#define REGISTER_GYR_DP_DGAIN_Z 0x6Bu
#define REGISTER_I3C_TC_SYNC_TPH 0x70u
#define REGISTER_I3C_TC_SYNC_TU 0x71u
#define REGISTER_I3C_TC_SYNC_ODR 0x72u
#define REGISTER_CMD 0x7Eu
#define REGISTER_CFG_RES 0x7Fu

#define ACC_INVALID 0x8000u  /* invalid value for acceleration data (datasheet pg. 22) */
#define GYR_INVALID 0x8000u  /* invalid value for gyroscope data (datasheet pg. 24) */
#define TEMP_INVALID 0x8000u /* invalid value for temperature data (datasheet pg. 25) */

#define ACC_RANGE_MAX 4.0f /* acceleration          (g)     (datasheet pg. 23) */
#define ACC_RANGE_MIN (-4.0f)
#define GYR_RANGE_MAX 250.0f /* angular acceleration (deg/s) (datasheet pg. 25) */
#define GYR_RANGE_MIN (-250.0f)

/* Starts reading multiple bytes starting from a register (useful for batch reading from multiple
 * contiguous registers at once).
 * Use this to read multiple contiguous registers in one go (aka "burst mode").
 * 2 dummy bytes will be received before any actual data, so the buffer should be 2 bytes larger
 * than the total amount of data expected and the first 2 bytes written to the buffer should be
 * ignored (datasheet pg. 218).
 * This function will block the current FreeRTOS task until the read finishes. */
static HAL_StatusTypeDef read_registers(struct fc_bmi323 *device, uint8_t reg, void *data, uint8_t length)
{
    assert(length >= 4); /* minimum possible length = 2 dummy bytes + 1 register's worth of data = 4 bytes*/

    HAL_StatusTypeDef status;

    /* first, send the register (datasheet pg. 223) */
    status = HAL_I2C_Master_Transmit_IT(device->hi2c, I2C_ADDRESS, &reg, sizeof(reg));
    if (status != HAL_OK)
        return status;

    if (xSemaphoreTake(*device->i2c_semaphore, 1000) != pdTRUE)
    {
        SEGGER_RTT_printf(0, "bmi323: read_registers timeout\n");
        return HAL_TIMEOUT;
    }

    status = HAL_I2C_Master_Receive_IT(device->hi2c, I2C_ADDRESS, (uint8_t *)data, length);
    if (status != HAL_OK)
        return status;

    if (xSemaphoreTake(*device->i2c_semaphore, 1000) != pdTRUE)
    {
        SEGGER_RTT_printf(0, "bmi323: read_registers timeout\n");
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

/* Starts writing multiple bytes starting from a register (useful for batch writing to multiple
 * contiguous registers at once).
 * This function will block the current FreeRTOS task until the write finishes. */
static HAL_StatusTypeDef write_registers(struct fc_bmi323 *device, uint8_t reg, void *data, uint8_t length)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write_IT(device->hi2c, I2C_ADDRESS, reg, sizeof(reg),
                                                    (uint8_t *)data, length);
    if (status != HAL_OK)
        return status;

    if (xSemaphoreTake(*device->i2c_semaphore, 100) != pdTRUE)
    {
        SEGGER_RTT_printf(0, "bmi323: write_registers timeout\n");
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

HAL_StatusTypeDef fc_bmi323_initialize(struct fc_bmi323 *bmi323, I2C_HandleTypeDef *hi2c, SemaphoreHandle_t *i2c_semaphore)
{
    bmi323->hi2c = hi2c;
    bmi323->i2c_semaphore = i2c_semaphore;

    /* when writing to registers with reserved bits, must read, update, then write (datasheet pg. 62) */
    /* write calibration values to registers (datasheet pg. 55) */

    /*
     * THE FIRST 2 BYTES OF ANY I2C REGISTER READ FROM THE BMI323 ARE 0 DUMMY BYTES.
     */

    HAL_StatusTypeDef status;

    /* check chip id (datasheet pg. 66) */
    uint16_t chip_id_value[2] = {0x1234, 0x5678};
    status = read_registers(bmi323, REGISTER_CHIP_ID, chip_id_value, sizeof(chip_id_value));
    if (status != HAL_OK)
        return status;
    if ((chip_id_value[1] & 0xFF) != 0x43u)
    {
        SEGGER_RTT_printf(0, "bmi323: device ID does not match expected\n");
        return HAL_ERROR;
    }

    /* check ERR_REG before enabling sensors (datasheet pg. 67) */
    uint16_t err_value[2] = {0x1234, 0x5678};
    status = read_registers(bmi323, REGISTER_ERR_REG, err_value, sizeof(err_value));
    if (status != HAL_OK)
        return status;
    if (err_value[1])
        return HAL_ERROR;

    /* check STATUS */
    uint16_t status_value[2] = {0x1234, 0x5678};
    status = read_registers(bmi323, REGISTER_STATUS, status_value, sizeof(status_value));
    if (status != HAL_OK)
        return status;
    // The BMI323 doesn't reset when the STM32 does, so the powerup flag may not always be set
    //    if ((status_value[1] & 1) == 0) BKPT_ERROR;

    uint8_t int_conf_data[] = {1, 0};

    status = write_registers(bmi323, REGISTER_INT_CONF, &int_conf_data[2], 2);
    if (status != HAL_OK)
        return status;

    /* =================================================================================== */
    /* configure ACC_CONF register (acc_mode, acc_range, acc_bw, acc_avg_num, and acc_odr) */
    /* =================================================================================== */

    uint8_t acc_conf_bytes[4]; /* 2 dummy bytes required by read_registers() */
    status = read_registers(bmi323, REGISTER_ACC_CONF, acc_conf_bytes, sizeof(acc_conf_bytes));
    if (status != HAL_OK)
        return status;

    /* ACC_CONF.acc_mode =    0b111  for normal power mode (datasheet pg. 22)
     * ACC_CONF.acc_avg_num = 0b000  for no averaging
     * ACC_CONF.acc_bw =      0b1    for most accurate filtering (?)
     * ACC_CONF.acc_range =   0b001  for +/- 4 g range (datasheet pg. 92)
     * ACC_CONF.acc_odr =     0b1011 for 800 Hz sample rate (we may change this later)
     * format: [x:1][acc_mode:3][x:1][acc_avg_num:3]_[acc_bw:1][acc_range:3][acc_odr:4]
     * binary: [x]  [111]       [x]  [000]           [1]       [001]        [1011]
     * hex:    0x70                                  0x9b
     */
    acc_conf_bytes[3] &= 0x88u; /* erase non-reserved bits */
    acc_conf_bytes[2] &= 0x00u;
    acc_conf_bytes[3] |= 0x70u; /* write to non-reserved bits */
    acc_conf_bytes[2] |= 0x9bu;
    status = write_registers(bmi323, REGISTER_ACC_CONF, &acc_conf_bytes[2], 2); /* no dummy bytes when writing */
    if (status != HAL_OK)
        return status;

    /* =================================================================================== */
    /* configure GYR_CONF register (gyr_mode, gyr_range, gyr_bw, gyr_avg_num, and gyr_odr) */
    /* =================================================================================== */

    uint8_t gyr_conf_bytes[4]; /* 2 dummy bytes required by read_registers() */
    status = read_registers(bmi323, REGISTER_GYR_CONF, gyr_conf_bytes, sizeof(acc_conf_bytes));
    if (status != HAL_OK)
        return status;

    /* GYR_CONF.gyr_mode =    0b111  for normal power mode (datasheet pg. 22)
     * GYR_CONF.gyr_avg_num = 0b000  for no averaging
     * GYR_CONF.gyr_bw =      0b1    for most accurate filtering (?)
     * GYR_CONF.gyr_range =   0b001  for +/- 250 deg/s range (datasheet pg. 94)
     * GYR_CONF.gyr_odr =     0b1011 for 800 Hz sample rate (we may change this later)
     * format: [x:1][gyr_mode:3][x:1][gyr_avg_num:3]_[gyr_bw:1][gyr_range:3][gyr_odr:4]
     * binary: [x]  [111]       [x]  [000]           [1]       [001]        [1011]
     * hex:    0x70                                  0x9b
     */
    gyr_conf_bytes[3] &= 0x88u; /* TODO: finish this shit */
    gyr_conf_bytes[2] &= 0x00u;
    gyr_conf_bytes[3] |= 0x70u;
    gyr_conf_bytes[2] |= 0x9bu;
    status = write_registers(bmi323, REGISTER_GYR_CONF, &gyr_conf_bytes[2],
                             sizeof(gyr_conf_bytes) - 2); /* no dummy bytes when writing */
    if (status != HAL_OK)
        return status;

    // TODO: Double check these
    uint8_t int_map2_bytes[2] = {40u, 05u};
    status = write_registers(bmi323, REGISTER_INT_MAP2, int_map2_bytes, sizeof(int_map2_bytes));

    if (status != HAL_OK)
        return status;

    return HAL_OK;
}

HAL_StatusTypeDef fc_bmi323_process(struct fc_bmi323 *bmi323, struct fc_bmi323_data *data)
{

    // TODO: Check if we even need to bother waiting for the data ready flag.
    //       The sensor might be ready before we read it for the first time.
    //       Plus, we can configure the sensor to give new data faster than we could
    //       ever possibly read it. From a signal processing standpoint, we can also
    //       enable the sensor's internal filtering to reduce aliasing.
    //
    // TODO: Also think about using actual GPIO interrupts to handle data ready.

    HAL_StatusTypeDef status;

    /*
     * Gyroscope
     */

    {
        int16_t gyro_data[4]; // dummy + 3-axis gyro data

        // multiplier to convert an int16_t to the sensor range
        // (65535 is the max of an int16_t)
        float scale = (GYR_RANGE_MAX - GYR_RANGE_MIN) / 65535.0f;

        status = read_registers(bmi323, REGISTER_GYR_DATA_X, gyro_data, sizeof(gyro_data));
        if (status != HAL_OK)
            return status;

        data->gyro_x = scale * (float)gyro_data[1];
        data->gyro_y = scale * (float)gyro_data[2];
        data->gyro_z = scale * (float)gyro_data[3];
    }

    /*
     * Temperature
     */

    {
        int16_t temp_data[2];

        status = read_registers(bmi323, REGISTER_TEMP_DATA, temp_data, sizeof(temp_data));
        if (status != HAL_OK)
            return status;

        data->temp = (float)temp_data[1] / 512.0f + 23.0f;
    }

    /*
     * Acceleration
     */

    {
        int16_t accel_data[4]; // dummy + 3-axis acceleration data

        status = read_registers(bmi323, REGISTER_ACC_DATA_X, accel_data, sizeof(accel_data));
        if (status != HAL_OK)
            return status;

        float scale = (ACC_RANGE_MAX - ACC_RANGE_MIN) / 65535.0f;

        data->accel_x = scale * (float)accel_data[1];
        data->accel_y = scale * (float)accel_data[2];
        data->accel_z = scale * (float)accel_data[3];
    }

    data->kernel_timestamp = xTaskGetTickCount();

    return HAL_OK;
}