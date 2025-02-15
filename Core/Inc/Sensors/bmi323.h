/*
 * bmi323.h
 *
 *  Created on: Nov 12, 2023
 *      Author: user
 */

#pragma once

#include <stdbool.h>
#include "stm32h7xx_hal.h"

/* struct */

struct fc_bmi323_data {
    /* datasheet pg. 22 */

    /* Acceleration - in Gs */
    float accel_x;
    float accel_y;
    float accel_z;
    bool accel_x_saturated;
    bool accel_y_saturated;
    bool accel_z_saturated;

    /* Gyroscope - in degrees/second */
    float gyro_x;
    float gyro_y;
    float gyro_z;
    bool gyro_x_saturated;
    bool gyro_y_saturated;
    bool gyro_z_saturated;

    /* Temperature - in Celcius */
    float temp;

    float time; /* sensor keeps timestamps for measurements (datasheet pg. 25) */
    uint32_t kernel_timestamp; /* in kernel ticks */
};

struct fc_bmi323 {
    I2C_HandleTypeDef *hi2c; /* the i2c peripheral */
};

/* functions */
HAL_StatusTypeDef fc_bmi323_initialize(struct fc_bmi323 *bmi323, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef fc_bmi323_process(struct fc_bmi323 *bmi323, struct fc_bmi323_data *data);