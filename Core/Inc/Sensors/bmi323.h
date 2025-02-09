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
    float acc_x;
    float acc_y;
    float acc_z;
    bool acc_x_saturated;
    bool acc_y_saturated;
    bool acc_z_saturated;

    /* Gyroscope - in degrees/second */
    float gyr_x;
    float gyr_y;
    float gyr_z;
    bool gyr_x_saturated;
    bool gyr_y_saturated;
    bool gyr_z_saturated;

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