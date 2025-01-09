/**
 * adxl375.h
 *
 * ADXL375 accelerometer driver. 
 */

/* "#pragma once" prevents this header file from being included multiple times in a C file */
#pragma once

#include "stm32h7xx_hal.h"

/*
 * Header files are for sharing things that other C files need.
 * Register addresses should go in adxl375.c and not this header file because other C files do not need to see them.
 * 
 * Header files should contain the absolute bare minimum that other files need to see.
 */

struct fc_adxl375 {
  I2C_HandleTypeDef *i2c_handle;

  float acceleration_x;
  float acceleration_y;
  float acceleration_z;
};

/*
 * The public-facing API should be as simple as possible. Consider what the flight software needs from the sensor and design the bare minimum API to provide that.
 */
HAL_StatusTypeDef fc_adxl375_initialize(struct fc_adxl375 *device,
                                        I2C_HandleTypeDef *i2c_handle);
HAL_StatusTypeDef fc_adxl375_process(struct fc_adxl375 *device);
