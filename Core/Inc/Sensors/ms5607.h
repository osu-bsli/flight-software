/*
 * fc_ms5607.h
 *
 *  Created on: Nov 5, 2023
 *      Author: bsli
 */

/*THIS IS TO BE EDITED- EVERYTHING BELOW IS JUST A TEMPLATE FOR NOW
 *
 *
 * Need to get PROM address
 * Pg. 8 has all the important constants*/

#ifndef INC_FC_MS5607_H_
#define INC_FC_MS5607_H_

#include "stm32h7xx_hal.h"

struct fc_ms5607
{
    I2C_HandleTypeDef *i2c_handle; /* the i2c peripheral */

    uint16_t C1; // C1 - Pressure Sensitivity
    uint16_t C2; // C2 - Pressure Offset
    uint16_t C3; // C3 - Temperature coefficient of pressure sensitivity
    uint16_t C4; // C4 - Temperature coefficient of pressure offset
    uint16_t C5; // C5 - Reference temperature
    uint16_t C6; // C6 - Temperature coefficient of the temperature
};

struct fc_ms5607_data
{
    float pressure_mbar; /* pressure in mbar */
    float temperature_c; /* temperature in degrees Celsius */
};

/* functions */
HAL_StatusTypeDef fc_ms5607_initialize(struct fc_ms5607 *device, I2C_HandleTypeDef *i2c_handle);
HAL_StatusTypeDef fc_ms5607_process(struct fc_ms5607 *device, struct fc_ms5607_data *data);

#endif /* INC_FC_MS5607_H_ */