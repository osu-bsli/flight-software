/*
 * ms5607.h
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
    uint8_t state;
    float last_pressure_mbar;
    float last_temperature_c;
    uint32_t conversion_started_ms;
    uint32_t D1, D2;

    // PROM contents
    // C[0] - 16-bit manufacturer reserved
    // C[1] - C1: Pressure Sensitivity
    // C[2] - C2: Pressure Offset
    // C[3] - C3: Temperature coefficient of pressure sensitivity
    // C[4] - C4: Temperature coefficient of pressure offset
    // C[5] - C5: Reference temperature
    // C[6] - C6: Temperature coefficient of the temperature
    uint16_t C[7];

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