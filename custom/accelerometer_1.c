/**
 * TODO: Properly attribute original author in accordance with GPL3.
 */
/**
* @Library for ADXL345 3-axis accelometer
* @Hardware dependencies: Could be changed very easily.
                                                STM32L152R uC
                                                SPI2
                                                Some GPIOs
* @Author Iman Hosseinzadeh iman[dot]hosseinzadeh AT gmail
 https://github.com/ImanHz

*/
/**
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <https://www.gnu.org/licenses/>.
**/

#include "accelerometer_1.h"
#include "stm32l4xx_hal.h"

#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define INT_SOURCE 0x30
#define DATA_FORMAT 0x31

#define handler hi2c

static void fs_accel1_write_register(uint8_t address, uint8_t value) {
  // Setting R/W = 0, i.e.: Write Mode
  address &= ~(0x80);

  HAL_GPIO_WritePin(ADXLCS_GPIO_Port, ADXLCS_Pin, GPIO_PIN_RESET);
  HAL_I2C_Master_Transmit(&handler, &address, 1, 10);
  HAL_I2C_Master_Transmit(&handler, &value, 1, 10);
  HAL_GPIO_WritePin(ADXLCS_GPIO_Port, ADXLCS_Pin, GPIO_PIN_SET);
}

static void fs_accel1_read_register(uint8_t address, uint8_t *value,
                                    uint8_t num) {
  // Multiple Byte Read Settings
  if (num > 1) {
    address |= 0x40;
  } else {
    address &= ~(0x40);
  }

  // Setting R/W = 1, i.e.: Read Mode
  address |= (0x80);

  HAL_GPIO_WritePin(ADXLCS_GPIO_Port, ADXLCS_Pin, GPIO_PIN_RESET);
  HAL_I2C_Master_Transmit(&handler, &address, 1, 10);
  HAL_I2C_Master_Receive(&handler, &value, num, 10);
  HAL_GPIO_WritePin(ADXLCS_GPIO_Port, ADXLCS_Pin, GPIO_PIN_SET);
}

void fs_accel1_init(ADXL_InitTypeDef *adxl) {
  HAL_GPIO_WritePin(ADXLCS_GPIO_Port, ADXLCS_Pin, GPIO_PIN_SET);
  HAL_Delay(5);
  fs_accel1_write_register(BW_RATE, 0b00001100);
  fs_accel1_write_register(DATA_FORMAT, 0b00001011);
  fs_accel1_write_register(POWER_CTL, 0b00001011);
}

void fs_accel1_get_acceleration(float *data, uint8_t outputType) {
  uint8_t data[6];
  fs_accel1_read_register(DATA0, data, 6);
  data[0] = ((int16_t)((data[1] * 256 + data[0])));
  data[1] = ((int16_t)((data[3] * 256 + data[2])));
  data[2] = ((int16_t)((data[5] * 256 + data[4])));
}

void fs_accel1_interrupt(void) {
  uint8_t reg = 0;
  while (1) {
    if (fs_accel1_interrupt_data_available()) {
    	// write to data.c
    }
  }
}

int fs_accel1_interrupt_data_available(void) {
  uint8_t reg = 0;
  fs_accel1_read_register(INT_SOURCE, &reg, 1);
  return reg & (1 << 8); // data ready
}
