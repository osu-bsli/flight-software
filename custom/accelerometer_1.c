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
#include "packet-parser/packet.h"
#include "telemetry.h"
#include "i2c.h"

#define ACCEL1_DEVID 0xE5
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define INT_ENABLE 0x2E
#define INT_SOURCE 0x30
#define DATA_FORMAT 0x31
#define ACCEL_DATA_BEGIN 0x32

#define ADXLCS_Pin GPIO_PIN_4
#define ADXLCS_GPIO_Port GPIOC

#define handler hi2c2

static void fs_accel1_write_register(uint8_t address, uint8_t value) {
  // Setting R/W = 0, i.e.: Write Mode
  address &= ~(0x80);

  HAL_GPIO_WritePin(ADXLCS_GPIO_Port, ADXLCS_Pin, GPIO_PIN_RESET);
  HAL_I2C_Master_Transmit(&handler, (ACCEL1_DEVID << 1), &address, 1, 10);
  HAL_I2C_Master_Transmit(&handler, (ACCEL1_DEVID << 1), &value, 1, 10);
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
  HAL_I2C_Master_Transmit(&handler, (ACCEL1_DEVID << 1), &address, 1, 10);
  HAL_I2C_Master_Receive(&handler, (ACCEL1_DEVID << 1), value, num, 10);
  HAL_GPIO_WritePin(ADXLCS_GPIO_Port, ADXLCS_Pin, GPIO_PIN_SET);
}

void fs_accel1_init(void) {
  HAL_GPIO_WritePin(ADXLCS_GPIO_Port, ADXLCS_Pin, GPIO_PIN_SET);
  HAL_Delay(5);
  fs_accel1_write_register(BW_RATE, 0b00001100);
  fs_accel1_write_register(DATA_FORMAT, 0b00001011);
  fs_accel1_write_register(POWER_CTL, 0b00001011);
  fs_accel1_write_register(INT_ENABLE, 0b1000000);
}

void fs_accel1_get_acceleration(int16_t* out_data) {
//  uint8_t data[6];
  unsigned char *data = (unsigned char*) out_data;
  fs_accel1_read_register(ACCEL_DATA_BEGIN, data, 6);
//  out_data[0] = ((int16_t)(((data[1] << 8) | data[0])));
//  out_data[1] = ((int16_t)(((data[3] << 8) | data[2])));
//  out_data[2] = ((int16_t)(((data[5] << 8) | data[4])));
}

int fs_accel1_interrupt_data_available(void) {
  uint8_t reg = 0;
  fs_accel1_read_register(INT_SOURCE, &reg, 1);
  return reg >> 7; // data ready
}

void fs_accel1_interrupt(void) {
  if (fs_accel1_interrupt_data_available()) {
	  int16_t* acceleration;
	  uint8_t* output;
	  float timestamp = ((float) HAL_GetTick()) / 1000.0f;
	  fs_accel1_get_acceleration(acceleration);
	  int packet_size = write_acceleration_packet(
		  output,
		  timestamp,
		  acceleration[0],
		  acceleration[1],
		  acceleration[2]
	  );
	  fc_telemetry_send_packet(output, packet_size);
  }
}
