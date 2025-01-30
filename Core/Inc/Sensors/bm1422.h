/*
 * bm1422.h
 *
 *  Created on: Jan 29, 2025
 *      Author: bsli
 */

#ifndef INC_FC_BM1422_H_
#define INC_FC_BM1422_H_

#include "stm32h7xx_hal.h"

/* I2C constants (Pg. 10) */
#define DEVICE_ID			               (0x0Eu << 1)	// There is a low and high address

// Register constants (pg. 10)
#define REGISTER_INFO      	          0x0D   // LSB
#define REGISTER_WIA		              0x0F
#define REGISTER_DATAX          		  0x10	 // LSB
#define REGISTER_DATAY		            0x12	 // LSB
#define REGISTER_DATAZ		            0x14	 // LSB
#define REGISTER_STA1	                0x18
#define REGISTER_CNTL1		            0x1B
#define REGISTER_CNTL2		            0x1C
#define REGISTER_CNTL3 	              0x1D
#define REGISTER_AVE_A                0x40
#define REGISTER_CNTL4                0x5C
#define REGISTER_TEMP	                0x60	 // LSB
#define REGISTER_OFF_X                0x6C
#define REGISTER_OFF_Y	              0x72
#define REGISTER_OFF_Z	              0x78
#define REGISTER_FINEOUTPUTX          0x90   // LSB
#define REGISTER_FINEOUTPUTY          0x92   // LSB
#define REGISTER_FINEOUTPUTZ          0x94   // LSB
#define REGISTER_GAIN_PARA_X	        0x9C   // LSB
#define REGISTER_GAIN_PARA_Y	        0x9E   // LSB


/* struct */

struct fc_bm1422 {
	I2C_HandleTypeDef *i2c_handle;	 /* the i2c peripheral */

	float magnetic_strength_x;
	float magnetic_strength_y;
	float magnetic_strength_z;
};

/* Functions */
int fc_bm1422_initialize(struct fc_bm1422 *device, I2C_HandleTypeDef *i2c_handle);
int fc_bm1422_process(struct fc_bm1422 *device);

#endif /* INC_FC_BM1422_H_ */
