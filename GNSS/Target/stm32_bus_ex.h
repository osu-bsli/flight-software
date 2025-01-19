/**
  ******************************************************************************
  * @file    stm32_bus_ex.h
  * @author  MCD Application Team
  * @brief   Header file for stm32_bus_ex.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32_BUS_EX_H
#define STM32_BUS_EX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "custom_conf.h"
#include "custom_errno.h"

/* Defines -------------------------------------------------------------------*/

#define BUS_I2C3_EV_IRQn        I2C3_EV_IRQn
#define BUS_I2C3_ER_IRQn        I2C3_ER_IRQn

int32_t BSP_I2C3_Send_IT(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C3_Recv_IT(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
int32_t BSP_I2C3_RegisterRxCallback(pI2C_CallbackTypeDef pCallback);
int32_t BSP_I2C3_RegisterErrorCallback(pI2C_CallbackTypeDef pCallback);
int32_t BSP_I2C3_RegisterAbortCallback(pI2C_CallbackTypeDef pCallback);
#endif

void BSP_EV_I2C3_IRQHanlder(void);
void BSP_ER_I2C3_IRQHanlder(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32_BUS_EX_H */

