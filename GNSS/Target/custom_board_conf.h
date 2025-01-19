/**
  ******************************************************************************
  * @file    custom_board_conf.h
  * @author  SRA Application Team
  * @brief   This file contains definitions for the GNSS components bus interfaces
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
#ifndef CUSTOM_BOARD_CONF_H
#define CUSTOM_BOARD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "custom_bus.h"
#include "stm32_bus_ex.h"
#include "custom_errno.h"

#define USE_I2C 1U

#define USE_CUSTOM_GNSS_TESEO_LIV3F	1U

#define CUSTOM_GNSS_I2C_Init        BSP_I2C3_Init
#define CUSTOM_GNSS_I2C_DeInit      BSP_I2C3_DeInit
#define CUSTOM_GNSS_I2C_Transmit_IT BSP_I2C3_Send_IT
#define CUSTOM_GNSS_I2C_Receive_IT  BSP_I2C3_Recv_IT
#define CUSTOM_GNSS_GetTick         BSP_GetTick

#define CUSTOM_RST_PORT                        GPIOH
#define CUSTOM_RST_PIN                         GPIO_PIN_10

#define CUSTOM_WAKEUP_PORT                     GPIOH
#define CUSTOM_WAKEUP_PIN                      GPIO_PIN_6

//#define CUSTOM_RegisterDefaultMspCallbacks     BSP_I2C3_RegisterDefaultMspCallbacks
#define CUSTOM_RegisterRxCb                    BSP_I2C3_RegisterRxCallback
#define CUSTOM_RegisterErrorCb                 BSP_I2C3_RegisterErrorCallback
#define CUSTOM_RegisterAbortCb                 BSP_I2C3_RegisterAbortCallback

/* To be checked */
#define CUSTOM_I2C_EV_IRQHanlder      BSP_I2C3_EV_IRQHanlder
#define CUSTOM_I2C_ER_IRQHanlder      BSP_I2C3_ER_IRQHanlder

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_BOARD_CONF_H */

