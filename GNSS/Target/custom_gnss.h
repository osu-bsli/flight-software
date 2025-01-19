/**
  ******************************************************************************
  * @file    Target/custom_gnss.h
  * @author  SRA Application Team
  * @brief   Header file for custom_gnss.c
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
#ifndef CUSTOM_GNSS_H
#define CUSTOM_GNSS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "custom_conf.h"
#include "custom_board_conf.h"
#include "custom_bus.h"
#include "custom_errno.h"

#ifndef USE_CUSTOM_GNSS_TESEO_LIV3F
#define USE_CUSTOM_GNSS_TESEO_LIV3F    1U
#endif

#if (USE_CUSTOM_GNSS_TESEO_LIV3F == 1)
#include "teseo_liv3f.h"
#endif

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup CUSTOM_GNSS CUSTOM GNSS
 * @{
 */

/** @defgroup CUSTOM_GNSS_Exported_Types CUSTOM GNSS Exported Types
 * @{
 */

/**
 * @brief GNSS Message
 */
typedef struct
{
  uint8_t *buf;
  uint16_t len;
} CUSTOM_GNSS_Msg_t;

/**
 * @brief GNSS Capabilities
 */
typedef struct
{
  uint8_t   Geofence;
  uint8_t   Datalog;
  uint8_t   Odemeter;
  uint8_t   AssistedGNSS;
} CUSTOM_GNSS_Capabilities_t;

/**
 * @}
 */

/** @defgroup CUSTOM_GNSS_Exported_Constants CUSTOM GNSS Exported Constants
 * @{
 */
#if (USE_CUSTOM_GNSS_TESEO_LIV3F == 1)
#define CUSTOM_TESEO_LIV3F 0
#endif

#define CUSTOM_GNSS_INSTANCES_NBR      (USE_CUSTOM_GNSS_TESEO_LIV3F)

#if (CUSTOM_GNSS_INSTANCES_NBR == 0)
#error "No gnss instance has been selected"
#endif

/**
 * @}
 */

/** @addtogroup CUSTOM_GNSS_Exported_Functions CUSTOM GNSS Exported Functions
 * @{
 */

/**
 * @brief  Initializes GNSS
 * @param  Instance GNSS instance
 * @retval BSP status
 */
int32_t CUSTOM_GNSS_Init(uint32_t Instance);

/**
 * @brief  Deinitialize GNSS
 * @param  Instance GNSS instance
 * @retval BSP status
 */
int32_t CUSTOM_GNSS_DeInit(uint32_t Instance);

/**
 * @brief  Start (or resume after a given timeout) communication via I2C.
 * @param  Instance GNSS instance
 * @retval none
 */
void    CUSTOM_GNSS_BackgroundProcess(uint32_t Instance);

/**
 * @brief  Get the buffer containing a message from GNSS
 * @param  Instance GNSS instance
 * @retval The message buffer
 */
const   CUSTOM_GNSS_Msg_t* CUSTOM_GNSS_GetMessage(uint32_t Instance);

/**
 * @brief  Release the NMEA message buffer
 * @param  Instance GNSS instance
 * @param  Message The message buffer
 * @retval BSP status
 */
int32_t CUSTOM_GNSS_ReleaseMessage(uint32_t Instance, const CUSTOM_GNSS_Msg_t *Message);

/**
 * @brief  Send a command to the GNSS
 * @param  Instance GNSS instance
 * @param  Message The message buffer
 * @retval BSP status
 */
int32_t CUSTOM_GNSS_Send(uint32_t Instance, const CUSTOM_GNSS_Msg_t *Message);

/**
 * @brief  Get the wake-up status
 * @param  Instance GNSS instance
 * @param  Message The message buffer
 * @retval BSP status
 */
int32_t CUSTOM_GNSS_Wakeup_Status(uint32_t Instance, uint8_t *status);

/**
 * @brief  Reset the GNSS and reinit the FSM
 * @param  Instance GNSS instance
 * @param  Message The message buffer
 * @retval BSP status
 */
int32_t CUSTOM_GNSS_Reset(uint32_t Instance);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_GNSS_H */

