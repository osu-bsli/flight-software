/**
  ******************************************************************************
  * @file    Target/custom_gnss.c
  * @author  SRA Application Team
  * @brief   This file provides a set of functions needed to manage the GNSS
  *          module
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

/* Includes ------------------------------------------------------------------*/
#include "custom_gnss.h"

/** @addtogroup BSP BSP
 * @{
 */

/** @addtogroup CUSTOM CUSTOM
 * @{
 */

/** @defgroup CUSTOM_GNSS CUSTOM GNSS
 * @{
 */

/** @defgroup CUSTOM_GNSS_Exported_Variables CUSTOM GNSS Exported Variables
 * @{
 */

/**
 * @}
 */

/** @defgroup CUSTOM_GNSS_Private_Variables CUSTOM GNSS Private Variables
 * @{
 */

static TESEO_LIV3F_Object_t teseo_liv3f_obj;

/**
 * @}
 */

/** @defgroup CUSTOM_GNSS_Private_Function_Prototypes CUSTOM GNSS Private Function Prototypes
 * @{
 */

#if (USE_CUSTOM_GNSS_TESEO_LIV3F == 1)
static int32_t TESEO_LIV3F_Probe(void);

static void CUSTOM_GNSS_Rst(void);
static void CUSTOM_GNSS_RegisterCallbacks(void);
#endif

/**
 * @}
 */

/** @defgroup CUSTOM_GNSS_Exported_Functions CUSTOM_GNSS Exported Functions
 * @{
 */

 __weak int32_t CUSTOM_GNSS_GetTick(void)
{
  return BSP_ERROR_NONE;
}

#if (USE_CUSTOM_GNSS_TESEO_LIV3F == 1)
#if (USE_I2C == 1)
/* I2C CBs */
void CUSTOM_GNSS_I2C_RxCb(I2C_HandleTypeDef *hi2c);
void CUSTOM_GNSS_I2C_ErrorCb(I2C_HandleTypeDef *hi2c);
void CUSTOM_GNSS_I2C_AbortCb(I2C_HandleTypeDef *hi2c);

__weak int32_t CUSTOM_GNSS_I2C_Transmit_IT(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  UNUSED(DevAddr);
  UNUSED(pData);
  UNUSED(Length);
  return BSP_ERROR_NONE;
}
__weak int32_t CUSTOM_GNSS_I2C_Receive_IT(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  UNUSED(DevAddr);
  UNUSED(pData);
  UNUSED(Length);
  return BSP_ERROR_NONE;
}

#else
/* UART CBs */
void CUSTOM_GNSS_UART_RxCb(UART_HandleTypeDef *huart);
void CUSTOM_GNSS_UART_ErrorCb(UART_HandleTypeDef *huart);

__weak int32_t CUSTOM_GNSS_UART_Transmit_IT(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  UNUSED(DevAddr);
  UNUSED(pData);
  UNUSED(Length);
  return BSP_ERROR_NONE;
}
__weak int32_t CUSTOM_GNSS_UART_Receive_IT(uint16_t DevAddr, uint8_t *pData, uint16_t Length)
{
  UNUSED(DevAddr);
  UNUSED(pData);
  UNUSED(Length);
  return BSP_ERROR_NONE;
}
__weak void CUSTOM_GNSS_UART_ClearOREF(void)
{
}

#endif /* USE_I2C */
#endif

int32_t CUSTOM_GNSS_Init(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  switch (Instance)
  {
#if (USE_CUSTOM_GNSS_TESEO_LIV3F == 1)
    case CUSTOM_TESEO_LIV3F:
      if (TESEO_LIV3F_Probe() != BSP_ERROR_NONE)
      {
        ret = BSP_ERROR_NO_INIT;
      }
      break;
#endif

    default:
      ret = BSP_ERROR_WRONG_PARAM;
      break;
  }

  return ret;
}

int32_t CUSTOM_GNSS_DeInit(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= CUSTOM_GNSS_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (TESEO_LIV3F_DeInit(&teseo_liv3f_obj) != BSP_ERROR_NONE)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

const CUSTOM_GNSS_Msg_t* CUSTOM_GNSS_GetMessage(uint32_t Instance)
{
  const CUSTOM_GNSS_Msg_t* msg;

  if (Instance >= CUSTOM_GNSS_INSTANCES_NBR)
  {
    msg = NULL;
  }
  else
  {
    msg = (CUSTOM_GNSS_Msg_t *)TESEO_LIV3F_GetMessage(&teseo_liv3f_obj);
  }

  return msg;
}

int32_t CUSTOM_GNSS_ReleaseMessage(uint32_t Instance, const CUSTOM_GNSS_Msg_t *Message)
{
  int32_t ret;

  if (Instance >= CUSTOM_GNSS_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (TESEO_LIV3F_ReleaseMessage(&teseo_liv3f_obj, (TESEO_LIV3F_Msg_t*)Message) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  return ret;
}

int32_t CUSTOM_GNSS_Send(uint32_t Instance, const CUSTOM_GNSS_Msg_t *Message)
{
  int32_t ret;

  if (Instance >= CUSTOM_GNSS_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (TESEO_LIV3F_Send(&teseo_liv3f_obj, (TESEO_LIV3F_Msg_t*)Message) != BSP_ERROR_NONE)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  return ret;
}

int32_t CUSTOM_GNSS_Wakeup_Status(uint32_t Instance, uint8_t *status)
{
  int32_t ret = BSP_ERROR_NONE;

  if (Instance >= CUSTOM_GNSS_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    (HAL_GPIO_ReadPin(CUSTOM_WAKEUP_PORT, CUSTOM_WAKEUP_PIN) == GPIO_PIN_RESET) ? (*status = 0) : (*status = 1);
  }

  return ret;
}

int32_t CUSTOM_GNSS_Reset(uint32_t Instance)
{
  int32_t ret = BSP_ERROR_NONE;

  if (Instance >= CUSTOM_GNSS_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    if (TESEO_LIV3F_DeInit(&teseo_liv3f_obj) != TESEO_LIV3F_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      if (TESEO_LIV3F_Init(&teseo_liv3f_obj) != TESEO_LIV3F_OK)
      {
        ret = BSP_ERROR_COMPONENT_FAILURE;
      }
      else
      {
        CUSTOM_GNSS_RegisterCallbacks();
      }
    }
  }

  return ret;
}

void CUSTOM_GNSS_BackgroundProcess(uint32_t Instance)
{
  (void)Instance;

  TESEO_LIV3F_I2C_BackgroundProcess();
}

/**
 * @}
 */

/** @defgroup CUSTOM_MOTION_SENSOR_Private_Functions IKS01A2 MOTION SENSOR Private Functions
 * @{
 */

#if (USE_CUSTOM_GNSS_TESEO_LIV3F  == 1)
/**
 * @brief  Register Bus IOs for instance 0
 * @retval BSP status
 */
static int32_t TESEO_LIV3F_Probe(void)
{
  TESEO_LIV3F_IO_t            io_ctx;

  int32_t ret;

  /* Register user BUS (I2C/UART) callbacks */
#if (USE_I2C == 1)

  io_ctx.BusType     = TESEO_LIV3F_I2C_BUS;
  io_ctx.Address     = TESEO_LIV3F_I2C_7BITS_ADDR;
  io_ctx.Init        = CUSTOM_GNSS_I2C_Init;
  io_ctx.DeInit      = CUSTOM_GNSS_I2C_DeInit;
  io_ctx.Transmit_IT = CUSTOM_GNSS_I2C_Transmit_IT;
  io_ctx.Receive_IT  = CUSTOM_GNSS_I2C_Receive_IT;

#else

  io_ctx.BusType     = TESEO_LIV3F_UART_BUS;
  io_ctx.Init        = CUSTOM_GNSS_UART_Init;
  io_ctx.DeInit      = CUSTOM_GNSS_UART_DeInit;
  io_ctx.Transmit_IT = CUSTOM_GNSS_UART_Transmit_IT;
  io_ctx.Receive_IT  = CUSTOM_GNSS_UART_Receive_IT;
  io_ctx.ClearOREF   = CUSTOM_GNSS_UART_ClearOREF;

#endif

  io_ctx.Reset         = CUSTOM_GNSS_Rst;
  io_ctx.GetTick       = CUSTOM_GNSS_GetTick;

//  CUSTOM_RegisterDefaultMspCallbacks();

  if (TESEO_LIV3F_RegisterBusIO(&teseo_liv3f_obj, &io_ctx) != TESEO_LIV3F_OK)
  {
    ret = BSP_ERROR_UNKNOWN_COMPONENT;
  }
  else
  {
    if (TESEO_LIV3F_Init(&teseo_liv3f_obj) != TESEO_LIV3F_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      CUSTOM_GNSS_RegisterCallbacks();

      ret = BSP_ERROR_NONE;
    }
  }
  return ret;
}

static void CUSTOM_GNSS_Rst(void)
{
  HAL_GPIO_WritePin(CUSTOM_RST_PORT, CUSTOM_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(CUSTOM_RST_PORT, CUSTOM_RST_PIN, GPIO_PIN_SET);
  HAL_Delay(150);
}

static void CUSTOM_GNSS_RegisterCallbacks(void)
{
#if (USE_I2C == 1)

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
  // FIXME: check the return value instead
  (void)CUSTOM_RegisterRxCb(CUSTOM_GNSS_I2C_RxCb);
  (void)CUSTOM_RegisterErrorCb(CUSTOM_GNSS_I2C_ErrorCb);
  (void)CUSTOM_RegisterAbortCb(CUSTOM_GNSS_I2C_AbortCb);
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

#else

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
  // FIXME: check the return value instead
  (void)CUSTOM_RegisterRxCb(CUSTOM_GNSS_UART_RxCb);
  (void)CUSTOM_RegisterErrorCb(CUSTOM_GNSS_UART_ErrorCb);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

#endif /* USE_I2C */
}

#if (USE_I2C == 1)

#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
void CUSTOM_GNSS_I2C_RxCb(I2C_HandleTypeDef *hi2c)
{
  (void)(hi2c);
  TESEO_LIV3F_I2C_RxCb();
}

void CUSTOM_GNSS_I2C_ErrorCb(I2C_HandleTypeDef *hi2c)
{
  (void)(hi2c);
  TESEO_LIV3F_I2C_ErrorCb();
}

void CUSTOM_GNSS_I2C_AbortCb(I2C_HandleTypeDef *hi2c)
{
  (void)(hi2c);
  TESEO_LIV3F_I2C_AbortCb();
}
#endif /* USE_HAL_I2C_REGISTER_CALLBACKS */

#else

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
void CUSTOM_GNSS_UART_RxCb(UART_HandleTypeDef *huart)
{
  (void)(huart);
  TESEO_LIV3F_UART_RxCb();
}

void CUSTOM_GNSS_UART_ErrorCb(UART_HandleTypeDef *huart)
{
  (void)(huart);
  TESEO_LIV3F_UART_ErrorCb();
}
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

#endif /* USE_I2C */

#endif

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

