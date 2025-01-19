#include "stm32h7xx_hal.h"
#include "Tasks/task_blinky.h"
#include "Tasks/task_sensors.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>
#include <SEGGER_RTT.h>
#include <mavlink_packets/mavlink.h>
#include <ff.h>
#include <fatfs.h>

/**
 * task_main.c
 *
 * The ONLY purpose of this task will be to perform basic initialization and
 * call other task start functions, then suspend itself.
 *
 * This is the only task where we are using STM32CubeMX generated code to
 * provide the stack buffer, task buffer, and initialization code.
 */

void initialize(void)
{
  SEGGER_RTT_printf(0, "Hello World from BSLI!\n");

  /*
   * SD Card Debugging Notes (Brian Jia)
   
    f_mount returns FR_DISK_ERR (1) with SD card plugged in after a long delay
    It returns FR_NOT_READY (3) with no SD card plugged in
   */

  if (BSP_SD_IsDetected())
  {
    SEGGER_RTT_printf(0, "SD Card is SUCCESSFULLY detected\n");
  }
  else
  {
    SEGGER_RTT_printf(0, "SD Card is NOT detected\n");
  }

  FRESULT fr_status = f_mount(&SDFatFS, SDPath, 1);
  if (fr_status == FR_OK)
  {
    SEGGER_RTT_printf(0, "SD Card f_mount success\n", fr_status);
  }
  else
  {
    SEGGER_RTT_printf(0, "SD Card f_mount error, code: %d\n", fr_status);
  }

  FIL test;
  fr_status = f_open(&test, "test.txt", FA_WRITE);
  if (fr_status == FR_OK)
  {
    SEGGER_RTT_printf(0, "fopen test.txt success\n");
  }
  else
  {
    SEGGER_RTT_printf(0, "fopen test.txt failed to open\n");
  }

  f_printf(&test, "Hello World BSLI! Current tick time: %d\n", HAL_GetTick());
  f_close(&test);
}

void task_main(void const *argument)
{
  UNUSED(argument);

  initialize();

  task_sensors_start();
  task_blinky_start();

  // Suspend the main task.
  vTaskSuspend(NULL);
}
