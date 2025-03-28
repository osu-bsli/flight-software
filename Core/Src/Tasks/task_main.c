#include "stm32h7xx_hal.h"
#include "Tasks/task_airbrakes.h"
#include "Tasks/task_blinky.h"
#include "Tasks/task_sensors.h"
#include "Tasks/task_sdcard.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>
#include <SEGGER_RTT.h>
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

void task_main(void const *argument)
{
  UNUSED(argument);

  SEGGER_RTT_printf(0, "Hello World from BSLI!\n");

  task_sdcard_init();

  // task_airbrakes_start();
  task_sensors_start();
  task_sdcard_start();
  task_blinky_start();

  // Suspend the main task.
  vTaskSuspend(NULL);
}
