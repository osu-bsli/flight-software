#include "stm32h7xx_hal.h"
#include "task_blinky.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>

/**
 * task_main.c
 *
 * The ONLY purpose of this task will be to perform basic initialization and
 * call other task start functions.
 *
 * This is the only task where we are using STM32CubeMX generated code to
 * provide the stack buffer, task buffer, and initialization code.
 */

void task_main(void const *argument) {
  UNUSED(argument);

  task_blinky_start();

  // Suspend the main task.
  vTaskSuspend(NULL);
}
