#include "stm32h7xx_hal.h"
#include "Tasks/task_blinky.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>
#include "External/RTT/SEGGER_RTT.h"
#include "rust_bindings.h"

/**
 * task_main.c
 *
 * The ONLY purpose of this task will be to perform basic initialization and
 * call other task start functions, then suspend itself.
 *
 * This is the only task where we are using STM32CubeMX generated code to
 * provide the stack buffer, task buffer, and initialization code.
 */

void initialize(void) {
  SEGGER_RTT_printf(0, "Hello World from BSLI!\n");

  AirbrakeData a = {
    .altitude = 123.45,
    .angle = 40,
    .speed = 54.321
  };

  uint32_t are_we_linking = airbrake_calculate(&a);

  SEGGER_RTT_printf(0, "Rust/C interop test: %i\n", are_we_linking);
}

void task_main(void const *argument) {
  UNUSED(argument);

  initialize();

  task_blinky_start();

  // Suspend the main task.
  vTaskSuspend(NULL);
}
