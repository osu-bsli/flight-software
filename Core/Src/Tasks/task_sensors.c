#include "Tasks/tasks.h"
#include "flight_software.h"
#include "stm32h7xx_hal.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>

static TaskHandle_t handle;
static StaticTask_t tcb;
static StackType_t stack[STACK_SIZE];

static void task_sensors(void *argument) {
  UNUSED(argument);

  while (true) {
    
  }
}

void task_sensors_start(void) {
  ASSERT(handle == NULL);

  handle = xTaskCreateStatic(
      task_sensors,             /* Function that implements the task. */
      "blinky",         /* Text name for the task. */
      STACK_SIZE,       /* Number of indexes in the xStack array. */
      NULL,             /* Parameter passed into the task. */
      tskIDLE_PRIORITY, /* Priority at which the task is created. */
      stack,            /* Array to use as the task's stack. */
      &tcb);            /* Variable to hold the task's data structure. */
}
