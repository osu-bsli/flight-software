#include "External/RTT/SEGGER_RTT.h"
#include "Sensors/adxl375.h"
#include "Tasks/tasks.h"
#include "flight_software.h"
#include "stm32h7xx_hal.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>

/*
 * task_sensors.c
 * 
 * Task that runs periodically to collect sensor data and make it available for
 * other tasks, for example the telemetry task and the airbrake task.
 */

static TaskHandle_t handle;
static StaticTask_t tcb;
static StackType_t stack[STACK_SIZE];

static TickType_t time;
const static TickType_t interval_ms = 100; // 10 Hz, we want 100 Hz eventually

static struct fc_adxl375 adxl375;

extern I2C_HandleTypeDef hi2c1;

static void task_sensors(void *argument) {
  UNUSED(argument);

  fc_adxl375_initialize(&adxl375, &hi2c1);

  while (true) {
    SEGGER_RTT_printf(0, "Sensor time (ms): %d\n", time);

    fc_adxl375_process(&adxl375);

    vTaskDelayUntil(&time, interval_ms);
  }
}

void task_sensors_start(void) {
  ASSERT(handle == NULL);

  handle = xTaskCreateStatic(
      task_sensors,     /* Function that implements the task. */
      "blinky",         /* Text name for the task. */
      STACK_SIZE,       /* Number of indexes in the xStack array. */
      NULL,             /* Parameter passed into the task. */
      tskIDLE_PRIORITY, /* Priority at which the task is created. */
      stack,            /* Array to use as the task's stack. */
      &tcb);            /* Variable to hold the task's data structure. */
}
