#include "flight_software.h"
#include "main.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>

/**
 * task_blinky.c
 *
 * Task for blinking the red debug LED on and off.
 *
 * @author Brian Jia
 */

/*
 * The variables below and the task function MUST be declared static so that
 * code from other .c files can't see them.
 */

/* Structure that will hold the handle of the task being created. */
static TaskHandle_t handle;

/* Structure that will hold the TCB (task metadata) of the task being created. */
static StaticTask_t tcb;

/*
 * Array that the task being created will use as its stack. Note this is
 * an array of StackType_t variables. The size of StackType_t is dependent on
 * the platform. On ARM-based platforms (i.e. STM32), StackType_t is .
 */
#define STACK_SIZE 1024
static StackType_t stack[STACK_SIZE];

/**
 * Function that implements the task being created.
 *
 * @param argument Passed in from the xTaskCreateStatic() call below, but in
 * this case, a NULL value is passed in.
 */
static void task_blinky(void *argument) {
  /* Suppress unused argument warning with UNUSED macro. */
  UNUSED(argument);

  /* Blink the LED. */
  while (true) {
    HAL_GPIO_TogglePin(GPIO_OUT_LED_RED_GPIO_Port, GPIO_OUT_LED_RED_Pin);
    /*
     * Delay for 500 ticks. We have the FreeRTOS tick rate set to 1000 Hz, so
     * this call will delay for 500 milliseconds.
     */
    vTaskDelay(500);
  }
}

/**
 * Function that creates the task.
 */
void task_blinky_start(void) {
  /*
   * Non-local variables are zero-initialized on startup so handle will
   * always be NULL when this function is called for the first time.
   */

  ASSERT(handle == NULL);

  /* Create the task without using any dynamic memory allocation. */
  handle = xTaskCreateStatic(
      task_blinky,             /* Function that implements the task. */
      "blinky",         /* Text name for the task. */
      STACK_SIZE,       /* Number of indexes in the xStack array. */
      NULL,             /* Parameter passed into the task. */
      tskIDLE_PRIORITY, /* Priority at which the task is created. */
      stack,            /* Array to use as the task's stack. */
      &tcb);            /* Variable to hold the task's data structure. */
}
