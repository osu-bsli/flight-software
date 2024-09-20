#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>

void start_main_task(void const *argument) {
  while (true) {
    vTaskDelay(1);
  }
}
