#include <SEGGER_RTT.h>
#include "flight_software.h"
#include "Sensors/adxl375.h"
#include "Sensors/bmi323.h"
#include "Sensors/ms5607.h"
#include "Sensors/bm1422.h"
#include "Tasks/tasks.h"
#include "stm32h7xx_hal.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>
#include <ff.h>
#include <fatfs.h>
#include <stdio.h>

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
static struct fc_bmi323 bmi323;
static struct fc_ms5607 ms5607;
static struct fc_bm1422 bm1422;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c4;

static SemaphoreHandle_t semaphore_i2c1;
static SemaphoreHandle_t semaphore_i2c4;

static void sd_card_failed()
{
  HAL_GPIO_WritePin(GPIO_OUT_LED_GREEN_GPIO_Port, GPIO_OUT_LED_GREEN_Pin, 0);
}

static void task_sensors(void *argument)
{
  UNUSED(argument);

  /* Create I2C semaphores */
  semaphore_i2c1 = xSemaphoreCreateBinary();
  semaphore_i2c4 = xSemaphoreCreateBinary();

  /* Set up SD card */

  // TODO: Maybe try re-opening the SD card if it disconnects mid-flight

  /*
   * BRIAN JIA'S NOTES FROM DEBUGGING HELL:
   *
   * DO NOT PLACE PROGRAM RAM INTO DTCM. THE SDMMC DMA CANNOT READ FROM DTCM AND IT WILL FAIL IN WEIRD WAYS.
   * f_mount() WILL TIME OUT FOR NO APPARENT REASON IF PROGRAM RAM AND THUS THE SDMMC DMA BUFFER IS IN DTCM.
   *
   * THAT TOOK A LITERAL YEAR TO DEBUG. FUCK.
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
    /* Turn on green LED to indicate SD card success */
    HAL_GPIO_WritePin(GPIO_OUT_LED_GREEN_GPIO_Port, GPIO_OUT_LED_GREEN_Pin, 1);
  }
  else
  {
    SEGGER_RTT_printf(0, "SD Card f_mount error, code: %d\n", fr_status);
  }

  /* Find a %d.csv filename that is free to use */
  char file_name[16];
  int file_num = 0;
  do
  {
    snprintf(file_name, 16, "%d.csv", file_num);
    file_num++;
  } while ((fr_status = f_stat(file_name, NULL)) == FR_OK);

  /* Open the csv */
  FIL log_csv;
  fr_status = f_open(&log_csv, file_name, FA_CREATE_NEW | FA_WRITE);
  if (fr_status == FR_OK)
  {
    SEGGER_RTT_printf(0, "Opened %s for telemetry logging\n", file_name);
  }
  else
  {
    SEGGER_RTT_printf(0, "Failed to open %s, f_open return code: %d\n", file_name, fr_status);
  }
  f_printf(&log_csv, "time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,high_g_accel_x,high_g_accel_y,high_g_accel_z,pressure_mbar,temperature_c\n");

  /* Initialize sensor drivers */
  HAL_StatusTypeDef status;
  status = fc_bm1422_initialize(&bm1422, &hi2c4, &semaphore_i2c4);
  if (status == HAL_OK)
  {
    SEGGER_RTT_printf(0, "bm1422 initialization success\n");
  }
  else
  {
    SEGGER_RTT_printf(0, "bm1422 initialization failed\n");
  }
  status = fc_adxl375_initialize(&adxl375, &hi2c1, &semaphore_i2c1);
  if (status == HAL_OK)
  {
    SEGGER_RTT_printf(0, "adxl375 initialization success\n");
  }
  else
  {
    SEGGER_RTT_printf(0, "adxl375 initialization failed\n");
  }
  status = fc_bmi323_initialize(&bmi323, &hi2c1, &semaphore_i2c1);
  if (status == HAL_OK)
  {
    SEGGER_RTT_printf(0, "bmi323 initialization success\n");
  }
  else
  {
    SEGGER_RTT_printf(0, "bmi323 initialization failed\n");
  }
  status = fc_ms5607_initialize(&ms5607, &hi2c4, &semaphore_i2c4);
  if (status == HAL_OK)
  {
    SEGGER_RTT_printf(0, "ms5607 initialization success\n");
  }
  else
  {
    SEGGER_RTT_printf(0, "ms5607 initialization failed\n");
  }

  while (true)
  {
    // SEGGER_RTT_printf(0, "Sensor time (ms): %d\n", time);

    struct fc_adxl375_data adxl375_data;
    fc_adxl375_process(&adxl375, &adxl375_data);

    struct fc_ms5607_data ms5607_data;
    fc_ms5607_process(&ms5607, &ms5607_data);

    struct fc_bmi323_data bmi323_data;
    fc_bmi323_process(&bmi323, &bmi323_data);

    struct fc_bm1422_data bm1422_data;
    fc_bm1422_process(&bm1422, &bm1422_data);

    int time_ms = time;
    float accel_x = bmi323_data.accel_x;
    float accel_y = bmi323_data.accel_x;
    float accel_z = bmi323_data.accel_x;
    float gyro_x = bmi323_data.gyro_x;
    float gyro_y = bmi323_data.gyro_y;
    float gyro_z = bmi323_data.gyro_z;
    float high_g_accel_x = adxl375_data.acceleration_x;
    float high_g_accel_y = adxl375_data.acceleration_y;
    float high_g_accel_z = adxl375_data.acceleration_z;
    float pressure_mbar = ms5607_data.pressure_mbar;
    float temperature_c = ms5607_data.temperature_c;
    char buf[256];
    /* Use snprintf because f_printf() does not support floats */
    snprintf(buf, 256, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
             time_ms,
             accel_x,
             accel_y,
             accel_z,
             gyro_x,
             gyro_y,
             gyro_z,
             high_g_accel_x,
             high_g_accel_y,
             high_g_accel_z,
             pressure_mbar,
             temperature_c);
    uint32_t chars_printed = f_printf(&log_csv, "%s", buf);
    if (chars_printed < 0)
    {
      sd_card_failed();
    }

    /* flush data to SD card */
    fr_status = f_sync(&log_csv);
    if (fr_status != FR_OK)
    {
      sd_card_failed();
    }

    vTaskDelayUntil(&time, interval_ms);

    HAL_GPIO_TogglePin(GPIO_OUT_LED_BLUE_GPIO_Port, GPIO_OUT_LED_BLUE_Pin);
  }
}

void task_sensors_start(void)
{
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

static void i2c_transfer_complete(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1)
  {
    xSemaphoreGiveFromISR(semaphore_i2c1, NULL);
  }
  else if (hi2c == &hi2c4)
  {
    xSemaphoreGiveFromISR(semaphore_i2c4, NULL);
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  i2c_transfer_complete(hi2c);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  i2c_transfer_complete(hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  i2c_transfer_complete(hi2c);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  i2c_transfer_complete(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  SEGGER_RTT_printf(0, "I2C non-blocking error\n");
}