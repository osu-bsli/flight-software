#include <SEGGER_RTT.h>
#include "flight_software.h"
#include "Tasks/task_sensors.h"
#include "checksum.h"
#include "Sensors/adxl375.h"
#include "Sensors/bmi323.h"
#include "Sensors/ms5607.h"
#include "Sensors/bm1422.h"
#include "telemetry.h"
#include "stm32h7xx_hal.h"
#include <FreeRTOS.h>
#include <stdbool.h>
#include <task.h>
#include <ff.h>
#include <fatfs.h>
#include <string.h>
#include <stdio.h>
#include "Airbrakes/airbrakes.h"


/*
* task_sensors.c
*
* Task that runs periodically to collect sensor data and make it available for
* other tasks, for example the telemetry task and the airbrake task.
*/

static TaskHandle_t handle;
static StaticTask_t tcb;
#define STACK_SIZE 65536
static StackType_t stack[STACK_SIZE];

static TickType_t time;
const static TickType_t interval_ms = 55; // ~18 Hz, we want 100 Hz eventually

static struct fc_adxl375 adxl375;
static struct fc_bmi323 bmi323;
static struct fc_ms5607 ms5607;
static struct fc_bm1422 bm1422;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c4;
extern UART_HandleTypeDef huart6;

static SemaphoreHandle_t semaphore_i2c1;
static SemaphoreHandle_t semaphore_i2c4;
static SemaphoreHandle_t semaphore_uart6;

static bool sdcard_is_in_degraded_state;

static void sdcard_set_not_degraded()
{
  /* Turn on green LED to indicate SD card success */
  sdcard_is_in_degraded_state = false;
  HAL_GPIO_WritePin(GPIO_OUT_LED_GREEN_GPIO_Port, GPIO_OUT_LED_GREEN_Pin, 1);
}

static void sdcard_set_degraded()
{
  sdcard_is_in_degraded_state = true;
  HAL_GPIO_WritePin(GPIO_OUT_LED_GREEN_GPIO_Port, GPIO_OUT_LED_GREEN_Pin, 0);
}

static FIL sdcard_and_logging_init()
{

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
    sdcard_set_not_degraded();
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

  return log_csv;
}

static void sensor_print_init_success_state(const char* name, bool was_successful) {
  if (was_successful)
  {
    SEGGER_RTT_printf(0, "%s initialization succeeded\n", name);
  }
  else
  {
    SEGGER_RTT_printf(0, "%s initialization failed\n", name);
  }
}

static void sensors_init()
{
  /* Initialize sensor drivers */
  HAL_StatusTypeDef status;
  
  status = fc_bm1422_initialize(&bm1422, &hi2c4, &semaphore_i2c4);
  sensor_print_init_success_state("bm1422", status == HAL_OK);

  status = fc_adxl375_initialize(&adxl375, &hi2c1, &semaphore_i2c1);
  sensor_print_init_success_state("adxl375", status == HAL_OK);

  status = fc_bmi323_initialize(&bmi323, &hi2c1, &semaphore_i2c1);
  sensor_print_init_success_state("bmi323", status == HAL_OK);

  status = fc_ms5607_initialize(&ms5607, &hi2c4, &semaphore_i2c4);
  sensor_print_init_success_state("ms5607", status == HAL_OK);
}

static void task_sensors(void *argument)
{
  UNUSED(argument);

  /* Create peripheral semaphores */
  semaphore_i2c1 = xSemaphoreCreateBinary();
  semaphore_i2c4 = xSemaphoreCreateBinary();
  semaphore_uart6 = xSemaphoreCreateBinary();

  FIL log_csv = sdcard_and_logging_init();
  f_printf(&log_csv, "time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,high_g_accel_x,high_g_accel_y,high_g_accel_z,pressure_mbar,temperature_c\n");
  
  sensors_init();

  while (true)
  {
    // SEGGER_RTT_printf(0, "Sensor time (ms): %d\n", time);

    struct fc_adxl375_data adxl375_data;
    fc_adxl375_process(&adxl375, &adxl375_data);

    struct fc_bm1422_data bm1422_data;
    fc_bm1422_process(&bm1422, &bm1422_data);

    struct fc_bmi323_data bmi323_data;
    fc_bmi323_process(&bmi323, &bmi323_data);

    struct fc_ms5607_data ms5607_data;
    fc_ms5607_process(&ms5607, &ms5607_data);

    int time_boot_ms = time;
    float accel_x = bmi323_data.accel_x;
    float accel_y = bmi323_data.accel_y;
    float accel_z = bmi323_data.accel_z;
    float gyro_x = bmi323_data.gyro_x;
    float gyro_y = bmi323_data.gyro_y;
    float gyro_z = bmi323_data.gyro_z;
    float high_g_accel_x = adxl375_data.accel_x;
    float high_g_accel_y = adxl375_data.accel_y;
    float high_g_accel_z = adxl375_data.accel_z;
    float pressure_mbar = ms5607_data.pressure_mbar;
    float temperature_c = ms5607_data.temperature_c;
    // TODO: Add bm1422 data to telemetry packet
    char buf[256];
    /* Use snprintf because f_printf() does not support floats */
    uint32_t chars_printed = snprintf(buf, 256, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                                      time_boot_ms,
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

    uint8_t status_flags = 0;
    if (sdcard_is_in_degraded_state) status_flags |= STATUS_FLAGS_SD_CARD_DEGRADED;
    if (adxl375.is_in_degraded_state) status_flags |= STATUS_FLAGS_ADXL375_DEGRADED;
    if (bm1422.is_in_degraded_state) status_flags |= STATUS_FLAGS_BM1422_DEGRADED;
    if (bmi323.is_in_degraded_state) status_flags |= STATUS_FLAGS_BMI323_DEGRADED;
    if (ms5607.is_in_degraded_state) status_flags |= STATUS_FLAGS_MS5607_DEGRADED;

    struct telemetry_packet packet = {
        .status_flags = status_flags,
        .time_boot_ms = time_boot_ms,
        .ms5607_pressure_mbar = pressure_mbar,
        .ms5607_temperature_c = temperature_c,
        .bmi323_accel_x = accel_x,
        .bmi323_accel_y = accel_y,
        .bmi323_accel_z = accel_z,
        .bmi323_gyro_x = gyro_x,
        .bmi323_gyro_y = gyro_y,
        .bmi323_gyro_z = gyro_z,
        .adxl375_accel_x = accel_x,
        .adxl375_accel_y = accel_y,
        .adxl375_accel_z = accel_z,
    };

    telemetry_packet_make_header(&packet);

    HAL_UART_Transmit_IT(&huart6, (uint8_t *)&packet, sizeof(packet));
    SEGGER_RTT_printf(0, "Telemetry packet sent: %d bytes\n", sizeof(packet));

    uint32_t chars_written_to_sd = f_printf(&log_csv, "%s", buf);
    if (chars_written_to_sd < 0)
    {
      sdcard_set_degraded();
    }

    /* flush data to SD card */
    FRESULT fr_status = f_sync(&log_csv);
    if (fr_status != FR_OK)
    {
      sdcard_set_degraded();
    }

    HAL_GPIO_TogglePin(GPIO_OUT_LED_BLUE_GPIO_Port, GPIO_OUT_LED_BLUE_Pin);

    // Wait for UART to be done (FreeRTOS)
    xSemaphoreTake(semaphore_uart6, portMAX_DELAY);

    vTaskDelayUntil(&time, interval_ms);
  }
}

void task_sensors_start(void)
{
  ASSERT(handle == NULL);

  handle = xTaskCreateStatic(
      task_sensors,     /* Function that implements the task. */
      "sensors",         /* Text name for the task. */
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart6)
  {
    xSemaphoreGiveFromISR(semaphore_uart6, NULL);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  SEGGER_RTT_printf(0, "HAL_I2C_ErrorCallback called\n");
}