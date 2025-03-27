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
#include "Fusion.h"

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
const static TickType_t interval_ms = 10; // 100 Hz

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
    switch (fr_status) {
      case FR_NO_FILESYSTEM:
        SEGGER_RTT_printf(0, "SD Card f_mount error: there is no filesystem on the SD card\n", fr_status);
        break;
      default:
        SEGGER_RTT_printf(0, "SD Card f_mount error, code: %d\n", fr_status);
        break;
    }
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

  bool retry = false;
  int num_retries = 0;

  do {
    if (retry) {
      retry = false;
      num_retries += 1;
      SEGGER_RTT_printf(0, "Retrying to initialize sensors...\n");
    }

    status = fc_bm1422_initialize(&bm1422, &hi2c4, &semaphore_i2c4);
    if (status != HAL_OK) retry = true;
    sensor_print_init_success_state("bm1422", status == HAL_OK);

    status = fc_adxl375_initialize(&adxl375, &hi2c1, &semaphore_i2c1);
    if (status != HAL_OK) retry = true;
    sensor_print_init_success_state("adxl375", status == HAL_OK);

    status = fc_bmi323_initialize(&bmi323, &hi2c1, &semaphore_i2c1);
    if (status != HAL_OK) retry = true;
    sensor_print_init_success_state("bmi323", status == HAL_OK);

    status = fc_ms5607_initialize(&ms5607, &hi2c4, &semaphore_i2c4);
    if (status != HAL_OK) retry = true;
    sensor_print_init_success_state("ms5607", status == HAL_OK);

  } while (retry && num_retries < 10);
}

static void task_sensors(void *argument)
{
  UNUSED(argument);

  /* Create peripheral semaphores */
  semaphore_i2c1 = xSemaphoreCreateBinary();
  semaphore_i2c4 = xSemaphoreCreateBinary();
  semaphore_uart6 = xSemaphoreCreateBinary();

  FIL log_file = sdcard_and_logging_init();
  f_printf(&log_file, "time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,high_g_accel_x,high_g_accel_y,high_g_accel_z,pressure_mbar,temperature_c\n");
  
  sensors_init();

  uint8_t packet_tx_buf[sizeof(struct telemetry_packet)];

  FusionAhrs ahrs;
  FusionAhrsInitialise(&ahrs);

  while (true)
  {
    // SEGGER_RTT_printf(0, "Sensor time (ms): %d\n", time);

    int start_ms = HAL_GetTick();
    struct fc_adxl375_data adxl375_data;
    fc_adxl375_process(&adxl375, &adxl375_data);

    struct fc_bm1422_data bm1422_data;
    fc_bm1422_process(&bm1422, &bm1422_data);

    struct fc_bmi323_data bmi323_data;
    fc_bmi323_process(&bmi323, &bmi323_data);

    struct fc_ms5607_data ms5607_data;
    fc_ms5607_process(&ms5607, &ms5607_data);
    int elapsed_ms = HAL_GetTick() - start_ms;

    // SEGGER_RTT_printf(0, "sensor process time: %d ms\n", elapsed_ms);

    int time_boot_ms = time;

    /* Sensor fusion */
    /* Flip signs and rearrange things as necessary to make sensor axes match FC axes */
    /* FC axes are oriented with +X being from the STM32 to the SD card slot, and +Y being from the STM32 to the SWD port */
    const FusionVector accelerometer = {-bmi323_data.accel_x, -bmi323_data.accel_y, bmi323_data.accel_z};
    const FusionVector gyroscope = {-bmi323_data.gyro_x, -bmi323_data.gyro_y, bmi323_data.gyro_z};
    const FusionVector magnetometer = {-bm1422_data.magnetic_strength_y, bm1422_data.magnetic_strength_x, bm1422_data.magnetic_strength_z};
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, interval_ms / 1000.0);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

    uint8_t status_flags = 0;
    if (sdcard_is_in_degraded_state) status_flags |= STATUS_FLAGS_SD_CARD_DEGRADED;
    if (adxl375.is_in_degraded_state) status_flags |= STATUS_FLAGS_ADXL375_DEGRADED;
    if (bm1422.is_in_degraded_state) status_flags |= STATUS_FLAGS_BM1422_DEGRADED;
    if (bmi323.is_in_degraded_state) status_flags |= STATUS_FLAGS_BMI323_DEGRADED;
    if (ms5607.is_in_degraded_state) status_flags |= STATUS_FLAGS_MS5607_DEGRADED;

    struct telemetry_packet packet = {
        .status_flags = status_flags,
        .time_boot_ms = time_boot_ms,
        .pitch = euler.angle.pitch,
        .yaw = euler.angle.yaw,
        .roll = euler.angle.roll,
        .accel_magnitude = FusionVectorMagnitude(accelerometer)
    };

    telemetry_packet_make_header(&packet);
    
    // Send a telemetry packet if the telemetry UART isn't busy
    if (HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY)
    {
      memcpy(packet_tx_buf, &packet, sizeof(packet));
      HAL_UART_Transmit_IT(&huart6, packet_tx_buf, sizeof(packet));
      HAL_GPIO_TogglePin(GPIO_OUT_LED_BLUE_GPIO_Port, GPIO_OUT_LED_BLUE_Pin);
      // SEGGER_RTT_printf(0, "Sent telemetry packet\n");
    }

    // unsigned int bytes_written_to_sd;
    // FRESULT fr_status = f_write(&log_file, (uint8_t *)&packet, sizeof(packet), &bytes_written_to_sd);
    // if (bytes_written_to_sd < 0 || fr_status != FR_OK)
    // {
    //   sdcard_set_degraded();
    // }

    // /* flush data to SD card */
    // fr_status = f_sync(&log_file);
    // if (fr_status != FR_OK)
    // {
    //   sdcard_set_degraded();
    // }


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