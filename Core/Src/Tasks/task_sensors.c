#include <SEGGER_RTT.h>
#include "flight_software.h"
#include "Tasks/task_sensors.h"
#include "Tasks/task_sdcard.h"
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
const static TickType_t interval_ms = LOG_INTERVAL_MS; // 100 Hz

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

  } while (retry && num_retries < 20);
}

static void task_sensors(void *argument)
{
  UNUSED(argument);

  /* Create peripheral semaphores */
  semaphore_i2c1 = xSemaphoreCreateBinary();
  semaphore_i2c4 = xSemaphoreCreateBinary();
  semaphore_uart6 = xSemaphoreCreateBinary();
  
  sensors_init();

  uint8_t packet_tx_buf[sizeof(struct telemetry_packet)];

  // Initialise sensor fusion algorithm
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
    const FusionVector gyroscope = {-bmi323_data.gyro_x, -bmi323_data.gyro_y, -bmi323_data.gyro_z};
    const FusionVector magnetometer = {-bm1422_data.magnetic_strength_y, bm1422_data.magnetic_strength_x, bm1422_data.magnetic_strength_z};
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, interval_ms / 1000.0);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    uint8_t status_flags = 0;
    if (sdcard_get_is_in_degraded_state()) status_flags |= STATUS_FLAGS_SD_CARD_DEGRADED;
    if (adxl375.is_in_degraded_state) status_flags |= STATUS_FLAGS_ADXL375_DEGRADED;
    if (bm1422.is_in_degraded_state) status_flags |= STATUS_FLAGS_BM1422_DEGRADED;
    if (bmi323.is_in_degraded_state) status_flags |= STATUS_FLAGS_BMI323_DEGRADED;
    if (ms5607.is_in_degraded_state) status_flags |= STATUS_FLAGS_MS5607_DEGRADED;

    // pitch <- fused roll
    // yaw <- fused pitch
    // roll <- fused yaw
    struct telemetry_packet tele_p = {
        .status_flags = status_flags,
        .time_boot_ms = time_boot_ms,
        .pitch = euler.angle.roll,
        .yaw = euler.angle.pitch,
        .roll = euler.angle.yaw,
        .accel_magnitude = FusionVectorMagnitude(accelerometer),
        .ms5607_pressure_mbar = ms5607_data.pressure_mbar
    };
    telemetry_packet_make_header(&tele_p);
    
    // Send a telemetry packet if the telemetry UART isn't busy
    if (HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY)
    {
      memcpy(packet_tx_buf, &tele_p, sizeof(tele_p));
      HAL_UART_Transmit_IT(&huart6, packet_tx_buf, sizeof(tele_p));
      HAL_GPIO_TogglePin(GPIO_OUT_LED_BLUE_GPIO_Port, GPIO_OUT_LED_BLUE_Pin);
      // SEGGER_RTT_printf(0, "Sent telemetry packet\n");
    }

    struct log_packet log_p = {
      .status_flags = status_flags,
      .time_boot_ms = time_boot_ms,
      .ms5607_pressure_mbar = ms5607_data.pressure_mbar,
      .ms5607_temperature_c = ms5607_data.temperature_c,
      .bmi323_accel_x = bmi323_data.accel_x,
      .bmi323_accel_y = bmi323_data.accel_y,
      .bmi323_accel_z = bmi323_data.accel_z,
      .bmi323_gyro_x = bmi323_data.gyro_x,
      .bmi323_gyro_y = bmi323_data.gyro_y,
      .bmi323_gyro_z = bmi323_data.gyro_z,
      .adxl375_accel_x = adxl375_data.accel_x,
      .adxl375_accel_y = adxl375_data.accel_y,
      .adxl375_accel_z = adxl375_data.accel_z,
    };
    logging_packet_make_header(&log_p);
    sdcard_write_to_log_file((uint8_t*)&log_p, sizeof(log_p));

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
      3, /* Priority at which the task is created. */
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