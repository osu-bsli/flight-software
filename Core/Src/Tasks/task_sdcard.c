#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <ff.h>
#include <fatfs.h>

#include "flight_software.h"
#include "Tasks/task_sdcard.h"
#include "Tasks/task_sensors.h"
#include "main.h"
#include <stdio.h>
#include <SEGGER_RTT.h>
#include <telemetry.h>

/**
 * task_sdcard.c
 *
 * The STM32 SDMMC driver has some code that blocks for a few seconds if the SD card loses connection during an operation.
 * To avoid freezing other things when that happens, all SDMMC comms are handled in this task.
 */

extern SD_HandleTypeDef hsd1;

static TaskHandle_t handle;
static StaticTask_t tcb;
static StackType_t stack[8192];

static QueueHandle_t queue_handle;
static StaticQueue_t queue;
static uint8_t queue_storage[16384];

static TickType_t time;
const static TickType_t interval_ms = 100; // Flush data every 100 ms

static bool sdcard_is_in_degraded_state;
static uint32_t total_bytes_written;
static bool prev_queue_is_space_available;

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

bool sdcard_get_is_in_degraded_state()
{
  return sdcard_is_in_degraded_state;
}

static void sdcard_mount()
{
  FRESULT fr_status = f_mount(&SDFatFS, SDPath, 1);
  if (fr_status == FR_OK)
  {
    SEGGER_RTT_printf(0, "SD Card f_mount success\n", fr_status);
    sdcard_set_not_degraded();
  }
  else
  {
    switch (fr_status)
    {
    case FR_NOT_READY:
      SEGGER_RTT_printf(0, "SD Card f_mount error: not ready. Perhaps there is no SD card inserted.\n", fr_status);
      SEGGER_RTT_printf(0, "If this error happens after f_mount takes a really long time, maybe the SDMMC DMA cannot access the memory it has been commanded to access.\n");
      break;
    case FR_NO_FILESYSTEM:
      SEGGER_RTT_printf(0, "SD Card f_mount error: there is no filesystem on the SD card\n", fr_status);
      break;
    default:
      SEGGER_RTT_printf(0, "SD Card f_mount error, code: %d\n", fr_status);
      break;
    }
  }
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

  sdcard_mount();

  FRESULT fr_status;

  /* Find a %d filename that is free to use */
  char file_name[16];
  int file_num = 0;
  do
  {
    snprintf(file_name, 16, "%d", file_num);
    file_num++;
  } while ((fr_status = f_stat(file_name, NULL)) == FR_OK);

  /* Open the file */
  FIL file;
  fr_status = f_open(&file, file_name, FA_CREATE_NEW | FA_WRITE);
  if (fr_status == FR_OK)
  {
    SEGGER_RTT_printf(0, "Opened file \"%s\" for telemetry logging\n", file_name);
  }
  else
  {
    SEGGER_RTT_printf(0, "Failed to open file \"%s\", f_open return code: %d\n", file_name, fr_status);
  }

  return file;
}

void sdcard_write_to_log_file(uint8_t *data, uint32_t len)
{
  bool queue_is_space_available = uxQueueSpacesAvailable(queue_handle) >= len;
  if (prev_queue_is_space_available && !queue_is_space_available) {
    SEGGER_RTT_printf(0, "task_sdcard: queue too full, beginning to drop log writes\n");
  }
  prev_queue_is_space_available = queue_is_space_available;
  if (queue_is_space_available)
  {
    for (int i = 0; i < len; i++)
    {
      xQueueSend(queue_handle, &data[i], 0);
    }
  }
}

static void task_sdcard(void *argument)
{
  UNUSED(argument);

  FIL log_file = sdcard_and_logging_init();

  FATFS *fs;
  uint32_t fre_clust;
  f_getfree(SDPath, &fre_clust, &fs);

  /* Get total sectors and free sectors */
  uint32_t tot_sect = (fs->n_fatent - 2) * fs->csize;
  uint32_t fre_sect = fre_clust * fs->csize;

  /* Get sector size */
  uint16_t sect_size;
  disk_ioctl(fs->drv, GET_SECTOR_SIZE, &sect_size);
  
  uint32_t bytes_free = sect_size * fre_sect;
  SEGGER_RTT_printf(0, "task_sdcard: Free bytes: %d\n", bytes_free);

  uint32_t logging_bytes_per_second = 1000 / (float)LOG_INTERVAL_MS * sizeof(struct log_packet);
  uint32_t seconds_of_log_space_remaining = bytes_free / logging_bytes_per_second;
  uint32_t minutes_of_log_space_remaining = seconds_of_log_space_remaining / 60;
  
  SEGGER_RTT_printf(0, "task_sdcard: Minutes of log space remaining: %d\n", minutes_of_log_space_remaining);
  
  while (true)
  {
    uint8_t buf[1024];
    int i = 0;
    while (uxQueueMessagesWaiting(queue_handle) > 0 && i < sizeof(buf))
    {
      xQueueReceive(queue_handle, &buf[i], 0);
      i++;
    }

    unsigned int bytes_written;
    FRESULT fr_status = f_write(&log_file, buf, i, &bytes_written);
    if (bytes_written < 0 || fr_status != FR_OK)
    {
      sdcard_set_degraded();
    }
    total_bytes_written += bytes_written;

    /* flush data to SD card */
    fr_status = f_sync(&log_file);
    if (fr_status != FR_OK)
    {
      sdcard_set_degraded();
    }

    vTaskDelayUntil(&time, interval_ms);
  }
}

void task_sdcard_init(void)
{
  ASSERT(queue_handle == NULL);

  queue_handle = xQueueCreateStatic(
      sizeof(queue_storage),
      1,
      queue_storage,
      &queue);
}

void task_sdcard_start(void)
{
  ASSERT(handle == NULL);

  handle = xTaskCreateStatic(
      task_sdcard,
      "sdcard",
      sizeof(stack) / sizeof(StackType_t),
      NULL,
      tskIDLE_PRIORITY,
      stack,
      &tcb);
}
