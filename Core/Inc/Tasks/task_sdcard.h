#pragma once

#include <stdbool.h>

void task_sdcard_init(void);
void task_sdcard_start(void);
bool sdcard_get_is_in_degraded_state(void);
void sdcard_write_to_log_file(uint8_t *data, uint32_t len);