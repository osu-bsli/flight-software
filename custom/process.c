#include "process.h"
#include "telemetry.h"

void fc_init() {
	fc_telemetry_init();
}

/*
 * Main process loop.
 */
void fc_process() {
	fc_telemetry_process();
}
