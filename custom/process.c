#include "process.h"
#include "telemetry.h"

void fc_init(void) {
	fc_telemetry_init();
}

/*
 * Main process loop.
 */
void fc_process(void) {
	fc_telemetry_process();
}
