#include "process.h"
#include "telemetry.h"
#include "barometer.h"

void fc_init(void) {
	fc_telemetry_init();
	fc_barometer_init();
}

/*
 * Main process loop.
 */
void fc_process(void) {
	fc_telemetry_process();
	fc_barometer_process();
}
