#include "process.h"
#include "telemetry.h"
#include "barometer.h"
#include "recovery.h"
#include "fc.h"

void fc_init(FlightComputer *fc) {
	fc_telemetry_init(fc);
	fc_barometer_init();
}

/*
 * Main process loop.
 */
void fc_process(FlightComputer *fc) {
	fc_telemetry_process(fc);
	fc_barometer_process();
	fc_recovery_process();
}
