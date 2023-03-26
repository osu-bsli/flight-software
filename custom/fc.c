#include "fc.h"
#include "telemetry.h"
#include "barometer.h"
#include "recovery.h"

void fc_init(FlightComputer *fc) {
	fc_telemetry_init(fc);
	fc_barometer_init(fc);
}

/*
 * Main process loop.
 */
void fc_process(FlightComputer *fc) {
	fc_telemetry_process(fc);
	fc_barometer_process(fc);
	/* fc_recovery_process(fc); */
}
