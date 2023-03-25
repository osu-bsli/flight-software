#include "telemetry.h"
#include "packet-parser/parser.h"

void telemetry_initialize(void) {
	packetlib_initialize();
}

void telemetry_process(void) {
	packetlib_process();
}
