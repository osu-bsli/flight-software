#include "telemetry.h"
#include "packet-parser/parser.h"

void fc_telemetry_init(void) {
	packetlib_initialize();
}

void fc_telemetry_process(void) {
	packetlib_process();
}
