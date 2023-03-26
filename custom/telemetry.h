#ifndef FC_TELEMETRY_H
#define FC_TELEMETRY_H

#include "fc.h"

void fc_telemetry_send_packet(uint8_t *packet_bytes, int packet_size);
void fc_telemetry_init(FlightComputer *fc);
void fc_telemetry_process(FlightComputer *fc);

#endif
