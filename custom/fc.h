#ifndef FC_H
#define FC_H

#include "data.h"

typedef struct FlightComputer {
	/* latest sensor data points */
	FCData data;
} FlightComputer;

/*
 * Initialization.
 */
void fc_init(FlightComputer *fc);

/*
 * Main process loop.
 */
void fc_process(FlightComputer *fc);

#endif
