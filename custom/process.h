#ifndef FC_PROCESS_H
#define FC_PROCESS_H

#include "fc.h"

/*
 * Initialization.
 */
void fc_init(FlightComputer *fc);

/*
 * Main process loop.
 */
void fc_process(FlightComputer *fc);

#endif
