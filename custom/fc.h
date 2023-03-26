#ifndef FC_H
#define FC_H

#include "data.h"

typedef struct FlightComputer {
	/* latest sensor data points */
	FCDataBarometerPoint latest_barometer_data;
	FCDataPrimaryAccelerometerPoint latest_primary_accelerometer_data;
	FCDataSecondaryAccelerometerPoint latest_secondary_accelerometer_data;
	FCDataMagnetometerPoint latest_magnetometer_data;
	FCDataGPSPoint latest_gps_data;
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
