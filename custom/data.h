#ifndef FC_DATA_H
#define FC_DATA_H

typedef struct FCDataBarometerPoint{
	float timestamp;
	float altitude;
} FCDataBarometerPoint;

typedef struct FCDataPrimaryAccelerometerPoint {
	float timestamp;
	float x;
	float y;
	float z;
} FCDataPrimaryAccelerometerPoint;

typedef struct FCDataSecondaryAccelerometerPoint {
	float timestamp;
	float x;
	float y;
	float z;
} FCDataSecondaryAccelerometerPoint;

typedef struct FCDataMagnetometerPoint {
	float timestamp;
	float x;
	float y;
	float z;
} FCDataMagnetometerPoint;

typedef struct FCDataGPSPoint {
	float timestamp;
	float latitude;
	float longitude;
	int satellite_count; // unsure if needed
	float ground_speed; // unsure if needed
} FCDataGPSPoint;

#endif
