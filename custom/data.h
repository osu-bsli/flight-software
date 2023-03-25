#ifndef FC_DATA_H
#define FC_DATA_H

struct {
	float timestamp;
	float altitude;
} fc_data_barometer_point;

struct {
	float timestamp;
	float x;
	float y;
	float z;
} fc_data_accelerometer_1_point;

struct {
	float timestamp;
	float x;
	float y;
	float z;
} fc_data_accelerometer_2_point;

struct {
	float timestamp;
	float x;
	float y;
	float z;
} fc_data_magnetometer_point;

struct {
	float timestamp;
	float latitude;
	float longitude;
	int satellite_count; // unsure if needed
	float ground_speed; // unsure if needed
} fc_data_gps_point;

#endif
