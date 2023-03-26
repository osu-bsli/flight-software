#ifndef FC_DATA_H
#define FC_DATA_H

typedef struct FCSensorData {
	/* barometer */
	float barometer_timestamp;
	float barometer_altitude;

	/* primary accelerometer */
	float primary_accelerometer_timestamp;
	float primary_accelerometer_x;
	float primary_accelerometer_y;
	float primary_accelerometer_z;

	/* secondary accelerometer */
	float secondary_accelerometer_timestamp;
	float secondary_accelerometer_x;
	float secondary_accelerometer_y;
	float secondary_accelerometer_z;

	/* gps */
	float gps_timestamp;
	float gps_latitude;
	float gps_longitude;
	int gps_satellite_count;
	float gps_ground_speed;

	/* board temperature */
	float board_temp_timestamp;
	float board_1_temp;
	float board_2_temp;
	float board_3_temp;
	float board_4_temp;

	/* board voltage */
	float board_voltage_timestamp;
	float board_1_voltage;
	float board_2_voltage;
	float board_3_voltage;
	float board_4_voltage;

	/* board current */
	float board_current_timestamp;
	float board_1_current;
	float board_2_current;
	float board_3_current;
	float board_4_current;

	/* battery voltage */
	float battery_voltage_timestamp;
	float battery_1_voltage;
	float battery_2_voltage;
	float battery_3_voltage;

	/* magnetometer voltage */
	float magnetometer_timestamp;
	float magnetometer_x;
	float magnetometer_y;
	float magnetometer_z;

	/* gyroscope */
	float gyroscope_timestamp;
	float gyroscope_x;
	float gyroscope_y;
	float gyroscope_z;
} FCData;

#endif
