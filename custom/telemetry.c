#include "telemetry.h"
#include "data.h"
#include "packet-parser/parser.h"
#include "packet-parser/packet.h"
#include <stdint.h>

static void fc_telemetry_send_packet(uint8_t *packet_bytes, int packet_size) {
	/* TODO: Use DMA mode to prevent blocking */
	/* HAL_UART_Transmit(???, packet_bytes, packet_size, Timeout); */
}

void fc_telemetry_init(void) {
	packetlib_initialize();
}

/* avoiding sending repeat data by storing its timestamp each time */
float latest_accelerometer_data_timestamp = 0.0f;
float latest_barometer_data_timestamp = 0.0f;
float latest_magnetometer_data_timestamp = 0.0f;
float latest_gps_data_timestamp = 0.0f;

void fc_telemetry_process(void) {
	packetlib_process();

	/* byte buffer to hold written packets before being sent */
	uint8_t packet_bytes[64];
	int packet_size = 0;

	/* TODO: Make packet sending atomic */

	/* TODO: Split altitude packets to handle each sensor separately */
	if (latest_barometer_data_timestamp < fc_data_barometer_point.timestamp) {
		packet_size = write_altitude_packet(
				packet_bytes,
				fc_data_barometer_point.timestamp,
				fc_data_barometer_point.altitude,
				0.0f);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		latest_barometer_data_timestamp = fc_data_barometer_point.timestamp;
	}

	/* TODO: Add another packet for the second accelerometer */
	if (latest_accelerometer_data_timestamp < fc_data_accelerometer_1_point.timestamp) {
		packet_size = write_acceleration_packet(
				packet_bytes,
				fc_data_accelerometer_1_point.timestamp,
				fc_data_accelerometer_1_point.x,
				fc_data_accelerometer_1_point.y,
				fc_data_accelerometer_1_point.z);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		latest_accelerometer_data_timestamp = fc_data_accelerometer_1_point.timestamp;
	}

	/* TODO: Combine all gps packets into 1 */
	if (latest_gps_data_timestamp < fc_data_gps_point.timestamp) {
		packet_size = write_gps_position_packet(
				packet_bytes,
				fc_data_gps_point.timestamp,
				fc_data_gps_point.latitude,
				fc_data_gps_point.longitude);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		packet_size = write_gps_ground_speed_packet(
				packet_bytes,
				fc_data_gps_point.timestamp,
				fc_data_gps_point.ground_speed);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		packet_size = write_gps_satellite_count_packet(
				packet_bytes,
				fc_data_gps_point.timestamp,
				fc_data_gps_point.satellite_count);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		latest_gps_data_timestamp = fc_data_gps_point.timestamp;
	}
}
