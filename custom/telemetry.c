#include "telemetry.h"
#include "data.h"
#include "fc.h"
#include "packet-parser/parser.h"
#include "packet-parser/packet.h"
#include <stdint.h>

static void fc_telemetry_send_packet(uint8_t *packet_bytes, int packet_size) {
	/* TODO: Use DMA mode to prevent blocking */
	/* HAL_UART_Transmit(???, packet_bytes, packet_size, Timeout); */
}

void fc_telemetry_init(FlightComputer *fc) {
	packetlib_initialize();
}

/* avoiding sending repeat data by storing its timestamp each time */
float latest_accelerometer_data_timestamp = 0.0f;
float latest_barometer_data_timestamp = 0.0f;
float latest_magnetometer_data_timestamp = 0.0f;
float latest_gps_data_timestamp = 0.0f;

void fc_telemetry_process(FlightComputer *fc) {
	packetlib_process();

	/* byte buffer to hold written packets before being sent */
	uint8_t packet_bytes[64];
	int packet_size = 0;

	/* TODO: Make packet sending atomic */

	/* TODO: Split altitude packets to handle each sensor separately */
	if (latest_barometer_data_timestamp < fc->latest_barometer_data.timestamp) {
		packet_size = write_altitude_packet(
				packet_bytes,
				fc->latest_barometer_data.timestamp,
				fc->latest_barometer_data.altitude,
				0.0f);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		latest_barometer_data_timestamp = fc->latest_barometer_data.timestamp;
	}

	/* TODO: Add another packet for the second accelerometer */
	if (latest_accelerometer_data_timestamp < fc->latest_primary_accelerometer_data.timestamp) {
		packet_size = write_acceleration_packet(
				packet_bytes,
				fc->latest_primary_accelerometer_data.timestamp,
				fc->latest_primary_accelerometer_data.x,
				fc->latest_primary_accelerometer_data.y,
				fc->latest_primary_accelerometer_data.z);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		latest_accelerometer_data_timestamp = fc->latest_primary_accelerometer_data.timestamp;
	}

	/* TODO: Combine all gps packets into 1 */
	if (latest_gps_data_timestamp < fc->latest_gps_data.timestamp) {
		packet_size = write_gps_position_packet(
				packet_bytes,
				fc->latest_gps_data.timestamp,
				fc->latest_gps_data.latitude,
				fc->latest_gps_data.longitude);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		packet_size = write_gps_ground_speed_packet(
				packet_bytes,
				fc->latest_gps_data.timestamp,
				fc->latest_gps_data.ground_speed);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		packet_size = write_gps_satellite_count_packet(
				packet_bytes,
				fc->latest_gps_data.timestamp,
				fc->latest_gps_data.satellite_count);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		latest_gps_data_timestamp = fc->latest_gps_data.timestamp;
	}
}
