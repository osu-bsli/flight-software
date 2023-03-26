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
float latest_board_temp_timestamp = 0.0f;
float latest_board_voltage_timestamp = 0.0f;
float latest_board_current_timestamp = 0.0f;
float latest_battery_voltage_timestamp = 0.0f;
float latest_magnetometer_timestamp = 0.0f;
float latest_gyroscope_timestamp = 0.0f;

void fc_telemetry_process(FlightComputer *fc) {
	packetlib_process();

	/* byte buffer to hold written packets before being sent */
	uint8_t packet_bytes[64];
	int packet_size = 0;

	/* TODO: Make packet sending atomic */

	/* TODO: Split altitude packets to handle each sensor separately */
	if (latest_barometer_data_timestamp < fc->data.barometer_timestamp) {
		latest_barometer_data_timestamp = fc->data.barometer_timestamp;

		packet_size = write_altitude_packet(
				packet_bytes,
				fc->data.barometer_timestamp,
				fc->data.barometer_altitude,
				0.0f);
		fc_telemetry_send_packet(packet_bytes, packet_size);
	}

	/* TODO: Add another packet for the second accelerometer */
	if (latest_accelerometer_data_timestamp < fc->data.primary_accelerometer_timestamp) {
		latest_accelerometer_data_timestamp = fc->data.primary_accelerometer_timestamp;

		packet_size = write_acceleration_packet(
				packet_bytes,
				fc->data.primary_accelerometer_timestamp,
				fc->data.primary_accelerometer_x,
				fc->data.primary_accelerometer_y,
				fc->data.primary_accelerometer_z);
		fc_telemetry_send_packet(packet_bytes, packet_size);
	}

	/* TODO: Combine all gps packets into 1 */
	if (latest_gps_data_timestamp < fc->data.gps_timestamp) {
		latest_gps_data_timestamp = fc->data.gps_timestamp;

		packet_size = write_gps_position_packet(
				packet_bytes,
				fc->data.gps_timestamp,
				fc->data.gps_latitude,
				fc->data.gps_longitude);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		packet_size = write_gps_ground_speed_packet(
				packet_bytes,
				fc->data.gps_timestamp,
				fc->data.gps_ground_speed);
		fc_telemetry_send_packet(packet_bytes, packet_size);

		packet_size = write_gps_satellite_count_packet(
				packet_bytes,
				fc->data.gps_timestamp,
				fc->data.gps_satellite_count);
		fc_telemetry_send_packet(packet_bytes, packet_size);
	}

	/* TODO Combine board measurements into packets per-board if possible */
	if (latest_board_temp_timestamp < fc->data.board_temp_timestamp) {
		latest_board_temp_timestamp = fc->data.board_temp_timestamp;

		packet_size = write_board_temperature_packet(
				packet_bytes,
				fc->data.board_temp_timestamp,
				fc->data.board_1_temp,
				fc->data.board_2_temp,
				fc->data.board_3_temp,
				fc->data.board_4_temp);
		fc_telemetry_send_packet(packet_bytes, packet_size);
	}
	if (latest_board_voltage_timestamp < fc->data.board_voltage_timestamp) {
		latest_board_voltage_timestamp = fc->data.board_voltage_timestamp;

		packet_size = write_board_voltage_packet(
				packet_bytes,
				fc->data.board_voltage_timestamp,
				fc->data.board_1_voltage,
				fc->data.board_2_voltage,
				fc->data.board_3_voltage,
				fc->data.board_4_voltage);
		fc_telemetry_send_packet(packet_bytes, packet_size);
	}
	if (latest_board_current_timestamp < fc->data.board_current_timestamp) {
		latest_board_current_timestamp = fc->data.board_current_timestamp;

		packet_size = write_board_current_packet(
				packet_bytes,
				fc->data.board_current_timestamp,
				fc->data.board_1_current,
				fc->data.board_2_current,
				fc->data.board_3_current,
				fc->data.board_4_current);
		fc_telemetry_send_packet(packet_bytes, packet_size);
	}

	if (latest_battery_voltage_timestamp < fc->data.battery_voltage_timestamp) {
		latest_battery_voltage_timestamp = fc->data.battery_voltage_timestamp;

		packet_size = write_battery_voltage_packet(
				packet_bytes,
				fc->data.battery_voltage_timestamp,
				fc->data.battery_1_voltage,
				fc->data.battery_2_voltage,
				fc->data.battery_3_voltage);
		fc_telemetry_send_packet(packet_bytes, packet_size);
	}

	if (latest_magnetometer_timestamp < fc->data.magnetometer_timestamp) {
		latest_magnetometer_timestamp = fc->data.magnetometer_timestamp;

		packet_size = write_magnetometer_packet(
				packet_bytes,
				fc->data.magnetometer_timestamp,
				fc->data.magnetometer_x,
				fc->data.magnetometer_y,
				fc->data.magnetometer_z);
		fc_telemetry_send_packet(packet_bytes, packet_size);
	}

	if (latest_gyroscope_timestamp < fc->data.gyroscope_timestamp) {
		latest_gyroscope_timestamp = fc->data.gyroscope_timestamp;

		packet_size = write_magnetometer_packet(
				packet_bytes,
				fc->data.gyroscope_timestamp,
				fc->data.gyroscope_x,
				fc->data.gyroscope_y,
				fc->data.gyroscope_z);
		fc_telemetry_send_packet(packet_bytes, packet_size);
	}

}
