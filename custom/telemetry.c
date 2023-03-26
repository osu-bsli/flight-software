#include "telemetry.h"
#include "data.h"
#include "fc.h"
#include "arming.h"
#include "packet-parser/parser.h"
#include "packet-parser/packet.h"
#include "main.h"
#include <stdint.h>

/*
 * altitude data buffer
 */
#define FC_PACKET_BUFFER_MAX_COUNT 10 // store up to 10 data points
typedef struct FCPacketBuffer {
	int size;
	uint8_t bytes[FC_PACKET_BUFFER_MAX_COUNT];
} FCPacketBuffer;

void fc_packet_buffer_initialize(FCPacketBuffer *buffer) {
	buffer->size = 0;
}

void fc_packet_buffer_dequeue(FCPacketBuffer *buffer) {
	if (buffer->size <= 0) {
		return;
	}

	int idx;
	for (idx = 1; idx < buffer->size; idx++) {
		buffer->bytes[idx - 1] = buffer->bytes[idx];
	}
	buffer->size -= 1;
}

void fc_packet_buffer_enqueue(FCPacketBuffer *buffer, uint8_t byte) {
	// if the buffer has too many items, dequeue
	if (buffer->size + 1 > FC_PACKET_BUFFER_MAX_COUNT) {
		altitude_buffer_dequeue(buffer);
	}

	buffer->bytes[buffer->size] = byte;
	buffer->size += 1;
}

void fc_telemetry_send_packet(uint8_t *packet_bytes, int packet_size) {
	/* TODO: Use DMA mode to prevent blocking */
//	HAL_UART_Transmit(&huart1, packet_bytes, packet_size, 100);

	/* TODO: Write to SD card */
//	HAL_SPI_Transmit(&hspi3, packet_bytes, packet_size, 100);
}

void fc_telemetry_init(FlightComputer *fc) {
	packetlib_initialize();
}

/* avoiding sending repeat data by storing its timestamp each time */
float latest_magnetometer_data_timestamp = 0.0f;

void fc_telemetry_process(FlightComputer *fc) {
	/* parse arming packets */
	packetlib_process();
	Packet *packet = packetlib_get_packet();
	if (packet->is_ready) {
		switch (packet->type) {
		case PACKET_TYPE_ARM_PRIMARY_FC:
			fc_arming_arm_primary_fc();
			break;
		case PACKET_TYPE_DISARM_PRIMARY_FC:
			fc_arming_disarm_primary_fc();
			break;
		case PACKET_TYPE_ARM_SECONDARY_FC:
			fc_arming_arm_secondary_fc();
			break;
		case PACKET_TYPE_DISARM_SECONDARY_FC:
			fc_arming_disarm_secondary_fc();
			break;
		case PACKET_TYPE_ARM_CAMERA:
			fc_arming_arm_camera();
			break;
		case PACKET_TYPE_DISARM_CAMERA:
			fc_arming_disarm_camera();
			break;
		default:
			break;
		}
	}

	/* byte buffer to hold written packets before being sent */
	uint8_t packet_bytes[64];
	int packet_size = 0;

	/* TODO: Make packet sending atomic */

	/* write arming status packets every time? */
	packet_size = write_arming_status_packet(
			packet_bytes,
			((float) HAL_GetTick()) / 1000.0f,
			fc_arming_is_accelerometer_1_armed(),
			fc_arming_is_accelerometer_2_armed(),
			fc_arming_is_barometer_armed());
	fc_telemetry_send_packet(packet_bytes, packet_size);

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
}
