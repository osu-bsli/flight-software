#include "recovery.h"

/*
 * altitude data buffer
 */
#define ALTITUDE_BUFFER_MAX_COUNT 10 // store up to 10 data points
#define ALTITUDE_BUFFER_MAX_TIME 5.0 // store up to 5 seconds of data
typedef struct AltitudeBuffer {
	int size;
	float altitudes[ALTITUDE_BUFFER_MAX_COUNT];
	float timestamps[ALTITUDE_BUFFER_MAX_COUNT]; // parallel with altitudes array
} AltitudeBuffer;

void altitude_buffer_initialize(AltitudeBuffer *buffer) {
	buffer->size = 0;
}

void altitude_buffer_dequeue(AltitudeBuffer *buffer) {
	if (buffer->size <= 0) {
		return;
	}

	int idx;
	for (idx = 1; idx < buffer->size; idx++) {
		buffer->altitudes[idx - 1] = buffer->altitudes[idx];
		buffer->timestamps[idx - 1] = buffer->timestamps[idx];
	}
	buffer->size -= 1;
}

void altitude_buffer_enqueue(AltitudeBuffer *buffer, float altitude, float timestamp) {

	// if the buffer has too many items, dequeue
	if (buffer->size + 1 > ALTITUDE_BUFFER_MAX_COUNT) {
		altitude_buffer_dequeue(buffer);
	}

	// dequeue any data points that are too old
	int idx;
	int dequeue_count = 0;
	for (idx = 0; idx < buffer->size; idx++) {
		if (timestamp - buffer->timestamps[idx] > ALTITUDE_BUFFER_MAX_TIME) {
			dequeue_count++;
		} else {
			break;
		}
	}
	int i;
	for (i = 0; i < dequeue_count; i++) {
		altitude_buffer_dequeue(buffer);
	}

	buffer->altitudes[buffer->size] = altitude;
	buffer->timestamps[buffer->size] = timestamp;
	buffer->size += 1;
}

void fc_recovery_process(void) {

}
