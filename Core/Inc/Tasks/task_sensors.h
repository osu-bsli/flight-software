/**
 * task_sensors.h
 *
 * @author Brian Jia
 */

#include <stdint.h>

void task_sensors_start(void);

#define TELEMETRY_PACKET_MAX_LEN 255
#define TELEMETRY_PACKET_MAGIC "FUCKPETER"

enum StatusFlags {
    STATUS_FLAGS_RECOVERY_ARMED = 1 << 0,
    STATUS_FLAGS_EMATCH_DROGUE_DEPLOYED = 1 << 1,
    STATUS_FLAGS_EMATCH_MAIN_DEPLOYED = 1 << 2,
    STATUS_FLAGS_SD_CARD_WORKING = 1 << 3,
};


struct __attribute__((packed)) telemetry_packet {
    char magic[9]; // 'FUCKPETER' in ASCII with no null terminator
    uint8_t size; // Total size of struct
    uint16_t crc16;
    uint8_t status_flags; // StatusFlags bitfield
    uint32_t time_boot_ms; // Timestamp (ms since system boot)
    float ms5607_pressure_mbar; // MS5607 Air Pressure (unit: mbar)
    float ms5607_temperature_c; // MS5607 Temperature (unit: degrees C)
    float bmi323_accel_x; // BMI323 Acceleration X (unit: G)
    float bmi323_accel_y; // BMI323 Acceleration Y (unit: G)
    float bmi323_accel_z; // BMI323 Acceleration Z (unit: G)
    float bmi323_gyro_x; // BMI323 Gyroscope X (unit: deg/s)
    float bmi323_gyro_y; // BMI323 Gyroscope Y (unit: deg/s)
    float bmi323_gyro_z; // BMI323 Gyroscope Z (unit: deg/s)
    float adxl375_accel_x; // ADXL375 Acceleration X (unit: G)
    float adxl375_accel_y; // ADXL375 Acceleration Y (unit: G)
    float adxl375_accel_z; // ADXL375 Acceleration Z (unit: G)
};