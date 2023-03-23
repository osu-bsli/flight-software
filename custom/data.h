static struct {
	float timestamp;
	float altitude;
} barometer_data_point;

static struct {
	float timestamp;
	float x;
	float y;
	float z;
} accelerometer_1_data_point;

static struct {
	float timestamp;
	float x;
	float y;
	float z;
} accelerometer_2_data_point;

static struct {
	float timestamp;
	float x;
	float y;
	float z;
} magnetometer_data_point;

static struct {
	float timestamp;
	float latitude;
	float longitude;
	int satellite_count; // unsure if needed
	float ground_speed; // unsure if needed
} gps_data_point;
