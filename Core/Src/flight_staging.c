/*
 * flight_staging.c
 *
 * Functions that track and update the state of the rocket.
 */

#define BUFFER_SIZE 10
#define FLIGHT_THRESHOLD 3.0

static float calculate_average(float *buffer);

typedef enum {
    GROUND,
    FLIGHT,
    DESCENT,
    AIRBRAKES,
    PARACHUTE,
} Flight_State;

static Flight_State FLIGHT_STATE = GROUND;

/**
 * This is called every sensor read, so we have a running average used to determine the
 * state of the rocket.
 */
void update_flight_state(float low_g_accel_x, float low_g_accel_y, float low_g_accel_z) {
    static float x_buf[BUFFER_SIZE] = {0.0};
    static float y_buf[BUFFER_SIZE] = {0.0};
    static float z_buf[BUFFER_SIZE] = {0.0};

    static int x_idx = 0;
    static int y_idx = 0;
    static int z_idx = 0;

    x_buf[x_idx] = low_g_accel_x;
    x_idx++;
    y_buf[y_idx] = low_g_accel_y;
    y_idx++;
    z_buf[z_idx] = low_g_accel_z;
    z_idx++;

    if(x_idx == BUFFER_SIZE) x_idx = 0;
    if(y_idx == BUFFER_SIZE) y_idx = 0;
    if(z_idx == BUFFER_SIZE) z_idx = 0;

    float avg_x = calculate_average(x_buf);
    float avg_y = calculate_average(y_buf);
    float avg_z = calculate_average(z_buf);

    if(avg_z > FLIGHT_THRESHOLD && FLIGHT_STATE == GROUND) {
        FLIGHT_STATE = FLIGHT;
    } else if(avg_z < FLIGHT_THRESHOLD && FLIGHT_STATE == FLIGHT) {
        FLIGHT_STATE = DESCENT;
    }
}

static float calculate_average(float *buffer) {
    float sum = 0.0;
    for(int i = 0; i < BUFFER_SIZE; i++) {
        sum += buffer[i];
    }
    return sum / BUFFER_SIZE;
}