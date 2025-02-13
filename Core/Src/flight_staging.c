/**
 * flight_staging.c
 * @author Ashton South
 *
 * Functions that track and update rocket state to deploy the parachute.
 */


/* TODO: Update these values */
#define BUFFER_SIZE 10
#define FLIGHT_THRESHOLD 3.0
#define AIRBRAKES_THRESHOLD -1.0
#define DESCENT_HIGH_THRESHOLD 0.0
#define GROUND_THRESHOLD 1.0
#define DESCENT_LOW_ALTITUDE 5000.0

typedef enum {
    GROUND,
    FLIGHT,
    AIRBRAKES,
    DESCENT_HIGH,
    DESCENT_LOW
} Flight_State;

typedef struct direction_data {
    float buf[BUFFER_SIZE];
    int idx;
    float avg;
} direction_data;

static float calculate_average(float *buffer);
static void update_average(direction_data *data, float new_datapoint);

static Flight_State FLIGHT_STATE = GROUND;

/**
 * This is called every sensor read, so we have a running average used to determine the
 * state of the rocket.
 */
void update_flight_state(float low_g_accel_x, float low_g_accel_y, float low_g_accel_z, float altitude) {
    /* TODO: Consider not storing x and y data */
    static direction_data x_data = { {0.0}, 0, 0.0 };
    static direction_data y_data = { {0.0}, 0, 0.0 };
    static direction_data z_data = { {0.0}, 0, 0.0 };

    update_average(&x_data, low_g_accel_x);
    update_average(&y_data, low_g_accel_y);
    update_average(&z_data, low_g_accel_z);

    /* Update FLIGHT_STATE depending on our averaged data and previous state. */
    if(z_data.avg > FLIGHT_THRESHOLD && FLIGHT_STATE == GROUND) {
        FLIGHT_STATE = FLIGHT;
    } else if(z_data.avg < AIRBRAKES_THRESHOLD && FLIGHT_STATE == FLIGHT) {
        FLIGHT_STATE = AIRBRAKES;
    } else if(z_data.avg > DESCENT_HIGH_THRESHOLD && FLIGHT_STATE == AIRBRAKES) {
        FLIGHT_STATE = DESCENT_HIGH;
        /* TODO: Deploy drogue parachute here */
    } else if(altitude < DESCENT_LOW_ALTITUDE && FLIGHT_STATE == DESCENT_HIGH) {
        FLIGHT_STATE = DESCENT_LOW;
        /* TODO: Deploy main parachute here */
    } else if(z_data.avg < GROUND_THRESHOLD && FLIGHT_STATE == DESCENT_LOW) {
        FLIGHT_STATE = GROUND;
    }

    /* TODO: Possibly log state? */
}

static float calculate_average(float *buffer) {
    float sum = 0.0;
    for(int i = 0; i < BUFFER_SIZE; i++) {
        sum += buffer[i];
    }
    return sum / BUFFER_SIZE;
}

static void update_average(direction_data *data, float new_datapoint) {
    data->buf[data->idx] = new_datapoint;
    data->idx++;
    if(data->idx == BUFFER_SIZE) data->idx = 0;
    data->avg = calculate_average(data->buf);
}