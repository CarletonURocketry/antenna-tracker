#include <uORB/uORB.h>

#ifndef _INSPACE_TRACKER_AIMING_H_
#define _INSPACE_TRACKER_AIMING_H_

/* Rocket sample delta time in ms */
#define ROCKET_SAMPLE_DT_MS 50
#define ITERATIONS 10

#define TELEM_SAMPLE_N 10

void* aiming_main(void* args);

/* UORB declarations for fused sensor data */
ORB_DECLARE(sensor_alt);

/*
 * Size of the internal queues for the fusioned data, which other threads should
 * match the size of their buffers to
 */
#define ALT_SENSOR_BUFFER 5

/* A fusioned altitude sample */
struct sensor_alt {
    uint64_t timestamp; /* Timestamp in microseconds */
    float altitude;     /* Altitude in meters */
};

typedef struct {
    struct sensor_gnss tracker_gnss;
    int tracker_gnss_n;
    struct sensor_alt tracker_alt;
    int tracker_alt_n;
    struct sensor_mag tracker_mag;
    int tracker_mag_n;
    struct sensor_alt rocket_alt[TELEM_SAMPLE_N];
    int rocket_alt_n;
    struct sensor_gnss rocket_gnss[TELEM_SAMPLE_N];
    int rocket_gnss_n;
} aiming_input_telem_t;

typedef struct {
    struct sensor_angle pan_angle;
    struct sensor_angle tilt_angle;
} aiming_output_angles_t;

union uorb_sensor_buff_t {
    struct sensor_gnss tracker_gnss;
    struct sensor_mag tracker_mag;
    struct sensor_baro tracker_baro;
    struct sensor_gnss rocket_gnss;
    struct sensor_alt rocket_alt;
    struct sensor_baro rocket_baro; /* This is temporary for fakesensor */
};

enum uorb_sensors_in {
    TRACKER_GNSS,
    TRACKER_MAG,
    TRACKER_BARO,
    ROCKET_GNSS,
    ROCKET_ALT,
    ROCKET_BARO
};

enum uorb_sensors_out {
    TILT_ANGLE,
    PAN_ANGLE
};

int calculate_altitude(struct sensor_baro *baro_data, struct sensor_alt *altitude);

#endif