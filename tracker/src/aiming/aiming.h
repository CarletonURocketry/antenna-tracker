#include <uORB/uORB.h>

#ifndef _INSPACE_TRACKER_AIMING_H_
#define _INSPACE_TRACKER_AIMING_H_

/* Rocket sample delta time in ms */
#define ROCKET_SAMPLE_DT_MS 50
#define ITERATIONS 10

void* aiming_main(void* args);

typedef struct {
    struct sensor_gnss tracker_gnss;
    struct sensor_mag tracker_mag;
    struct sensor_baro tracker_baro;
    struct sensor_gnss rocket_gnss;
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
};

enum {
    TRACKER_GNSS,
    TRACKER_MAG,
    TRACKER_BARO,
    ROCKET_GNSS
};

enum {
    TILT_ANGLE,
    PAN_ANGLE
};

#endif