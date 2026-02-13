#include <pthread.h>
#include "aiming.h"
#include <uORB/uORB.h>
#include <poll.h>
#include <string.h>
#include <time.h>
#include "../syslogging.h"
#include <math.h>

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

ORB_DECLARE(sensor_gnss);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_baro);
ORB_DECLARE(sensor_hinge_angle);

int mag_to_heading(struct sensor_mag *sensor_mag, float *heading)
{
    if (sensor_mag == NULL || heading == NULL) {
        inerr("Invalid sensor_mag or heading pointer\n");
        return -1;
    }

    float x_cal = sensor_mag->x - CONFIG_INSPACE_MAG_CALIB_X;
    float y_cal = sensor_mag->y - CONFIG_INSPACE_MAG_CALIB_Y;

    float heading_rad = atan2f(x_cal, y_cal);
    float heading_deg = heading_rad * 180.0f / M_PI;

    heading_deg += CONFIG_INSPACE_MAG_DECLINATION;

    while (heading_deg < 0.0f) {
        heading_deg += 360.0f;
    }
    while (heading_deg >= 360.0f) {
        heading_deg -= 360.0f;
    }

    *heading = heading_deg;
    return 0;
}

void aim_tracker(aiming_input_telem_t *aiming_input_telem, aiming_output_angles_t *aiming_output_angles){
    if (aiming_input_telem == NULL || aiming_output_angles == NULL) {
        inerr("Invalid input/output pointers in aim_tracker\n");
        aiming_output_angles->pan_angle.angle = 0.0f;
        aiming_output_angles->tilt_angle.angle = 0.0f;
        return;
    }

    float current_heading;
    int ret = mag_to_heading(&aiming_input_telem->tracker_mag, &current_heading);
    if (ret != 0) {
        inerr("Failed to compute heading from magnetometer\n");
        aiming_output_angles->pan_angle.angle = 0.0f;
        aiming_output_angles->tilt_angle.angle = 0.0f;
        return;
    }

    const float METERS_PER_DEG_LAT = 111320.0f;
    const float METERS_PER_DEG_LON_AT_EQUATOR = 111320.0f;

    float lat_avg = (aiming_input_telem->tracker_gnss.latitude + aiming_input_telem->rocket_gnss.latitude) / 2.0f;
    float meters_per_deg_lon = METERS_PER_DEG_LON_AT_EQUATOR * cosf(lat_avg * M_PI / 180.0f);

    float enu_e = (aiming_input_telem->rocket_gnss.longitude - aiming_input_telem->tracker_gnss.longitude) * meters_per_deg_lon;
    float enu_n = (aiming_input_telem->rocket_gnss.latitude - aiming_input_telem->tracker_gnss.latitude) * METERS_PER_DEG_LAT;
    float enu_u = aiming_input_telem->rocket_gnss.altitude - aiming_input_telem->tracker_gnss.altitude;

    float az_rad = atan2f(enu_e, enu_n);
    float az_deg = az_rad * 180.0f / M_PI;
    
    while (az_deg < 0.0f) {
        az_deg += 360.0f;
    }
    while (az_deg >= 360.0f) {
        az_deg -= 360.0f;
    }

    float horiz_dist = sqrtf(enu_e*enu_e + enu_n*enu_n);
    float elevation;
    if (horiz_dist < 1e-6f) {
        elevation = (enu_u >= 0.0f) ? 90.0f : -90.0f;
    } else {
        float el_rad = atanf(enu_u / horiz_dist);
        elevation = el_rad * 180.0f / M_PI;
    }

    float pan_error = az_deg - current_heading;
    while (pan_error > 180.0f) {
        pan_error -= 360.0f;
    }
    while (pan_error <= -180.0f) {
        pan_error += 360.0f;
    }

    aiming_output_angles->pan_angle.angle = pan_error;
    aiming_output_angles->tilt_angle.angle = elevation;

    ininfo("Aiming: current_hdg=%.1f°, desired_az=%.1f°, pan_err=%.1f°, el=%.1f°\n",
           current_heading, az_deg, pan_error, elevation);
}

void* aiming_main(void* args){
    int err;

    struct pollfd uorb_fds_in[] = {
        [TRACKER_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TRACKER_MAG] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TRACKER_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
    };

    struct orb_metadata const *uorb_metas_in[] = {
        [TRACKER_GNSS] = ORB_ID(sensor_gnss),
        [TRACKER_MAG] = ORB_ID(sensor_mag),
        [TRACKER_BARO] = ORB_ID(sensor_baro),
        [ROCKET_GNSS] = ORB_ID(sensor_gnss),
    };

    for(int i = 0; i < sizeof(uorb_metas_in) / sizeof(uorb_metas_in[0]); i++){
        if(uorb_metas_in[i] == NULL){
            inerr("Error getting uORB metadata: %s\n", strerror(errno));
            pthread_exit(NULL);
        }
    }

    uorb_fds_in[TRACKER_GNSS].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_GNSS], 0);
    uorb_fds_in[TRACKER_MAG].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_MAG], 0);
    uorb_fds_in[TRACKER_BARO].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_BARO], 0);
    uorb_fds_in[ROCKET_GNSS].fd = orb_subscribe_multi(uorb_metas_in[ROCKET_GNSS], 1);

    for(int i = 0; i < sizeof(uorb_fds_in) / sizeof(uorb_fds_in[0]); i++){
        if(uorb_fds_in[i].fd < 0){
            inerr("Error subscribing to uORB %s: %s\n", uorb_metas_in[i]->o_name, strerror(errno));
            pthread_exit(NULL);
        }
    }

    struct pollfd uorb_fds_out[] = {
        [PAN_ANGLE] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TILT_ANGLE] = {.fd = -1, .events = POLLIN, .revents = 0},
    };

    struct orb_metadata const *uorb_metas_out[] = {
        [PAN_ANGLE] = ORB_ID(sensor_hinge_angle),
        [TILT_ANGLE] = ORB_ID(sensor_hinge_angle),
    };

    for(int i = 0; i < sizeof(uorb_metas_out) / sizeof(uorb_metas_out[0]); i++){
        if(uorb_metas_out[i] == NULL){
            inerr("Error getting uORB metadata: %s\n", strerror(errno));
            pthread_exit(NULL);
        }
    }

    int pan_instance = 0;
    int tilt_instance = 1;
    uorb_fds_out[PAN_ANGLE].fd = orb_advertise_multi_queue(uorb_metas_out[PAN_ANGLE], NULL, &pan_instance, 1);
    uorb_fds_out[TILT_ANGLE].fd = orb_advertise_multi_queue(uorb_metas_out[TILT_ANGLE], NULL, &tilt_instance, 1);

    for(int i = 0; i < sizeof(uorb_fds_out) / sizeof(uorb_fds_out[0]); i++){
        if(uorb_fds_out[i].fd < 0){
            inerr("Error advertising to uORB %s: %s\n", uorb_metas_out[i]->o_name, strerror(errno));
            pthread_exit(NULL);
        }
    }

    aiming_input_telem_t aiming_input_telem;
    memset(&aiming_input_telem, 0, sizeof(aiming_input_telem));

    for(;;){
        aiming_output_angles_t aiming_output_angles;
        aim_tracker(&aiming_input_telem, &aiming_output_angles);

        orb_publish_multi(uorb_fds_out[PAN_ANGLE].fd, &aiming_output_angles.pan_angle, sizeof(aiming_output_angles.pan_angle));
        orb_publish_multi(uorb_fds_out[TILT_ANGLE].fd, &aiming_output_angles.tilt_angle, sizeof(aiming_output_angles.tilt_angle));

        err = poll(uorb_fds_in, sizeof(uorb_fds_in) / sizeof(uorb_fds_in[0]), 1000);
        if(err < 0){
            inerr("Error polling uORB data: %s\n", strerror(err));
            continue;
        }

        for(int i = 0; i < sizeof(uorb_fds_in) / sizeof(uorb_fds_in[0]); i++){
            /* skip if data not available of fd is invalid */
            if (uorb_fds_in[i].fd < 0 || !(uorb_fds_in[i].revents & POLLIN)) {
                continue;
            }

            union uorb_sensor_buff_t uorb_sensor_buff;
            err = orb_copy_multi(uorb_fds_in[i].fd, &uorb_sensor_buff, sizeof(uorb_sensor_buff));
            if(err < 0){
                inerr("Error copying uORB data: %s\n", strerror(err));
                continue;
            }

            switch(i){
                case TRACKER_GNSS:
                    aiming_input_telem.tracker_gnss = uorb_sensor_buff.tracker_gnss;
                    ininfo("Tracker GNSS: %f, %f, %f\n", aiming_input_telem.tracker_gnss.latitude, aiming_input_telem.tracker_gnss.longitude, aiming_input_telem.tracker_gnss.altitude);
                    break;
                case TRACKER_MAG:
                    aiming_input_telem.tracker_mag = uorb_sensor_buff.tracker_mag;
                    ininfo("Tracker MAG: %f, %f, %f\n", aiming_input_telem.tracker_mag.x, aiming_input_telem.tracker_mag.y, aiming_input_telem.tracker_mag.z);
                    break;
                case TRACKER_BARO:
                    aiming_input_telem.tracker_baro = uorb_sensor_buff.tracker_baro;
                    ininfo("Tracker BARO: %f, %f\n", aiming_input_telem.tracker_baro.pressure, aiming_input_telem.tracker_baro.temperature);
                    break;
                case ROCKET_GNSS:
                    aiming_input_telem.rocket_gnss = uorb_sensor_buff.rocket_gnss;
                    ininfo("Rocket GNSS: %f, %f, %f\n", aiming_input_telem.rocket_gnss.latitude, aiming_input_telem.rocket_gnss.longitude, aiming_input_telem.rocket_gnss.altitude);
                    break;
            }
        }

    }

    pthread_exit(NULL);
}
