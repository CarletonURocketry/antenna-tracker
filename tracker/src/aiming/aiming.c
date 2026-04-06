#include "aiming.h"
#include "../syslogging.h"
#include "fusion.c"
#include "kinematics.h"
#include "utm.h"
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <time.h>
#include <uORB/uORB.h>

ORB_DECLARE(sensor_gnss);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_baro);
ORB_DECLARE(sensor_alt);
ORB_DECLARE(sensor_hinge_angle);

int mag_to_heading(struct sensor_mag *sensor_mag, const mag_calib_t *calib, float *heading) {
    float x_cal = sensor_mag->x - calib->hard_iron_x;
    float y_cal = sensor_mag->y - calib->hard_iron_y;

    float x_norm = x_cal / calib->soft_radius_x;
    float y_norm = y_cal / calib->soft_radius_y;

    float heading_rad = atan2f(x_norm, y_norm);
    float heading_deg = heading_rad * 180.0f / M_PI;

    heading_deg += CONFIG_INSPACE_MAG_DECLINATION;

    if (heading_deg < 0.0f) heading_deg += 360.0f;
    if (heading_deg >= 360.0f) heading_deg -= 360.0f;

    *heading = heading_deg;
    return 0;
}

void aim_tracker(aiming_input_telem_t *aiming_input_telem, uint16_t time_offset_ms,
                 aiming_output_angles_t *aiming_output_angles, float angle_correction) {
    float last_rocket_alt = aiming_input_telem->rocket_alt[aiming_input_telem->rocket_alt_n - 1].altitude;

    // pos_vec_t avg_vel = {0, 0, 0};
    // pos_vec_t avg_accel = {0, 0, 0};
    utm_coord_t last_rocket_pos;

    latlon_to_utm(aiming_input_telem->rocket_gnss[aiming_input_telem->rocket_gnss_n - 1].latitude,
                  aiming_input_telem->rocket_gnss[aiming_input_telem->rocket_gnss_n - 1].longitude, &last_rocket_pos);

    utm_coord_t tracker_coord_utm;
    latlon_to_utm(aiming_input_telem->tracker_gnss.latitude, aiming_input_telem->tracker_gnss.longitude,
                  &tracker_coord_utm);

    // float predicted_x = const_accel_eq(time_offset_ms, avg_vel.x, avg_accel.x, last_rocket_pos.x);
    // float predicted_y = const_accel_eq(time_offset_ms, avg_vel.y, avg_accel.y, last_rocket_pos.y);
    // float predicted_z = const_accel_eq(time_offset_ms, avg_vel.z, avg_accel.z, last_rocket_alt);

    // Change in pos between rocket pos and tracker pos
    // float delta_x = predicted_x - last_tracker_pos.x;
    // float delta_y = predicted_y - last_tracker_pos.y;
    // float delta_z = predicted_z - aiming_input_telem->tracker_alt.altitude;

    float delta_x = last_rocket_pos.x - tracker_coord_utm.x;
    float delta_y = last_rocket_pos.y - tracker_coord_utm.y;
    float delta_z = last_rocket_alt - aiming_input_telem->tracker_alt.altitude;

    float bearing_deg = atan2f(delta_x, delta_y) * (180.0f / M_PI);
    if (bearing_deg < 0.0f) bearing_deg += 360.0f;
    aiming_output_angles->pan_angle.angle = bearing_deg + angle_correction;

    float horizontal_distance = sqrt(delta_x * delta_x + delta_y * delta_y);

    /* We are using -90 to 90 degree angles (The datum, aka 0, will be the rocket). In pwm control, remember to add 90
     * degrees to compensate for this! */
    aiming_output_angles->tilt_angle.angle = atan2(delta_z, horizontal_distance) * (180.0 / M_PI);
}

void *aiming_main(void *args) {
    int err;

    mag_calib_t calib = *(mag_calib_t *)args;

    struct pollfd uorb_fds_in[] = {
        [TRACKER_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TRACKER_MAG] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TRACKER_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
    };

    struct orb_metadata const *uorb_metas_in[] = {
        [TRACKER_GNSS] = ORB_ID(sensor_gnss),   [TRACKER_MAG] = ORB_ID(sensor_mag),
        [TRACKER_BARO] = ORB_ID(sensor_baro),   [ROCKET_GNSS] = ORB_ID(sensor_gnss),
        [ROCKET_ALT] = ORB_ID(sensor_altitude), [ROCKET_BARO] = ORB_ID(sensor_baro),
    };

    for (int i = 0; i < sizeof(uorb_metas_in) / sizeof(uorb_metas_in[0]); i++) {
        if (uorb_metas_in[i] == NULL) {
            inerr("Error getting uORB metadata: %s\n", strerror(errno));
            pthread_exit(NULL);
        }
    }

    uorb_fds_in[TRACKER_GNSS].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_GNSS], 0);
    uorb_fds_in[TRACKER_MAG].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_MAG], 0);
    uorb_fds_in[TRACKER_BARO].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_BARO], 0);
    uorb_fds_in[ROCKET_GNSS].fd = orb_subscribe_multi(uorb_metas_in[ROCKET_GNSS], 1);
    uorb_fds_in[ROCKET_ALT].fd = orb_subscribe_multi(uorb_metas_in[ROCKET_ALT], 0);
    uorb_fds_in[ROCKET_BARO].fd = orb_subscribe_multi(uorb_metas_in[ROCKET_BARO], 1);

    for (int i = 0; i < sizeof(uorb_fds_in) / sizeof(uorb_fds_in[0]); i++) {
        if (uorb_fds_in[i].fd < 0) {
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

    for (int i = 0; i < sizeof(uorb_metas_out) / sizeof(uorb_metas_out[0]); i++) {
        if (uorb_metas_out[i] == NULL) {
            inerr("Error getting uORB metadata: %s\n", strerror(errno));
            pthread_exit(NULL);
        }
    }

    int pan_instance = 0;
    int tilt_instance = 1;
    uorb_fds_out[PAN_ANGLE].fd = orb_advertise_multi_queue(uorb_metas_out[PAN_ANGLE], NULL, &pan_instance, 1);
    uorb_fds_out[TILT_ANGLE].fd = orb_advertise_multi_queue(uorb_metas_out[TILT_ANGLE], NULL, &tilt_instance, 1);

    for (int i = 0; i < sizeof(uorb_fds_out) / sizeof(uorb_fds_out[0]); i++) {
        if (uorb_fds_out[i].fd < 0) {
            inerr("Error advertising to uORB %s: %s\n", uorb_metas_out[i]->o_name, strerror(errno));
            pthread_exit(NULL);
        }
    }

    aiming_input_telem_t aiming_input_telem;
    memset(&aiming_input_telem, 0, sizeof(aiming_input_telem));

    union uorb_sensor_buff_t uorb_sensor_buff[TELEM_SAMPLE_N];

    for (;;) {
        err = poll(uorb_fds_in, sizeof(uorb_fds_in) / sizeof(uorb_fds_in[0]), -1);
        if (err < 0) {
            inerr("Error polling uORB data: %s\n", strerror(err));
            continue;
        }

        for (int i = 0; i < sizeof(uorb_fds_in) / sizeof(uorb_fds_in[0]); i++) {
            /* skip if data not available of fd is invalid */
            if (uorb_fds_in[i].fd < 0 || !(uorb_fds_in[i].revents & POLLIN)) {
                continue;
            }

            err = orb_copy_multi(uorb_fds_in[i].fd, uorb_sensor_buff, sizeof(uorb_sensor_buff));
            if (err < 0) {
                inerr("Error copying uORB data: %s\n", strerror(err));
                continue;
            }

            for (int j = 0; j < (err / uorb_metas_in[i]->o_size); j++) {
                switch (i) {
                case TRACKER_GNSS: {
                    aiming_input_telem.tracker_gnss = uorb_sensor_buff[j].tracker_gnss;
                    aiming_input_telem.tracker_gnss_n = 1;
                    break;
                }
                case TRACKER_BARO: {
                    struct sensor_alt tracker_alt;
                    calculate_altitude(&uorb_sensor_buff[j].tracker_baro, &tracker_alt);
                    aiming_input_telem.tracker_alt = tracker_alt;
                    aiming_input_telem.tracker_alt_n = 1;
                    break;
                }
                case TRACKER_MAG: {
                    aiming_input_telem.tracker_mag = uorb_sensor_buff[j].tracker_mag;
                    aiming_input_telem.tracker_mag_n = 1;
                    break;
                }
                case ROCKET_ALT: {
                    struct sensor_altitude rocket_alt = uorb_sensor_buff[j].rocket_alt;
                    aiming_input_telem.rocket_alt[aiming_input_telem.rocket_alt_n] = rocket_alt;
                    aiming_input_telem.rocket_alt_n = 1;
                    break;
                }
                case ROCKET_GNSS: {
                    struct sensor_gnss rocket_gnss = uorb_sensor_buff[j].rocket_gnss;
                    aiming_input_telem.rocket_gnss[aiming_input_telem.rocket_gnss_n] = rocket_gnss;
                    aiming_input_telem.rocket_gnss_n = 1;
                    break;
                    break;
                }
                /* Temporary for fakesensor */
                case ROCKET_BARO: {
                    if (aiming_input_telem.rocket_alt_n >= TELEM_SAMPLE_N) break;
                    struct sensor_alt rocket_alt_baro;
                    calculate_altitude(&uorb_sensor_buff[j].rocket_baro, &rocket_alt_baro);
                    aiming_input_telem.rocket_alt[aiming_input_telem.rocket_alt_n++] = (struct sensor_altitude){
                        .timestamp = rocket_alt_baro.timestamp, .altitude = rocket_alt_baro.altitude};
                    break;
                }
                }

                if (aiming_input_telem.rocket_gnss_n > 0 && aiming_input_telem.rocket_alt_n > 0 &&
                    aiming_input_telem.tracker_gnss_n > 0 && aiming_input_telem.tracker_alt_n > 0) {

                    ininfo("Tracker GNSS: %f, %f\n", aiming_input_telem.tracker_gnss.latitude,
                           aiming_input_telem.tracker_gnss.longitude);
                    ininfo("Tracker Altitude: %f\n", aiming_input_telem.tracker_alt.altitude);
                    ininfo("Rocket GNSS: %f, %f\n",
                           aiming_input_telem.rocket_gnss[aiming_input_telem.rocket_gnss_n - 1].latitude,
                           aiming_input_telem.rocket_gnss[aiming_input_telem.rocket_gnss_n - 1].longitude);
                    ininfo("Rocket Altitude: %f\n",
                           aiming_input_telem.rocket_alt[aiming_input_telem.rocket_alt_n - 1].altitude);

                    /* Launch Canada basics GSE coords */
                    // 47°56'41.53 "N 81°51'05.58" W
                    // aiming_input_telem.tracker_gnss.latitude = 47.944869f;
                    // aiming_input_telem.tracker_gnss.longitude = -81.851550f;
                    // aiming_input_telem.tracker_gnss_n = 1;

                    // aiming_input_telem.tracker_alt.altitude = 363.0f;
                    // aiming_input_telem.tracker_alt_n = 1;

                    aiming_output_angles_t out;
                    aim_tracker(&aiming_input_telem, 0, &out, calib.angle_correction);

                    struct sensor_angle pan_angle = {.angle = out.pan_angle.angle};
                    orb_publish_multi(uorb_fds_out[PAN_ANGLE].fd, &pan_angle, sizeof(pan_angle));

                    struct sensor_angle tilt_angle = {.angle = out.tilt_angle.angle};
                    orb_publish_multi(uorb_fds_out[TILT_ANGLE].fd, &tilt_angle, sizeof(tilt_angle));
                }
            }
        }
    }

    pthread_exit(NULL);
}
