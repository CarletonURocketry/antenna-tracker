#include "aiming.h"
#include "fusion.c"
// #ifdef CONFIG_LIB_MADGWICK
// #include "Fusion/Fusion.h"
// #endif
#include <pthread.h>
// #ifdef CONFIG_UORB
#include <uORB/uORB.h>
// #endif
#include "../syslogging.h"
#include "kinematics.h"
#include "utm.h"
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <time.h>

ORB_DECLARE(sensor_gnss);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_accel);
ORB_DECLARE(sensor_gyro);
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
                 aiming_output_angles_t *aiming_output_angles) {
    float last_rocket_alt = aiming_input_telem->rocket_alt[aiming_input_telem->rocket_alt_n - 1].altitude;

    pos_vec_t avg_vel = {0, 0, 0};
    pos_vec_t avg_accel = {0, 0, 0};
    utm_coord_t last_rocket_pos;

    latlon_to_utm(aiming_input_telem->rocket_gnss[aiming_input_telem->rocket_gnss_n - 1].latitude,
                  aiming_input_telem->rocket_gnss[aiming_input_telem->rocket_gnss_n - 1].longitude, &last_rocket_pos);

    utm_coord_t last_tracker_pos;
    latlon_to_utm(aiming_input_telem->tracker_gnss.latitude, aiming_input_telem->tracker_gnss.longitude,
                  &last_tracker_pos);

    float predicted_x = const_accel_eq(time_offset_ms, avg_vel.x, avg_accel.x, last_rocket_pos.x);
    float predicted_y = const_accel_eq(time_offset_ms, avg_vel.y, avg_accel.y, last_rocket_pos.y);
    float predicted_z = const_accel_eq(time_offset_ms, avg_vel.z, avg_accel.z, last_rocket_alt);

    // Change in pos between rocket pos and tracker pos
    float delta_x = predicted_x - last_tracker_pos.x;
    float delta_y = predicted_y - last_tracker_pos.y;
    float delta_z = predicted_z - aiming_input_telem->tracker_alt.altitude;

    // Convert to degrees and correct for magnetic heading
    float bearing_deg = atan2f(delta_x, delta_y) * (180.0f / M_PI);
    float pan_deg = bearing_deg - aiming_input_telem->mag_heading_deg;
    while (pan_deg > 180.0f)
        pan_deg -= 360.0f;
    while (pan_deg < -180.0f)
        pan_deg += 360.0f;
    aiming_output_angles->pan_angle.angle = pan_deg;

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
        [TRACKER_ACCEL] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TRACKER_GYRO] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TRACKER_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
    };

    struct orb_metadata const *uorb_metas_in[] = {
        [TRACKER_GNSS] = ORB_ID(sensor_gnss),   [TRACKER_MAG] = ORB_ID(sensor_mag),
        [TRACKER_ACCEL] = ORB_ID(sensor_accel), [TRACKER_GYRO] = ORB_ID(sensor_gyro),
        [TRACKER_BARO] = ORB_ID(sensor_baro),   [ROCKET_GNSS] = ORB_ID(sensor_gnss),
        [ROCKET_ALT] = ORB_ID(sensor_alt),      [ROCKET_BARO] = ORB_ID(sensor_baro),
    };

    for (int i = 0; i < sizeof(uorb_metas_in) / sizeof(uorb_metas_in[0]); i++) {
        if (uorb_metas_in[i] == NULL) {
            inerr("Error getting uORB metadata: %s\n", strerror(errno));
            pthread_exit(NULL);
        }
    }

    uorb_fds_in[TRACKER_GNSS].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_GNSS], 0);
    uorb_fds_in[TRACKER_MAG].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_MAG], 0);
    uorb_fds_in[TRACKER_ACCEL].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_ACCEL], 0);
    uorb_fds_in[TRACKER_GYRO].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_GYRO], 0);
    uorb_fds_in[TRACKER_BARO].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_BARO], 0);
    uorb_fds_in[ROCKET_GNSS].fd = orb_subscribe_multi(uorb_metas_in[ROCKET_GNSS], 1);
    uorb_fds_in[ROCKET_ALT].fd = orb_subscribe_multi(uorb_metas_in[ROCKET_ALT], 1);
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

    // #ifdef CONFIG_LIB_MADGWICK
    //     FusionAhrs ahrs;
    //     FusionAhrsInitialise(&ahrs);
    //     const FusionAhrsSettings ahrs_settings = {
    //         .convention = FusionConventionNwu,
    //         .gain = 0.5f,
    //         .gyroscopeRange = 2000.0f, /* FSR set to +-2000 dps in imu_configure() */
    //         .accelerationRejection = 10.0f,
    //         .magneticRejection = 10.0f,
    //         .recoveryTriggerPeriod = 63, /* 5s * 12.5 Hz default ODR */
    //     };
    //     FusionAhrsSetSettings(&ahrs, &ahrs_settings);
    //     uint64_t last_gyro_timestamp = 0;
    // #endif

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
                case TRACKER_MAG: {
                    aiming_input_telem.tracker_mag = uorb_sensor_buff[j].tracker_mag;
                    aiming_input_telem.tracker_mag_n = 1;

                    float heading;
                    if (mag_to_heading(&aiming_input_telem.tracker_mag, &calib, &heading) == 0) {
                        aiming_input_telem.mag_heading_deg = heading;
                        aiming_input_telem.mag_heading_n = 1;
                        ininfo("Tracker heading: %.2f deg\n", heading);
                    }
                    break;
                }
                case TRACKER_ACCEL: {
                    aiming_input_telem.tracker_accel = uorb_sensor_buff[j].tracker_accel;
                    aiming_input_telem.tracker_accel_n = 1;
                    break;
                }
                case TRACKER_GYRO: {
                    aiming_input_telem.tracker_gyro = uorb_sensor_buff[j].tracker_gyro;
                    aiming_input_telem.tracker_gyro_n = 1;
                    break;
                }
                case TRACKER_BARO: {
                    struct sensor_alt tracker_alt;
                    calculate_altitude(&uorb_sensor_buff[j].tracker_baro, &tracker_alt);
                    aiming_input_telem.tracker_alt = tracker_alt;
                    aiming_input_telem.tracker_alt_n = 1;
                    break;
                }
                case ROCKET_ALT: {
                    if (aiming_input_telem.rocket_alt_n >= TELEM_SAMPLE_N) break;
                    struct sensor_alt rocket_alt = uorb_sensor_buff[j].rocket_alt;
                    aiming_input_telem.rocket_alt[aiming_input_telem.rocket_alt_n++] = rocket_alt;
                    break;
                }
                case ROCKET_GNSS: {
                    if (aiming_input_telem.rocket_gnss_n >= TELEM_SAMPLE_N) break;
                    struct sensor_gnss rocket_gnss = uorb_sensor_buff[j].rocket_gnss;
                    aiming_input_telem.rocket_gnss[aiming_input_telem.rocket_gnss_n++] = rocket_gnss;
                    break;
                }
                /* Temporary for fakesensor */
                case ROCKET_BARO: {
                    if (aiming_input_telem.rocket_alt_n >= TELEM_SAMPLE_N) break;
                    struct sensor_alt rocket_alt_baro;
                    calculate_altitude(&uorb_sensor_buff[j].rocket_baro, &rocket_alt_baro);
                    aiming_input_telem.rocket_alt[aiming_input_telem.rocket_alt_n++] = rocket_alt_baro;
                    break;
                }
                }

                /* point north */
                // if (aiming_input_telem.mag_heading_n == 1) {
                //     struct sensor_angle pan_angle = {.angle = aiming_input_telem.mag_heading_deg};
                //     orb_publish_multi(uorb_fds_out[PAN_ANGLE].fd, &pan_angle, sizeof(pan_angle));
                //     aiming_input_telem.mag_heading_n = 0;
                // }

                // #ifdef CONFIG_LIB_MADGWICK
                //                 /* https://github.com/xioTechnologies/Fusion */
                //                 if (i == TRACKER_GYRO && aiming_input_telem.tracker_accel_n > 0 &&
                //                     aiming_input_telem.tracker_mag_n > 0) {
                //                     uint64_t current_ts = aiming_input_telem.tracker_gyro.timestamp;

                //                     if (last_gyro_timestamp != 0) {
                //                         uint64_t delta_us = current_ts - last_gyro_timestamp;
                //                         float delta_time = (float)delta_us / 1000000.0f;

                //                         /* Invert Y to align IMU axes with magnetometer */
                //                         FusionVector gyroscope = {.axis = {
                //                                                       .x = aiming_input_telem.tracker_gyro.x *
                //                                                       (180.0f / M_PI), .y =
                //                                                       -aiming_input_telem.tracker_gyro.y * (180.0f /
                //                                                       M_PI), .z = aiming_input_telem.tracker_gyro.z *
                //                                                       (180.0f / M_PI),
                //                                                   }};
                //                         FusionVector accelerometer = {.axis = {
                //                                                           .x = aiming_input_telem.tracker_accel.x,
                //                                                           .y = -aiming_input_telem.tracker_accel.y,
                //                                                           .z = aiming_input_telem.tracker_accel.z,
                //                                                       }};
                //                         FusionVector magnetometer = {.axis = {
                //                                                          .x = aiming_input_telem.tracker_mag.x,
                //                                                          .y = aiming_input_telem.tracker_mag.y,
                //                                                          .z = aiming_input_telem.tracker_mag.z,
                //                                                      }};

                //                         FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta_time);

                //                         /* yaw from quaternion: atan2(x*y + w*z, w*w + x*x - 0.5) */
                //                         FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
                //                         float heading = atan2f(q.element.x * q.element.y + q.element.w * q.element.z,
                //                                                q.element.w * q.element.w + q.element.x * q.element.x
                //                                                - 0.5f) *
                //                                         (180.0f / M_PI);
                //                         if (heading < 0.0f) heading += 360.0f;
                //                         heading += CONFIG_INSPACE_MAG_DECLINATION;
                //                         while (heading >= 360.0f)
                //                             heading -= 360.0f;

                //                         float mag_heading_raw;
                //                         if (mag_to_heading(&aiming_input_telem.tracker_mag, &calib, &mag_heading_raw)
                //                         == 0) {
                //                             ininfo("Tracker heading (AHRS): %.2f deg, (mag raw): %.2f deg\n",
                //                             (double)heading,
                //                                    (double)mag_heading_raw);
                //                         } else {
                //                             ininfo("Tracker heading (AHRS): %.2f deg\n", (double)heading);
                //                         }
                //                         aiming_input_telem.mag_heading_deg = heading;
                //                         aiming_input_telem.mag_heading_n = 1;
                //                     }
                //                     last_gyro_timestamp = current_ts;
                //                 }
                // #endif /* CONFIG_LIB_MADGWICK */
            }
        }
    }

    pthread_exit(NULL);
}
