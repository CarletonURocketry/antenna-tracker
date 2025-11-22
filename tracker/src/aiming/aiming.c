#include <pthread.h>
#include "aiming.h"
#include "fusion.c"
// #ifdef CONFIG_UORB
#include <uORB/uORB.h>
// #endif
#include <poll.h>
#include <string.h>
#include <time.h>
#include "../syslogging.h"
#include "utm.h"
#include "kinematics.h"

ORB_DECLARE(sensor_gnss);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_baro);
ORB_DECLARE(sensor_alt);
ORB_DECLARE(sensor_hinge_angle);

void aim_tracker(aiming_input_telem_t *aiming_input_telem, uint16_t time_offset_ms, aiming_output_angles_t *aiming_output_angles){    
    float last_rocket_alt = aiming_input_telem->rocket_alt[aiming_input_telem->rocket_alt_n - 1].altitude;
    
    pos_vec_t avg_vel = {0, 0, 0};
    pos_vec_t avg_accel = {0, 0, 0};
    utm_coord_t last_rocket_pos;

    latlon_to_utm(aiming_input_telem->rocket_gnss[aiming_input_telem->rocket_gnss_n - 1].latitude, aiming_input_telem->rocket_gnss[aiming_input_telem->rocket_gnss_n - 1].longitude, &last_rocket_pos);
    
    utm_coord_t last_tracker_pos;
    latlon_to_utm(aiming_input_telem->tracker_gnss.latitude, aiming_input_telem->tracker_gnss.longitude, &last_tracker_pos);

    float predicted_x = const_accel_eq(time_offset_ms, avg_vel.x, avg_accel.x, last_rocket_pos.x);
    float predicted_y = const_accel_eq(time_offset_ms, avg_vel.y, avg_accel.y, last_rocket_pos.y);
    float predicted_z = const_accel_eq(time_offset_ms, avg_vel.z, avg_accel.z, last_rocket_alt);

    // Change in pos between rocket pos and tracker pos
    float delta_x = predicted_x - last_tracker_pos.x;
    float delta_y = predicted_y - last_tracker_pos.y;
    float delta_z = predicted_z - aiming_input_telem->tracker_alt.altitude;


    // Convert to spherical coordinates (azimuth and elevation angles)
    aiming_output_angles->pan_angle.angle = atan2(delta_y, delta_x) * (180.0 / M_PI);

    float horizontal_distance = sqrt(delta_x * delta_x + delta_y * delta_y);
    aiming_output_angles->tilt_angle.angle = atan2(delta_z, horizontal_distance) * (180.0 / M_PI);
}

void* aiming_main(void* args){
    int err;

    struct pollfd uorb_fds_in[] = {
        [TRACKER_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TRACKER_MAG] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TRACKER_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
    };

    struct orb_metadata const *uorb_metas_in[] = {
        [TRACKER_GNSS] = ORB_ID(sensor_gnss),
        [TRACKER_MAG] = ORB_ID(sensor_mag),
        [TRACKER_BARO] = ORB_ID(sensor_baro),
        [ROCKET_GNSS] = ORB_ID(sensor_gnss),
        [ROCKET_ALT] = ORB_ID(sensor_alt),
        [ROCKET_BARO] = ORB_ID(sensor_baro),
    };

    for(int i = 0; i < sizeof(uorb_metas_in) / sizeof(uorb_metas_in[0]); i++){
        if(uorb_metas_in[i] == NULL){
            inerr("Error getting uORB metadata: %s\n", strerror(errno));
            pthread_exit(NULL);
        }
    }

    uorb_fds_in[TRACKER_GNSS].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_GNSS], 1);
    uorb_fds_in[TRACKER_MAG].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_MAG], 0);
    uorb_fds_in[TRACKER_BARO].fd = orb_subscribe_multi(uorb_metas_in[TRACKER_BARO], 1);
    uorb_fds_in[ROCKET_GNSS].fd = orb_subscribe_multi(uorb_metas_in[ROCKET_GNSS], 0);
    uorb_fds_in[ROCKET_ALT].fd = orb_subscribe_multi(uorb_metas_in[ROCKET_ALT], 0);
    uorb_fds_in[ROCKET_BARO].fd = orb_subscribe_multi(uorb_metas_in[ROCKET_BARO], 0);

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
    
    union uorb_sensor_buff_t uorb_sensor_buff[TELEM_SAMPLE_N];

    for(;;){
        err = poll(uorb_fds_in, sizeof(uorb_fds_in) / sizeof(uorb_fds_in[0]), -1);
        if(err < 0){
            inerr("Error polling uORB data: %s\n", strerror(err));
            continue;
        }

        for(int i = 0; i < sizeof(uorb_fds_in) / sizeof(uorb_fds_in[0]); i++){
            /* skip if data not available of fd is invalid */
            if (uorb_fds_in[i].fd < 0 || !(uorb_fds_in[i].revents & POLLIN)) {
                continue;
            }

            err = orb_copy_multi(uorb_fds_in[i].fd, uorb_sensor_buff, sizeof(uorb_sensor_buff));       
            if(err < 0){
                inerr("Error copying uORB data: %s\n", strerror(err));
                continue;
            }

            for (int j = 0; j < (err / uorb_metas_in[i]->o_size); j++) {
               switch(i){
                case TRACKER_GNSS:
                {
                    aiming_input_telem.tracker_gnss = uorb_sensor_buff[j].tracker_gnss;
                    aiming_input_telem.tracker_gnss_n++;
                    // ininfo("Tracker GNSS: %d, %f, %f\n", aiming_input_telem.tracker_gnss.timestamp, aiming_input_telem.tracker_gnss.latitude, aiming_input_telem.tracker_gnss.longitude);
                    break;
                }
                case TRACKER_MAG:
                {
                    aiming_input_telem.tracker_mag = uorb_sensor_buff[j].tracker_mag;
                    // ininfo("Tracker MAG: %d, %f, %f, %f\n", aiming_input_telem.tracker_mag.timestamp, aiming_input_telem.tracker_mag.x, aiming_input_telem.tracker_mag.y, aiming_input_telem.tracker_mag.z);
                    break;
                }
                case TRACKER_BARO:
                {
                    struct sensor_alt tracker_alt;
                    calculate_altitude(&uorb_sensor_buff[j].tracker_baro, &tracker_alt);
                    aiming_input_telem.tracker_alt = tracker_alt;
                    aiming_input_telem.tracker_alt_n++;
                    // ininfo("Tracker ALT: %d, %f\n", aiming_input_telem.tracker_alt.timestamp, aiming_input_telem.tracker_alt.altitude);
                    break;
                }
                case ROCKET_ALT:
                {
                    if (aiming_input_telem.rocket_alt_n >= TELEM_SAMPLE_N) break;
                    struct sensor_alt rocket_alt = uorb_sensor_buff[j].rocket_alt;
                    aiming_input_telem.rocket_alt[aiming_input_telem.rocket_alt_n++] = rocket_alt;
                    // ininfo("Rocket ALT: %d, %f\n", rocket_alt.timestamp, rocket_alt.altitude);
                    break;
                }
                case ROCKET_GNSS:
                {
                    if (aiming_input_telem.rocket_gnss_n >= TELEM_SAMPLE_N) break;
                    struct sensor_gnss rocket_gnss = uorb_sensor_buff[j].rocket_gnss;
                    aiming_input_telem.rocket_gnss[aiming_input_telem.rocket_gnss_n++] = rocket_gnss;
                    // ininfo("Rocket GNSS:%d, %f, %f\n",rocket_gnss.timestamp, rocket_gnss.latitude, rocket_gnss.longitude);
                    break;
                }
                /* Temporary for fakesensor */
                case ROCKET_BARO:
                {
                    if (aiming_input_telem.rocket_alt_n >= TELEM_SAMPLE_N) break;
                    struct sensor_alt rocket_alt_baro;
                    calculate_altitude(&uorb_sensor_buff[j].rocket_baro, &rocket_alt_baro);
                    aiming_input_telem.rocket_alt[aiming_input_telem.rocket_alt_n++] = rocket_alt_baro;
                    // ininfo("Rocket ALT: %d, %f\n", rocket_alt_baro.timestamp, rocket_alt_baro.altitude);
                    break;
                }
                } 

            
                if (aiming_input_telem.rocket_gnss_n == TELEM_SAMPLE_N && aiming_input_telem.rocket_alt_n == TELEM_SAMPLE_N) {
                    aiming_output_angles_t aiming_output_angles;

                    /* TODO: remove this to get actual tracker data */
                    struct sensor_alt tracker_alt = {.timestamp=0, .altitude=568.922241};
                    struct sensor_gnss tracker_gnss ={.timestamp=0, .latitude=47.990833, .longitude=-81.851111}; 
                    aiming_input_telem.tracker_alt = tracker_alt;
                    aiming_input_telem.tracker_gnss= tracker_gnss;

                    aim_tracker(&aiming_input_telem, 50, &aiming_output_angles);

                    // orb_publish_multi(uorb_fds_out[PAN_ANGLE].fd, &aiming_output_angles.pan_angle, sizeof(aiming_output_angles.pan_angle));
                    // orb_publish_multi(uorb_fds_out[TILT_ANGLE].fd, &aiming_output_angles.tilt_angle, sizeof(aiming_output_angles.tilt_angle));

                    ininfo("PAN ANGLE %f\n", aiming_output_angles.pan_angle.angle);
                    ininfo("TILT ANGLE %f\n", aiming_output_angles.tilt_angle.angle);

                    aiming_input_telem.rocket_gnss_n = 0;
                    aiming_input_telem.tracker_gnss_n = 0;
                    aiming_input_telem.rocket_alt_n = 0;
                    aiming_input_telem.tracker_alt_n = 0;
                }
            }

            
        }

    }

    pthread_exit(NULL);
}