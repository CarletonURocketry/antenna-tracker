#include <pthread.h>
#include "aiming.h"
#include <uORB/uORB.h>
#include <poll.h>
#include <string.h>
#include <time.h>
#include "../syslogging.h"
#include "utm/utm.h"
#include "kinematics.h"

ORB_DECLARE(sensor_gnss);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_baro);
ORB_DECLARE(sensor_hinge_angle);

void aim_tracker(aiming_input_telem_t *aiming_input_telem, aiming_output_angles_t *aiming_output_angles, size_t size_aiming_input){
    /* TODO: figure out aiming algoritm */
    // ASSUME DATA IS NON-ZERO AND WORKING

    size_t last_index = size_aiming_input - 1;

    UTMCoord index0_utm = latlon_to_utm(aiming_input_telem[0]->rocket_gnss.latitude, aiming_input_telem[0]->rocket_gnss.longitude);
    UTMCoord pos = latlon_to_utm(aiming_input_telem[last_index]->rocket_gnss.latitude, aiming_input_telem[last_index]->rocket_gnss.longitude);
    double alt = aiming_input_telem[last_index]->rocket_gnss.altitude;

    // Create functions given positions, gives velocity and acceleration!!!

    integrated_pos_t vel = get_avg_vel(aiming_input_telem, size_aiming_input);
    integrated_pos_t accel = get_avg_accel(aiming_input_telem, size_aiming_input);




    float time_s = 1 / ITERATIONS; // One second divided by iterations per second

    for (int i =0; i < ITERATIONS; i++) {
        double predicted_easting = const_accel_eq(time_s * i, vel.easting[0], accel.easting[0], pos.easting);
        double predicted_northing = const_accel_eq(time_s * i, vel.northing[0], accel.northing[0], pos.northing);
        double predicted_alt = const_accel_eq(time_s * i, vel.altitude[0], accel.altitude[0], alt);

        // Do something with predicted positions
    }

    /* generate random angles to publish for now */
    aiming_output_angles->pan_angle.angle = rand() % 360;
    aiming_output_angles->tilt_angle.angle = rand() % 360;
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