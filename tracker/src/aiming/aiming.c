#include <pthread.h>
#include "aiming.h"
#include <uORB/uORB.h>
#include <poll.h>
#include <string.h>
#include <time.h>
#include "../syslogging.h"

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

void aim_tracker(aiming_input_telem_t *aiming_input_telem, aiming_output_angles_t *aiming_output_angles){
    /* TODO: figure out aiming algoritm */

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

    uorb_fds_in[TRACKER_GNSS].fd = orb_subscribe_multi(orb_get_meta("sensor_gnss"), 0);
    uorb_fds_in[TRACKER_MAG].fd = orb_subscribe_multi(orb_get_meta("sensor_mag"), 0);
    uorb_fds_in[TRACKER_BARO].fd = orb_subscribe_multi(orb_get_meta("sensor_baro"), 0);
    uorb_fds_in[ROCKET_GNSS].fd = orb_subscribe_multi(orb_get_meta("sensor_gnss"), 1);

    struct pollfd uorb_fds_out[] = {
        [PAN_ANGLE] = {.fd = -1, .events = POLLIN, .revents = 0},
        [TILT_ANGLE] = {.fd = -1, .events = POLLIN, .revents = 0},
    };

    int tilt_instance = 0;
    int pan_instance = 1;
    uorb_fds_out[TILT_ANGLE].fd = orb_advertise_multi_queue(orb_get_meta("sensor_hinge_angle"), NULL, &tilt_instance, 1);
    uorb_fds_out[PAN_ANGLE].fd = orb_advertise_multi_queue(orb_get_meta("sensor_hinge_angle"), NULL, &pan_instance, 1);

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