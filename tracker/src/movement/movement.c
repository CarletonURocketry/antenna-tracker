#include <pthread.h>
#include "movement.h"
#include <uORB/uORB.h>
#include <poll.h>
#include <string.h>
#include "../syslogging.h"

typedef struct {
    struct sensor_angle tilt_angle;
    struct sensor_angle pan_angle;
} movement_input_angles_t;

union uorb_sensor_buff_t {
    struct sensor_angle tilt_angle;
    struct sensor_angle pan_angle;
};

enum {
    TILT_ANGLE,
    PAN_ANGLE
};

void* movement_main(void* args){
    int err;

    struct pollfd uorb_fds[] = {
        [TILT_ANGLE] = {.fd = -1, .events = POLLIN, .revents = 0},
        [PAN_ANGLE] = {.fd = -1, .events = POLLIN, .revents = 0},
    };

    uorb_fds[TILT_ANGLE].fd = orb_subscribe_multi(orb_get_meta("sensor_hinge_angle"), 0);
    uorb_fds[PAN_ANGLE].fd = orb_subscribe_multi(orb_get_meta("sensor_hinge_angle"), 1);

    movement_input_angles_t movement_input_angles;
    memset(&movement_input_angles, 0, sizeof(movement_input_angles));

    for(;;){
        err = poll(uorb_fds, sizeof(uorb_fds) / sizeof(uorb_fds[0]), 1000);
        if(err < 0){
            inerr("Error polling uORB data: %s\n", strerror(err));
            continue;
        }

        for(int i = 0; i < sizeof(uorb_fds) / sizeof(uorb_fds[0]); i++){
            /* skip if data not available of fd is invalid */
            if (uorb_fds[i].fd < 0 || !(uorb_fds[i].revents & POLLIN)) {
                continue;
            }

            union uorb_sensor_buff_t uorb_sensor_buff;
            err = orb_copy_multi(uorb_fds[i].fd, &uorb_sensor_buff, sizeof(uorb_sensor_buff));
            if(err < 0){
                inerr("Error copying uORB data: %s\n", strerror(err));
                continue;
            }

            switch(i){
                case PAN_ANGLE:
                    movement_input_angles.pan_angle = uorb_sensor_buff.pan_angle;
                    break;
                case TILT_ANGLE:
                    movement_input_angles.tilt_angle = uorb_sensor_buff.tilt_angle;
                    break;
            }
        }

        ininfo("Tilt angle: %f, Pan angle: %f\n", movement_input_angles.tilt_angle.angle, movement_input_angles.pan_angle.angle);
    }

    pthread_exit(NULL);
}