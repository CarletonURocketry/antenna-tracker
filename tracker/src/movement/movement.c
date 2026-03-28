#include "movement.h"
#include "../syslogging.h"
#include "motor.h"
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <uORB/uORB.h>

#define SERVO_MAX_STEP_DEG 3.0f
#define SERVO_WRAP_BUFFER_DEG 10.0f
#define SERVO_HOME_ANGLE                                                                                               \
    (CONFIG_INSPACE_TRACKER_MIN_ANGLE + (CONFIG_INSPACE_TRACKER_MAX_ANGLE - CONFIG_INSPACE_TRACKER_MIN_ANGLE) / 2)

typedef struct {
    struct sensor_angle tilt_angle;
    struct sensor_angle pan_angle;
} movement_input_angles_t;

union uorb_sensor_buff_t {
    struct sensor_angle tilt_angle;
    struct sensor_angle pan_angle;
};

enum { TILT_ANGLE, PAN_ANGLE };

ORB_DECLARE(sensor_hinge_angle);

void *movement_main(void *args) {
    int err;

    struct pollfd uorb_fds[] = {
        [TILT_ANGLE] = {.fd = -1, .events = POLLIN, .revents = 0},
        [PAN_ANGLE] = {.fd = -1, .events = POLLIN, .revents = 0},
    };

    struct orb_metadata const *uorb_metas[] = {
        [TILT_ANGLE] = ORB_ID(sensor_hinge_angle),
        [PAN_ANGLE] = ORB_ID(sensor_hinge_angle),
    };

    for (int i = 0; i < sizeof(uorb_metas) / sizeof(uorb_metas[0]); i++) {
        if (uorb_metas[i] == NULL) {
            inerr("Error getting uORB metadata: %s\n", strerror(errno));
            pthread_exit(NULL);
        }
    }

    err = pwm_state_setup();
    if (err < 0) {
        inerr("Failed to set up PWM state\n");
        pthread_exit(NULL);
    }
    ininfo("pwm_state_setup complete\n");

    uorb_fds[PAN_ANGLE].fd = orb_subscribe_multi(uorb_metas[PAN_ANGLE], 0);
    uorb_fds[TILT_ANGLE].fd = orb_subscribe_multi(uorb_metas[TILT_ANGLE], 1);

    for (int i = 0; i < sizeof(uorb_fds) / sizeof(uorb_fds[0]); i++) {
        if (uorb_fds[i].fd < 0) {
            inerr("Error subscribing to uORB %s: %s\n", uorb_metas[i]->o_name, strerror(errno));
            pthread_exit(NULL);
        }
    }

    /* homing the servos*/
    move_angle(SERVO_HOME_ANGLE, 0);
    move_angle(SERVO_HOME_ANGLE, 1);
    /* wait for the servos to home, hardcoded since I don't have feedback, unsure on how to do this better */
    sleep(2);

    float target_pan = SERVO_HOME_ANGLE;
    float target_tilt = SERVO_HOME_ANGLE;
    float current_pan = SERVO_HOME_ANGLE;
    float current_tilt = SERVO_HOME_ANGLE;

    for (;;) {
        err = poll(uorb_fds, sizeof(uorb_fds) / sizeof(uorb_fds[0]), 10);
        if (err < 0) {
            inerr("Error polling uORB data: %s\n", strerror(err));
            continue;
        }

        for (int i = 0; i < sizeof(uorb_fds) / sizeof(uorb_fds[0]); i++) {
            /* skip if data not available of fd is invalid */
            if (uorb_fds[i].fd < 0 || !(uorb_fds[i].revents & POLLIN)) {
                continue;
            }

            union uorb_sensor_buff_t uorb_sensor_buff;
            err = orb_copy_multi(uorb_fds[i].fd, &uorb_sensor_buff, sizeof(uorb_sensor_buff));
            if (err < 0) {
                inerr("Error copying uORB data: %s\n", strerror(err));
                continue;
            }

            switch (i) {
            case PAN_ANGLE:
                target_pan = uorb_sensor_buff.pan_angle.angle;
                break;
            case TILT_ANGLE:
                target_tilt = uorb_sensor_buff.tilt_angle.angle;
                break;
            }
        }

        /* correct both angles to be the closest to the middle of the range */
        if (target_tilt - SERVO_HOME_ANGLE > 180.0f + SERVO_WRAP_BUFFER_DEG) target_tilt -= 360.0f;
        if (target_tilt - SERVO_HOME_ANGLE < -(180.0f + SERVO_WRAP_BUFFER_DEG)) target_tilt += 360.0f;
        if (target_pan - SERVO_HOME_ANGLE > 180.0f + SERVO_WRAP_BUFFER_DEG) target_pan -= 360.0f;
        if (target_pan - SERVO_HOME_ANGLE < -(180.0f + SERVO_WRAP_BUFFER_DEG)) target_pan += 360.0f;

        /* clip to servo range */
        if (target_tilt < CONFIG_INSPACE_TRACKER_MIN_ANGLE) target_tilt = CONFIG_INSPACE_TRACKER_MIN_ANGLE;
        if (target_tilt > CONFIG_INSPACE_TRACKER_MAX_ANGLE) target_tilt = CONFIG_INSPACE_TRACKER_MAX_ANGLE;
        if (target_pan < CONFIG_INSPACE_TRACKER_MIN_ANGLE) target_pan = CONFIG_INSPACE_TRACKER_MIN_ANGLE;
        if (target_pan > CONFIG_INSPACE_TRACKER_MAX_ANGLE) target_pan = CONFIG_INSPACE_TRACKER_MAX_ANGLE;

        /* move servos towards target angle */
        if (target_tilt != current_tilt) {
            float tilt_step = target_tilt - current_tilt;

            /* clip to max step */
            if (tilt_step > SERVO_MAX_STEP_DEG) tilt_step = SERVO_MAX_STEP_DEG;
            if (tilt_step < -SERVO_MAX_STEP_DEG) tilt_step = -SERVO_MAX_STEP_DEG;
            current_tilt += tilt_step;
            err = move_angle((int)current_tilt, 1);
            if (err < 0) {
                inerr("Failed to move tilt motor to angle %.1f: %s\n", current_tilt, strerror(err));
                pthread_exit(NULL);
            }

            ininfo("Tilt current %.1f, target %.1f\n", current_tilt, target_tilt);
        }

        if (target_pan != current_pan) {
            float pan_step = target_pan - current_pan;
            if (pan_step > SERVO_MAX_STEP_DEG) pan_step = SERVO_MAX_STEP_DEG;
            if (pan_step < -SERVO_MAX_STEP_DEG) pan_step = -SERVO_MAX_STEP_DEG;
            current_pan += pan_step;
            err = move_angle((int)current_pan, 0);
            if (err < 0) {
                inerr("Failed to move pan motor to angle %.1f: %s\n", current_pan, strerror(err));
                pthread_exit(NULL);
            }

            ininfo("Pan current %.1f, target %.1f\n", current_pan, target_pan);
        }
    }

    pthread_exit(NULL);
}