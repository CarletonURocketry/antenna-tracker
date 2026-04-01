#include "movement.h"
#include "../syslogging.h"
#include "motor.h"
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <string.h>
#include <time.h>
#include <uORB/uORB.h>

#define SERVO_SPEED_DEG_PER_MS 0.07f
#define SERVO_WRAP_BUFFER_DEG 10.0f /* buffer to prevent the servo from wrapping around the min and max angle */
#define PAN_SERVO_HOME_ANGLE                                                                                           \
    (CONFIG_INSPACE_TRACKER_MIN_ANGLE + (CONFIG_INSPACE_TRACKER_MAX_ANGLE - CONFIG_INSPACE_TRACKER_MIN_ANGLE) / 2)
#define SERVO_DEADBAND_DEG 1.0f /* deadband to prevent the servo from oscillating*/

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

    /* servo movement check */
    move_angle(0, 0);
    move_angle(0, 1);
    sleep(2);

    float target_pan = 0.0f;
    float target_tilt = 0.0f;
    float current_pan = 0.0f;
    float current_tilt = 0.0f;

    struct timespec last_step_time;
    clock_gettime(CLOCK_MONOTONIC, &last_step_time);

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
                target_pan = uorb_sensor_buff.pan_angle.angle + PAN_SERVO_HOME_ANGLE;
                break;

            case TILT_ANGLE:
                target_tilt =
                    uorb_sensor_buff.tilt_angle.angle * 3.0f; /* the scaling is needed because we have a 3:1 gear ratio,
                                                                 tilt gets scaled down to having a 0-90 degree range*/
                break;
            }
        }

        /* correct both angles to be the closest to the middle of the range */
        if (target_tilt - PAN_SERVO_HOME_ANGLE > 180.0f + SERVO_WRAP_BUFFER_DEG) target_tilt -= 360.0f;
        if (target_tilt - PAN_SERVO_HOME_ANGLE < -(180.0f + SERVO_WRAP_BUFFER_DEG)) target_tilt += 360.0f;
        if (target_pan - PAN_SERVO_HOME_ANGLE > 180.0f + SERVO_WRAP_BUFFER_DEG) target_pan -= 360.0f;
        if (target_pan - PAN_SERVO_HOME_ANGLE < -(180.0f + SERVO_WRAP_BUFFER_DEG)) target_pan += 360.0f;

        /* clip to servo range */
        if (target_tilt < CONFIG_INSPACE_TRACKER_MIN_ANGLE) target_tilt = CONFIG_INSPACE_TRACKER_MIN_ANGLE;
        if (target_tilt > CONFIG_INSPACE_TRACKER_MAX_ANGLE) target_tilt = CONFIG_INSPACE_TRACKER_MAX_ANGLE;
        if (target_pan < CONFIG_INSPACE_TRACKER_MIN_ANGLE) target_pan = CONFIG_INSPACE_TRACKER_MIN_ANGLE;
        if (target_pan > CONFIG_INSPACE_TRACKER_MAX_ANGLE) target_pan = CONFIG_INSPACE_TRACKER_MAX_ANGLE;

        /* compute max step */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        float elapsed_ms = (float)(now.tv_sec - last_step_time.tv_sec) * 1000.0f +
                           (float)(now.tv_nsec - last_step_time.tv_nsec) / 1.0e6f;
        last_step_time = now;
        float max_step = elapsed_ms * SERVO_SPEED_DEG_PER_MS;

        /* move servos towards target angle */
        float tilt_error = target_tilt - current_tilt;
        if (fabsf(tilt_error) > SERVO_DEADBAND_DEG) {
            float tilt_step = tilt_error > 0.0f ? fminf(tilt_error, max_step) : fmaxf(tilt_error, -max_step);
            current_tilt += tilt_step;
            err = move_angle((int)roundf(270.0 - current_tilt), 1); // angle inverted because servo inverted
            if (err < 0) {
                inerr("Failed to move tilt motor to angle %.1f: %s\n", current_tilt, strerror(err));
                pthread_exit(NULL);
            }
        }

        float pan_error = target_pan - current_pan;
        if (fabsf(pan_error) > SERVO_DEADBAND_DEG) {
            float pan_step = pan_error > 0.0f ? fminf(pan_error, max_step) : fmaxf(pan_error, -max_step);
            current_pan += pan_step;
            err = move_angle((int)roundf(270.0 - current_pan), 0); // angle inverted because servo inverted
            if (err < 0) {
                inerr("Failed to move pan motor to angle %.1f: %s\n", current_pan, strerror(err));
                pthread_exit(NULL);
            }
        }
    }

    pthread_exit(NULL);
}