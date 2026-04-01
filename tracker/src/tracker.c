#include "tracker.h"
#include "aiming/aiming.h"
#include "collection/collection.h"
#include "movement/motor.h"
#include "movement/movement.h"
#include <errno.h>
#include <float.h>
#include <math.h>
#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <uORB/uORB.h>

ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_gyro);
ORB_DECLARE(sensor_hinge_angle);

static int imu_configure(void) {
    int fd = orb_subscribe(ORB_ID(sensor_gyro));
    if (fd < 0) {
        printf("Failed to subscribe to sensor_gyro: %s\n", strerror(errno));
        return -errno;
    }

    int err = orb_ioctl(fd, SNIOC_SETFULLSCALE, 2000);
    if (err < 0) {
        printf("Failed to set gyro FSR to 2000 dps: %s\n", strerror(errno));
    }

    orb_unsubscribe(fd);
    return err;
}

static int run_mag_calibration(mag_calib_t *calib) {
    int mag_fd = orb_subscribe(ORB_ID(sensor_mag));
    if (mag_fd < 0) {
        inerr("Calibration: failed to subscribe to sensor_mag: %s\n", strerror(errno));
        return -1;
    }

    int pan_servo_instance = 0;
    int pan_fd = orb_advertise_multi_queue(ORB_ID(sensor_hinge_angle), NULL, &pan_servo_instance, 1);
    if (pan_fd < 0) {
        inerr("Calibration: failed to advertise hinge_angle: %s\n", strerror(errno));
        orb_unsubscribe(mag_fd);
        return -1;
    }

    struct pollfd mag_poll = {.fd = mag_fd, .events = POLLIN, .revents = 0};

    float min_x = FLT_MAX;
    float max_x = -FLT_MAX;
    float min_y = FLT_MAX;
    float max_y = -FLT_MAX;

    struct sensor_angle pan_cmd;
    pan_cmd.angle = 135.0f;
    orb_publish_multi(pan_fd, &pan_cmd, sizeof(pan_cmd));

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        int err = poll(&mag_poll, 1, 0);
        if (err < 0) {
            inerr("Error polling uORB: %s\n", strerror(errno));
            continue;
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (now.tv_sec - start.tv_sec > 5) {
            break;
        }

        if (mag_poll.revents & POLLIN) {
            struct sensor_mag mag_data;
            err = orb_copy(ORB_ID(sensor_mag), mag_fd, &mag_data);
            if (err < 0) {
                inerr("Error copying uORB data: %s\n", strerror(errno));
                continue;
            }

            if (mag_data.x < min_x) min_x = mag_data.x;
            if (mag_data.x > max_x) max_x = mag_data.x;
            if (mag_data.y < min_y) min_y = mag_data.y;
            if (mag_data.y > max_y) max_y = mag_data.y;
        }
    }

    // struct sensor_angle pan_cmd;
    // for (int pass = 0; pass < 1; pass++) {
    //     for (float angle = CONFIG_INSPACE_TRACKER_MIN_ANGLE - 135.0f;
    //          angle <= CONFIG_INSPACE_TRACKER_MAX_ANGLE - 135.0f; angle += 1.0f) {
    //         pan_cmd.angle = angle;
    //         orb_publish_multi(pan_fd, &pan_cmd, sizeof(pan_cmd));

    //         usleep(1000 * 10);

    //         int err = poll(&mag_poll, 1, 0);
    //         if (err < 0) {
    //             inerr("Error polling uORB: %s\n", strerror(errno));
    //             continue;
    //         }

    //         if (mag_poll.revents & POLLIN) {
    //             struct sensor_mag mag_data;
    //             err = orb_copy(ORB_ID(sensor_mag), mag_fd, &mag_data);
    //             if (err < 0) {
    //                 inerr("Error copying uORB data: %s\n", strerror(errno));
    //                 continue;
    //             }

    //             if (mag_data.x < min_x) min_x = mag_data.x;
    //             if (mag_data.x > max_x) max_x = mag_data.x;
    //             if (mag_data.y < min_y) min_y = mag_data.y;
    //             if (mag_data.y > max_y) max_y = mag_data.y;
    //         }
    //     }
    // }

    calib->hard_iron_x = (max_x + min_x) / 2.0f;
    calib->hard_iron_y = (max_y + min_y) / 2.0f;
    calib->soft_radius_x = (max_x - min_x) / 2.0f;
    calib->soft_radius_y = (max_y - min_y) / 2.0f;

    ininfo("Calibration complete:\n");
    printf("Hard iron X=%.1f Y=%.1f\n", calib->hard_iron_x, calib->hard_iron_y);
    printf("Soft iron X=%.1f Y=%.1f\n", calib->soft_radius_x, calib->soft_radius_y);

    pan_cmd.angle = 0.0f;
    orb_publish_multi(pan_fd, &pan_cmd, sizeof(pan_cmd));
    usleep(1000 * 3000);

    float avg_heading;
    for (int i = 0; i < 5; i++) {
        int err = poll(&mag_poll, 1, 0);
        if (err < 0) {
            inerr("Error polling uORB: %s\n", strerror(errno));
            continue;
        }

        if (mag_poll.revents & POLLIN) {
            struct sensor_mag mag_data;
            err = orb_copy(ORB_ID(sensor_mag), mag_fd, &mag_data);
            if (err < 0) {
                inerr("Error copying uORB data: %s\n", strerror(errno));
                continue;
            }

            float heading;
            if (mag_to_heading(&mag_data, calib, &heading) == 0) {
                avg_heading += heading / 5.0f;
            };
        }
    }

    ininfo("Average heading: %.3f\n", avg_heading);
    if (avg_heading <= 180.0f)
        avg_heading = -avg_heading;
    else
        avg_heading = 360.0f - avg_heading;
    ininfo("Heading correction: %.3f\n", avg_heading);
    calib->angle_correction = avg_heading;

    // for (float angle = CONFIG_INSPACE_TRACKER_MAX_ANGLE - 135.0f; angle >= CONFIG_INSPACE_TRACKER_MIN_ANGLE - 135.0f;
    //      angle -= 1.5f) {
    //     pan_cmd.angle = angle;
    //     orb_publish_multi(pan_fd, &pan_cmd, sizeof(pan_cmd));

    //     usleep(1000 * 10);

    //     int err = poll(&mag_poll, 1, 0);
    //     if (err < 0) {
    //         inerr("Error polling uORB: %s\n", strerror(errno));
    //         continue;
    //     }

    //     if (mag_poll.revents & POLLIN) {
    //         struct sensor_mag mag_data;
    //         err = orb_copy(ORB_ID(sensor_mag), mag_fd, &mag_data);
    //         if (err < 0) {
    //             inerr("Error copying uORB data: %s\n", strerror(errno));
    //             continue;
    //         }

    //         float heading;
    //         if (mag_to_heading(&mag_data, calib, &heading) == 0) {
    //             if (heading < 2.0f || heading > 358.0f) {
    //                 calib->angle_correction = angle;
    //                 ininfo("North offset: %.1f deg\n", calib->angle_correction);
    //                 orb_unsubscribe(pan_fd);
    //                 orb_unsubscribe(mag_fd);
    //                 return 0;
    //             }
    //         }
    //     }
    // }
    return 0;
}

int main(void) {

    int err;
    err = setup_syslogging();
    if (err != 0) {
        printf("Failed to setup syslogging: %s\n", strerror(err));
        return -1;
    }

    err = imu_configure();
    if (err < 0) {
        inerr("IMU configuration failed\n");
    }

    pthread_t movement_thread;
    err = pthread_create(&movement_thread, NULL, movement_main, NULL);
    if (err != 0) {
        inerr("Failed to create movement thread: %s\n", strerror(err));
        return -1;
    }

    /* Wait for servos to home, there is probably a better way than just to sleep a hardcoded amount  */
    sleep(2);

    mag_calib_t calib = {.hard_iron_x = 0.0f,
                         .hard_iron_y = 0.0f,
                         .soft_radius_x = 1.0f,
                         .soft_radius_y = 1.0f,
                         .angle_correction = 0.0f};
    err = run_mag_calibration(&calib);
    if (err < 0) {
        inerr("Mag calibration failed, continuing without calib values\n");
    }

    pthread_t collection_thread;
    ininfo("Starting collection\n");
    err = pthread_create(&collection_thread, NULL, collection_main, NULL);
    if (err != 0) {
        inerr("Failed to create collection thread: %s\n", strerror(err));
        return -1;
    }

    pthread_t aiming_thread;
    err = pthread_create(&aiming_thread, NULL, aiming_main, &calib);
    if (err != 0) {
        inerr("Failed to create aiming thread: %s\n", strerror(err));
        return -1;
    }

    pthread_join(aiming_thread, NULL);
    pthread_join(movement_thread, NULL);
    pthread_join(collection_thread, NULL);
    return 0;
}