#include "../packets/packet.h"
#include "../tracker.h"
#include "radio.h"
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include "collection.h"
#include <nuttx/uorb.h>
#include <poll.h>

#define TENTH_DEGREE_TO_RAD 3.18309886184

/* Setup UORB topics */

enum uorb_sensors {
    SENSOR_ACCEL,  /* Accelerometer */
    SENSOR_GYRO,   /* Gyroscope */
    SENSOR_BARO,   /* Barometer */
    SENSOR_MAG,    /* Magnetometer */
    SENSOR_GNSS,   /* GNSS */
    SENSOR_ALT,    /* Altitude fusion */
    SENSOR_ERROR,  /* Error messages */
    SENSOR_STATUS, /* Status messages */
};

union uorb_data {
    struct sensor_accel accel;
    struct sensor_gyro gyro;
    struct sensor_baro baro;
    struct sensor_mag mag;
    struct sensor_gnss gnss;

    struct fusion_altitude alt;
    struct error_message error;
    struct status_message status;
};

static struct pollfd uorb_fds[] = {
    [SENSOR_ACCEL] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GYRO] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_MAG] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_ERROR] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_STATUS] = {.fd = -1, .events = POLLIN, .revents = 0},
};

ORB_DECLARE(sensor_accel);
ORB_DECLARE(sensor_gyro);
ORB_DECLARE(sensor_baro);
ORB_DECLARE(sensor_mag);
ORB_DECLARE(sensor_gnss);
ORB_DECLARE(fusion_altitude);

static struct orb_metadata const *uorb_metas[] = {
    [SENSOR_ACCEL] = ORB_ID(sensor_accel),  [SENSOR_GYRO] = ORB_ID(sensor_gyro),
    [SENSOR_BARO] = ORB_ID(sensor_baro),    [SENSOR_MAG] = ORB_ID(sensor_mag),
    [SENSOR_GNSS] = ORB_ID(sensor_gnss),    [SENSOR_ALT] = ORB_ID(fusion_altitude),
    [SENSOR_ERROR] = ORB_ID(error_message), [SENSOR_STATUS] = ORB_ID(status_message),
};

void *collection_main(void *args) {

    FILE *telem_file = fopen("./mocking/el_blasto.hex", "r");
    if (telem_file == NULL) {
        inerr("Failed to open telem file: %s\n", strerror(errno));
        pthread_exit(NULL);
    }

    char buffer[PACKET_MAX_SIZE];
    char line[PACKET_MAX_SIZE * 2];
    while (fgets(line, sizeof(line), telem_file) != NULL) {
        size_t byte_count = 0;

        /* TODO: handle odd len hex strings*/
        for (size_t i = 0; i < strlen(line); i += 2) {
            if (line[i] == '\n' || line[i + 1] == '\n') {
                break;
            }

            sscanf(&line[i], "%2hhx", (unsigned char *)&buffer[byte_count++]);
        }
        parsed_packet packet = parse_packet(buffer);

        /* Convert data to uORB format */

        struct sensor_accel accel = {.temperature = 0,
                                     .timestamp = (packet.timestamp * 30000 + packet.accel.time_offset) / 1000,
                                     .x = packet.accel.x / 100,
                                     .y = packet.accel.y / 100,
                                     .z = packet.accel.z};

        struct sensor_gyro gyro = {.temperature = 0,
                                   .timestamp = (packet.timestamp * 30000 + packet.ang_vel.time_offset) / 1000,
                                   .x = packet.ang_vel.x * TENTH_DEGREE_TO_RAD,
                                   .y = packet.ang_vel.y * TENTH_DEGREE_TO_RAD,
                                   .z = packet.ang_vel.z * TENTH_DEGREE_TO_RAD};

        struct sensor_baro baro = {.temperature = packet.temp.temperature,
                                   .pressure = packet.pres.pressure,
                                   .timestamp = (packet.timestamp * 30000 + packet.pres.time_offset) / 1000};

        struct sensor_mag mag = {.status = 0,
                                 .temperature = 0,
                                 .timestamp = (packet.timestamp * 30000 + packet.mag.time_offset) / 1000,
                                 .x = packet.mag.x * 10,
                                 .y = packet.mag.y * 10,
                                 .z = packet.mag.z * 10};
        
    }

    fclose(telem_file);
    ininfo("Telem file closed");

    pthread_exit(NULL);
}