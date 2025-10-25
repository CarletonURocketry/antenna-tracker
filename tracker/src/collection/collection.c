#include "../packets/packet.h"
#include "../tracker.h"
#include "radio.h"
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include "collection.h"
#include <uORB/uORB.h>
#include <poll.h>

#define TENTH_DEGREE_TO_RAD 3.18309886184

void *collection_main(void *args) {

    orb_advertise_multi_queue();

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