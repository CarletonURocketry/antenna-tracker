#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "../tracker.h"
#include "../packets/packet.h"
#include <pthread.h>
#include "collection.h"
#include "mocking/el_blasto.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <uORB/uORB.h>
#include <poll.h>

#ifdef CONFIG_LPWAN_RN2XX3
    #include <nuttx/wireless/ioctl.h>
    #include <nuttx/wireless/lpwan/rn2xx3.h>
#endif

#if defined(CONFIG_DEBUG_UORB)
static const char sensor_alt_format[] = "sensor altitude - timestamp:%" PRIu64 ",altitude:%hf";
ORB_DEFINE(sensor_altitude, struct sensor_altitude, sensor_alt_format);
#else
ORB_DEFINE(sensor_altitude, struct sensor_altitude, 0);
#endif

ORB_DECLARE(sensor_gnss);
ORB_DECLARE(sensor_altitude);

enum {
    ROCKET_GNSS,
    ROCKET_ALT
};

#define config_error(err)                                                                                              \
    if (err) {                                                                                                         \
        err = errno;                                                                                                   \
        inerr("Error configuring radio, line %d: %d\n", __LINE__, err);                                                \
        return err;                                                                                                    \
    }
    
int configure_radio(int fd, struct radio_options const *config) {
    int err = 0;

#if defined(CONFIG_LPWAN_RN2XX3)
    int32_t txpwr = config->txpwr * 100;
    uint64_t sync = config->sync;

    err = ioctl(fd, WLIOC_SETRADIOFREQ, config->freq);
    config_error(err);
    err = ioctl(fd, WLIOC_SETTXPOWERF, &txpwr);
    config_error(err);
    err = ioctl(fd, WLIOC_SETSPREAD, config->spread);
    config_error(err);
    err = ioctl(fd, WLIOC_SETCODERATE, config->cr);
    config_error(err);
    err = ioctl(fd, WLIOC_SETBANDWIDTH, config->bw);
    config_error(err);
    err = ioctl(fd, WLIOC_CRCEN, config->crc);
    config_error(err);
    err = ioctl(fd, WLIOC_IQIEN, config->iqi);
    config_error(err);
    err = ioctl(fd, WLIOC_SETSYNC, &sync);
    config_error(err);
    err = ioctl(fd, WLIOC_SETPRLEN, config->preamble);
    config_error(err);
#endif /* defined(CONFIG_LPWAN_RN2XX3) */

    return err;
}

int parse_packet(uint8_t* buffer, ssize_t buff_len, struct pollfd uorb_fds_out[]){
    pkt_hdr_t* header = (pkt_hdr_t*)buffer;
    /* There was a bug where instead of padding the call sign with null terminators we just padded wiht ascii 0's,
       as a workaround for now we can assume the call sign len is always 6 chars
    */

    buffer += sizeof(pkt_hdr_t);

    for (int i = 0; i < header->blocks; i++) {
        blk_hdr_t* block_hdr = (blk_hdr_t*)buffer;

        ssize_t block_size = blk_body_len(block_hdr->type);

        for(int j = 0; j < block_hdr->count; j++){
            switch (block_hdr->type) {
                case DATA_ALT_SEA: {
                    struct alt_blk_t* alt_blk = (struct alt_blk_t*) buffer + sizeof(blk_hdr_t) + block_size * j;

                    struct sensor_altitude altitude = {
                        .timestamp = parse_blk_timestamp_ms(header->timestamp, alt_blk->time_offset),
                        .altitude = (float)alt_blk->altitude
                    };

                    ininfo("Time: %d - Alt: %d", altitude.timestamp, altitude.altitude);
                    orb_publish_multi(uorb_fds_out[ROCKET_ALT].fd, &altitude, sizeof(altitude));
                    break;
                }
                case DATA_LAT_LONG: {
                    struct coord_blk_t* coord_blk = (struct coord_blk_t*) buffer + sizeof(blk_hdr_t) + block_size * j;

                    struct sensor_gnss coord = {
                        .timestamp = parse_blk_timestamp_ms(header->timestamp, coord_blk->time_offset),
                        .latitude = coord_blk->latitude,
                        .longitude = coord_blk->longitude
                    };

                    ininfo("Time: %d - Lat: %d - Long: %d\n", coord.timestamp, coord.latitude, coord.longitude);
                    orb_publish_multi(uorb_fds_out[ROCKET_GNSS].fd, &coord, sizeof(coord));
                    break;
                }
                case DATA_ANGULAR_VEL: {
                    struct ang_vel_blk_t* ang_vel_blk = (struct ang_vel_blk_t*) buffer + sizeof(blk_hdr_t) + block_size * j;
                    ininfo("Angular Velocity - Time: %d - X: %d - Y: %d - Z: %d\n", parse_blk_timestamp_ms(header->timestamp, ang_vel_blk->time_offset), ang_vel_blk->x, ang_vel_blk->y, ang_vel_blk->z);
                    break;
                }
                case DATA_ACCEL_REL: {
                    struct accel_blk_t* accel_blk = (struct accel_blk_t*) buffer + sizeof(blk_hdr_t) + block_size * j;
                    ininfo("Acceleration - Time: %d - X: %d - Y: %d - Z: %d\n", parse_blk_timestamp_ms(header->timestamp, accel_blk->time_offset), accel_blk->x, accel_blk->y, accel_blk->z);
                    break;
                }
                case DATA_MAGNETIC: {
                    struct mag_blk_t* mag_blk = (struct mag_blk_t*) buffer + sizeof(blk_hdr_t) + block_size * j;
                    ininfo("Magnetic - Time: %d - X: %d - Y: %d - Z: %d\n", parse_blk_timestamp_ms(header->timestamp, mag_blk->time_offset), mag_blk->x, mag_blk->y, mag_blk->z);
                    break;
                }
            }
        }

        buffer += sizeof(blk_hdr_t) + blk_body_len(block_hdr->type) * block_hdr->count;
    }

    return 0;
}


void* collection_main(void* args){
    struct pollfd uorb_fds_out[] = {
        [ROCKET_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
        [ROCKET_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
    };

    struct orb_metadata const *uorb_metas_out[] = {
        [ROCKET_GNSS] = ORB_ID(sensor_gnss),
        [ROCKET_ALT] = ORB_ID(sensor_altitude),
    };

    for(int i = 0; i < sizeof(uorb_metas_out) / sizeof(uorb_metas_out[0]); i++){
        if(uorb_metas_out[i] == NULL){
            inerr("Error getting uORB metadata: %s\n", strerror(errno));
            pthread_exit(NULL);
        }
    }

    int rocket_gnss_instance = 1;
    int rocket_alt_instance = 0;
    uorb_fds_out[ROCKET_GNSS].fd = orb_advertise_multi_queue(uorb_metas_out[ROCKET_GNSS], NULL, &rocket_gnss_instance, 1);
    uorb_fds_out[ROCKET_ALT].fd = orb_advertise_multi_queue(uorb_metas_out[ROCKET_ALT], NULL, &rocket_alt_instance, 1);

    for(int i = 0; i < sizeof(uorb_fds_out) / sizeof(uorb_fds_out[0]); i++){
        if(uorb_fds_out[i].fd < 0){
            inerr("Error advertising to uORB %s: %s\n", uorb_metas_out[i]->o_name, strerror(errno));
            pthread_exit(NULL);
        }
    }

    #ifdef CONFIG_INSPACE_TRACKER_RADIO_MOCK 
        ininfo("Mocking radio data")

        FILE* telem_file = fmemopen(EL_BLASTO_RAW_HEX, sizeof(EL_BLASTO_RAW_HEX), "r");
        if(telem_file == NULL){
            inerr("Failed to open telem file: %s\n", strerror(errno));
            pthread_exit(NULL);
        }

        char buffer[PACKET_MAX_SIZE];
        char line[PACKET_MAX_SIZE * 2];
        while(fgets(line, sizeof(line), telem_file) != NULL){
            size_t byte_count = 0;

            /* TODO: handle odd len hex strings*/
            for (size_t i = 0; i < strlen(line); i += 2) {
                if (line[i] == '\n' || line[i + 1] == '\n') {
                    break;
                }
                
                sscanf(&line[i], "%2hhx", (unsigned char*)&buffer[byte_count++]);
            }
            parse_packet(buffer, byte_count, orb_fds_out);
        }

        fclose(telem_file);
    #endif

    #ifdef CONFIG_INSPACE_TRACKER_RADIO
    int err;

    int radio_fd = open(CONFIG_INSPACE_TRACKER_RADIO_PATH, O_RDWR);
    if (radio_fd < 0) {
        err = errno;
        inerr("Error getting radio handle: %d\n", err);
        goto err_cleanup;
    }

    /* TODO: maybe add this to the config */
    struct radio_options config = {
        .sync = 0x43,
        .freq = 433050000,
        .txpwr = 15,
        .bw = 125,
        .preamble = 6,
        .spread = 7,
        .cr = RN2XX3_CR_4_5,
        .crc = true,
        .iqi = false,
    };

    err = configure_radio(radio_fd, &config);
    if (err) {
        inerr("Error configuring radio: %d\n", err);
        /* Error will have been reported in configure_rn2483 where we can say which
         * config failed in particular */
        goto err_cleanup;
    }

    uint8_t buffer[PACKET_MAX_SIZE];
    ssize_t b_read;

    for(;;){
        ininfo("Waiting for packet...\n");
        b_read = read(radio_fd, buffer, sizeof(buffer));
        if (b_read < 0)
        {
            err = errno;
            inerr("Error receiving from radio: %d\n", err);
            goto err_cleanup;
        }

        for (ssize_t i = 0; i < b_read; i++) {
            printf("%02X ", buffer[i]);
        }
        printf("\n");

        err = parse_packet(buffer, b_read, uorb_fds_out);
        if(err < 0){
            inerr("Error parsing packet: %d\n", err);
        }

    }

err_cleanup:
        if (radio_fd != -1) {
            close(radio_fd);
        }
    #endif
    
    pthread_exit(NULL);
}