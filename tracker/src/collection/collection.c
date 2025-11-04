#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "../tracker.h"
#include "../packets/packet.h"
#include <pthread.h>
#include "radio.h"
#include "mocking/el_blasto.h"
#include <fcntl.h>
#include <sys/ioctl.h>

#ifdef CONFIG_LPWAN_RN2XX3
    #include <nuttx/wireless/ioctl.h>
    #include <nuttx/wireless/lpwan/rn2xx3.h>
#endif

#define config_error(err)                                                                                              \
    if (err) {                                                                                                         \
        err = errno;                                                                                                   \
        inerr("Error configuring radio, line %d: %d\n", __LINE__, err);                                                \
        return err;                                                                                                    \
    }
    
int configure_radio(int fd, struct radio_options const *config) {
    int err = 0;

// #if defined(CONFIG_LPWAN_RN2XX3)
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
// #endif /* defined(CONFIG_LPWAN_RN2XX3) */

    return err;
}

void* collection_main(void* args){
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
            parse_packet(buffer);
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
        b_read = read(radio_fd, buffer, sizeof(buffer));
        if (b_read < 0)
        {
            err = errno;
            inerr("Error receiving from radio: %d\n", err);
            goto err_cleanup;
        }
        printf("%s\n", buffer);
    }

err_cleanup:
        if (radio_fd != -1) {
            close(radio_fd);
        }
    #endif
    
    pthread_exit(NULL);
}