#ifndef _INSPACE_TRACKER_RADIO_H_
#define _INSPACE_TRACKER_RADIO_H_

#include "../packets/packet.h"
#include <stdbool.h>

typedef struct {
    uint16_t timestamp;
    uint8_t packet_num;
    struct alt_blk_t alt_sea;
    struct ang_vel_blk_t ang_vel;
    struct accel_blk_t accel;
    struct coord_blk_t coord;
} parsed_packet;

#ifdef CONFIG_LPWAN_RN2XX3
#include <nuttx/wireless/lpwan/rn2xx3.h>
struct radio_options {
    uint64_t sync;       /* Sync word */
    uint32_t freq;       /* Frequency, Hz */
    int32_t txpwr;       /* Transmit power, dBm */
    uint32_t bw;         /* Bandwidth, kHz */
    uint16_t preamble;   /* Preamble length */
    uint8_t spread;      /* Spread factor */
    enum rn2xx3_cr_e cr; /* Coding rate */
    bool crc;            /* CRC enabled */
    bool iqi;            /* IQI enabled */
};
#endif

parsed_packet parse_packet(char* buffer);

#endif