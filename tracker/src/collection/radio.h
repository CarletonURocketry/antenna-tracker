#ifndef _ANTENNA_TRACKER_RADIO_H_
#define _ANTENNA_TRACKER_RADIO_H_

#include "../packets/packet.h"

typedef struct {
    uint16_t timestamp;
    uint8_t packet_num;
    struct alt_blk_t alt_sea;
    struct ang_vel_blk_t ang_vel;
    struct accel_blk_t accel;
    struct coord_blk_t coord;
} parsed_packet;

parsed_packet parse_packet(char* buffer);

#endif