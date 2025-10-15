#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <stdint.h>
#include "packets/packet.h"

typedef struct {
    uint16_t timestamp;
    uint8_t packet_num;
    struct alt_blk_t alt_sea;
    struct alt_blk_t alt_launch;
    struct temp_blk_t temp;
    struct pres_blk_t pres;
    struct hum_blk_t hum;
    struct ang_vel_blk_t ang_vel;
    struct accel_blk_t accel;
    struct mag_blk_t mag;
    struct coord_blk_t coord;
    struct volt_blk_t volt;
    struct status_blk_t status;
    struct error_blk_t error;
} parsed_packet;




#endif