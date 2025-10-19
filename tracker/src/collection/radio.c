#include "radio.h"
#include "../packets/packet.h"
#include <stdio.h>

parsed_packet parse_packet(char* buffer){
    pkt_hdr_t* header = (pkt_hdr_t*)buffer;
    /* There was a bug where instead of padding the call sign with null terminators we just padded wiht ascii 0's,
       as a workaround for now we can assume the call sign len is always 6 chars
    */

    // Offset for the packet header
    size_t offset = sizeof(pkt_hdr_t);

    parsed_packet packet;

    // Should we clear memory to 0 for each packet? Or keep previous values if block not present?
    // For now last known values if block not present.
    // Reasoning: If we don't have the position at a given time, the tracker might go into a bad state/return to default position instead of staying where it is.
    // memset(&packet, 0, sizeof(packet));

    for (uint8_t i = 0; i < header->blocks; i++) {
        blk_hdr_t* block = (blk_hdr_t*)(buffer + offset);

        size_t block_size = sizeof(blk_hdr_t) + blk_body_len(block->type);
        offset += block_size;

        switch (block->type) {
            case DATA_ALT_SEA: {
                struct alt_blk_t* alt_block = (struct alt_blk_t*)block_body((uint8_t*)block);
                packet.alt_sea = *alt_block;
                break;
            }
            case DATA_ALT_LAUNCH: {
                struct alt_blk_t* alt_block = (struct alt_blk_t*)block_body((uint8_t*)block);
                packet.alt_launch = *alt_block;
                break;
            }
            case DATA_ANGULAR_VEL: {
                struct ang_vel_blk_t* ang_vel_block = (struct ang_vel_blk_t*)block_body((uint8_t*)block);
                packet.ang_vel = *ang_vel_block;
                break;
            }
            case DATA_ACCEL_REL: {
                struct accel_blk_t* accel_block = (struct accel_blk_t*)block_body((uint8_t*)block);
                packet.accel = *accel_block;
                break;
            }
            case DATA_LAT_LONG: {
                struct coord_blk_t* coord_block = (struct coord_blk_t*)block_body((uint8_t*)block);
                packet.coord = *coord_block;
                break;
            }
        }
    }


    // Print the parsed packet data
    printf("--------------Block count: %u--------------\n", header->blocks);
    printf("Altitude Sea: %d\n", packet.alt_sea.altitude);
    printf("Altitude Launch: %d\n", packet.alt_launch.altitude);
    printf("Angular Velocity: x=%d, y=%d, z=%d\n", packet.ang_vel.x, packet.ang_vel.y, packet.ang_vel.z);
    printf("Acceleration: x=%d, y=%d, z=%d\n", packet.accel.x, packet.accel.y, packet.accel.z);
    printf("Coordinates: lat=%d, lon=%d\n", packet.coord.latitude, packet.coord.longitude);

    return packet;
}