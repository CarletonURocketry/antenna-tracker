#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "packets/packet.h"
#include "tracker.h"

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
            case DATA_TEMP: {
                struct temp_blk_t* temp_block = (struct temp_blk_t*)block_body((uint8_t*)block);
                packet.temp = *temp_block;
                break;
            }
            case DATA_PRESSURE: {
                struct pres_blk_t* pres_block = (struct pres_blk_t*)block_body((uint8_t*)block);
                packet.pres = *pres_block;
                break;
            }
            case DATA_HUMIDITY: {
                struct hum_blk_t* hum_block = (struct hum_blk_t*)block_body((uint8_t*)block);
                packet.hum = *hum_block;
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
            case DATA_MAGNETIC: {
                struct mag_blk_t* mag_block = (struct mag_blk_t*)block_body((uint8_t*)block);
                packet.mag = *mag_block;
                break;
            }
            case DATA_LAT_LONG: {
                struct coord_blk_t* coord_block = (struct coord_blk_t*)block_body((uint8_t*)block);
                packet.coord = *coord_block;
                break;
            }
            case DATA_VOLTAGE: {
                struct volt_blk_t* volt_block = (struct volt_blk_t*)block_body((uint8_t*)block);
                packet.volt = *volt_block;
                break;
            }
            case DATA_STATUS: {
                struct status_blk_t* status_block = (struct status_blk_t*)block_body((uint8_t*)block);
                packet.status = *status_block;
                break;
            }
            case DATA_ERROR: {
                struct error_blk_t* error_block = (struct error_blk_t*)block_body((uint8_t*)block);
                packet.error = *error_block;
                break;
            }
        }
    }


    // Print the parsed packet data
    // printf("--------------Block count: %u--------------\n", header->blocks);
    // printf("Altitude Sea: %d\n", packet.alt_sea.altitude);
    // printf("Altitude Launch: %d\n", packet.alt_launch.altitude);
    // printf("Temperature: %d\n", packet.temp.temperature);
    // printf("Pressure: %d\n", packet.pres.pressure);
    // printf("Humidity: %d\n", packet.hum.humidity);
    // printf("Angular Velocity: x=%d, y=%d, z=%d\n", packet.ang_vel.x, packet.ang_vel.y, packet.ang_vel.z);
    // printf("Acceleration: x=%d, y=%d, z=%d\n", packet.accel.x, packet.accel.y, packet.accel.z);
    // printf("Magnetic: x=%d, y=%d, z=%d\n", packet.mag.x, packet.mag.y, packet.mag.z);
    // printf("Coordinates: lat=%d, lon=%d\n", packet.coord.latitude, packet.coord.longitude);
    // printf("Voltage: %d\n", packet.volt.voltage);
    // printf("Status: %u\n", packet.status.status_code);
    // printf("Error: %u\n", packet.error.error_code);

    return packet;
}

int main(void){
    FILE* telem_file = fopen("./mocking/el_blasto.hex", "r");
    if(telem_file == NULL){
        printf("Failed to open telem file: %s\n", strerror(errno));
        return -1;
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

    return 0;
}