#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "packets/packet.h"


int parse_packet(char* buffer){
    pkt_hdr_t* header = (pkt_hdr_t*)buffer;
    printf("Packet call sign: %s\n", header->call_sign);
    printf("Packet timestamp: %d\n", header->timestamp);
    printf("Packet blocks: %d\n", header->blocks);
    printf("Packet packet_num: %d\n", header->packet_num);

    return 0;
}

int main(void){
    FILE* telem_file = fopen("./mocking/el_blasto.hex", "r");
    if(telem_file == NULL){
        printf("Failed to open telem file: %s\n", strerror(errno));
        return -1;
    }

    char buffer[PACKET_MAX_SIZE];
    while(fgets(buffer, sizeof(buffer), telem_file) != NULL){
        parse_packet(buffer);
    }

    return 0;
}