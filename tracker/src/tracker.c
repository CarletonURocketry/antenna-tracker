#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "packets/packet.h"


int parse_packet(char* buffer){
    pkt_hdr_t* header = (pkt_hdr_t*)buffer;
    /* There was a bug where instead of padding the call sign with null terminators we just padded wiht ascii 0's,
       as a workaround for now we can assume the call sign len is always 6 chars
    */
    printf("Call sign: %.6s\n", header->call_sign);

    return 0;
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