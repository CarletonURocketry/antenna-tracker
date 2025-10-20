#include <stdio.h>
#include <errno.h>
#include <string.h>
#include "../tracker.h"
#include "../packets/packet.h"
#include <pthread.h>
#include "radio.h"
#include "mocking/el_blasto.h"

void* collection_main(void* args){

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

    ininfo("Telem file closed");

    pthread_exit(NULL);
}