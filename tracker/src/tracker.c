#include "tracker.h"
#include <pthread.h>
#include "collection/collection.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "aiming/aiming.h"
#include "movement/movement.h"

int main(void){

    int err;
    err = setup_syslogging();
        if(err != 0){
            printf("Failed to setup syslogging: %s\n", strerror(err));
            return -1;
        }

    pthread_t collection_thread;

    ininfo("Starting tracker\n");
    err = pthread_create(&collection_thread, NULL, collection_main, NULL);
    if(err != 0){
        inerr("Failed to create collection thread: %s\n", strerror(err));
        return -1;
    }

    pthread_t aiming_thread;
    err = pthread_create(&aiming_thread, NULL, aiming_main, NULL);
    if(err != 0){
        inerr("Failed to create aiming thread: %s\n", strerror(err));
        return -1;
    }

    pthread_t movement_thread;
    err = pthread_create(&movement_thread, NULL, movement_main, NULL);
    if(err != 0){
        inerr("Failed to create movement thread: %s\n", strerror(err));
        return -1;
    }

    pthread_join(collection_thread, NULL);
    pthread_join(aiming_thread, NULL);
    pthread_join(movement_thread, NULL);
    return 0;
}