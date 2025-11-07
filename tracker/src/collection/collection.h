#ifndef _INSPACE_TRACKER_COLLECTION_
#define _INSPACE_TRACKER_COLLECTION_

#include "../packets/packet.h"
#include <stdbool.h>
#include <uORB/uORB.h>
#include <poll.h>

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

/* UORB declarations for fused sensor data */
ORB_DECLARE(sensor_altitude);

/*
 * Size of the internal queues for the fusioned data, which other threads should
 * match the size of their buffers to
 */
#define ORB_ALT_BUFFER 5

/* A fusioned altitude sample */
struct sensor_altitude{
    uint64_t timestamp; /* Timestamp in microseconds */
    float altitude;     /* Altitude in meters */
};


void* collection_main(void*);
int parse_packet(uint8_t* buffer, ssize_t buff_len, struct pollfd uorb_fds_out[]);

#endif