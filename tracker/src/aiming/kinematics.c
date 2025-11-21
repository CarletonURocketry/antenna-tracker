#include "utm.h"
#include "aiming.h"
#include "kinematics.h"
#include <stdlib.h>
#include <assert.h>


float const_accel_eq(uint16_t time_ms, float vel, float accel, float orig_pos) {
    float time_s = time_ms / 1000;
    return orig_pos + vel * time_s + 0.5 * accel * (time_s * time_s);
}

int pos_to_vels(aiming_input_telem_t *aiming_input_telem, int size, pos_vec_t *velocities) {
    /* size parameter represents the number of GNSS samples in the rocket_gnss array */
    int num_samples = aiming_input_telem->rocket_gnss_n;
    if (num_samples < 2) {
        return -1; /* Need at least 2 points */
    }
    
    utm_coord_t utm_coords[num_samples];

    for (int i = 0; i < num_samples; i++) {
        latlon_to_utm(aiming_input_telem->rocket_gnss[i].latitude, aiming_input_telem->rocket_gnss[i].longitude, &utm_coords[i]);
    }
    
    /* Skip first point cuz we need 2 points to calculate velocity */
    for (int i = 0; i < num_samples - 1; i++) {
        velocities[i].x = (utm_coords[i + 1].easting - utm_coords[i].easting) / (ROCKET_SAMPLE_DT_MS / 1000.0f);
        velocities[i].y = (utm_coords[i + 1].northing - utm_coords[i].northing) / (ROCKET_SAMPLE_DT_MS / 1000.0f);
        velocities[i].z = (aiming_input_telem->rocket_gnss[i + 1].altitude - aiming_input_telem->rocket_gnss[i].altitude) / (ROCKET_SAMPLE_DT_MS / 1000.0f);
    }
    
    return 0;
}

int pos_to_accels(aiming_input_telem_t *aiming_input_telem, int size, pos_vec_t *accelerations) {
    int num_samples = aiming_input_telem->rocket_gnss_n;
    if (num_samples < 3) {
        return -1; /* Need at least 3 points */
    }
    
    pos_vec_t vel_data[num_samples - 1];
    if (pos_to_vels(aiming_input_telem, size, vel_data) < 0) {
        return -1;
    }

    /* Skip another point cuz we need 2 points to calculate acceleration */
    for (int i = 0; i < num_samples - 2; i++) {
        accelerations[i].x = (vel_data[i + 1].x - vel_data[i].x) / (ROCKET_SAMPLE_DT_MS / 1000.0f);
        accelerations[i].y = (vel_data[i + 1].y - vel_data[i].y) / (ROCKET_SAMPLE_DT_MS / 1000.0f);
        accelerations[i].z = (vel_data[i + 1].z - vel_data[i].z) / (ROCKET_SAMPLE_DT_MS / 1000.0f);
    }

    return 0;
}

int get_avg(pos_vec_t *vectors, int size, pos_vec_t *avg) {
    avg->x = 0;
    avg->y = 0;
    avg->z = 0;

    for (int i = 0; i < size; i++) {
        avg->x += vectors[i].x;
        avg->y += vectors[i].y;
        avg->z += vectors[i].z;
    }

    avg->x /= size;
    avg->y /= size;
    avg->z /= size;

    return 0;
}