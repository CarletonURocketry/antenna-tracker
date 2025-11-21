#include "utm.h"
#include "aiming.h"
#include "kinematics.h"
#include <stdlib.h>
#include <assert.h>


float const_accel_eq(uint16_t time_ms, float vel, float accel, float orig_pos) {
    int time_s = time_ms / 1000;
    return orig_pos + vel * time_s + 0.5 * accel * (time_s * time_s);
}

int pos_to_vels(aiming_input_telem_t *aiming_input_telem, int num_points, integrated_pos_t *velocities) {
    /* We loose a point from calculating vel */
    int size = num_points - 1;

    float easting_vel[size];
    float northing_vel[size];
    float alt_vel[size];

    /* Skip first point cuz we need 2 points to calculate velocity */
    for (int i = 1; i < num_points; i++) {
        utm_coord_t pos = latlon_to_utm(aiming_input_telem[i].rocket_gnss.latitude, aiming_input_telem[i].rocket_gnss.longitude);
        utm_coord_t prev_pos = latlon_to_utm(aiming_input_telem[i - 1].rocket_gnss.latitude, aiming_input_telem[i - 1].rocket_gnss.longitude);

        easting_vel[i - 1] = (pos.easting - prev_pos.easting) / ROCKET_SAMPLE_DT_MS;
        northing_vel[i - 1] = (pos.northing - prev_pos.northing) / ROCKET_SAMPLE_DT_MS;
        alt_vel[i - 1] = (aiming_input_telem[i].rocket_gnss.altitude - aiming_input_telem[i - 1].rocket_gnss.altitude) / ROCKET_SAMPLE_DT_MS;
    }


    velocities->easting = easting_vel;
    velocities->northing = northing_vel;
    velocities->altitude = alt_vel;
    velocities->size = size;
    
    return 0;
}

int pos_to_accels(aiming_input_telem_t *aiming_input_telem, int num_points, integrated_pos_t *accelerations) {
    integrated_pos_t vel_data;
    pos_to_vels(aiming_input_telem, num_points, &vel_data);

    /* Minus 2 cuz we lose another point when calculating acceleration (lost 1 from vel) */
    int size = num_points - 2;

    float easting_accel[size];
    float northing_accel[size];
    float alt_accel[size];

    /* Skip first point cuz we need 2 points to calculate acceleration */
    for (int i = 1; i < num_points - 1; i++) {
        easting_accel[i - 1] = (vel_data.easting[i] - vel_data.easting[i - 1]) / ROCKET_SAMPLE_DT_MS;
        northing_accel[i - 1] = (vel_data.northing[i] - vel_data.northing[i - 1]) / ROCKET_SAMPLE_DT_MS;
        alt_accel[i - 1] = (vel_data.altitude[i] - vel_data.altitude[i - 1]) / ROCKET_SAMPLE_DT_MS;
    }

    accelerations->easting = easting_accel;
    accelerations->northing = northing_accel;
    accelerations->altitude = alt_accel;
    accelerations->size = size;

    return 0;
}


integrated_avg_pos_t get_avg_vel(aiming_input_telem_t *aiming_input_telem, int num_points) {
    integrated_pos_t vel_data;
    pos_to_vels(aiming_input_telem, num_points, &vel_data);

    float total_vel_easting = 0.0;
    float total_vel_northing = 0.0;
    float total_vel_altitude = 0.0;

    for (int i = 0; i < vel_data.size; i++) {
        total_vel_easting += vel_data.easting[i];
        total_vel_northing += vel_data.northing[i];
        total_vel_altitude += vel_data.altitude[i];
    }

    integrated_avg_pos_t result = (integrated_avg_pos_t) {
        total_vel_easting / vel_data.size,
        total_vel_northing / vel_data.size,
        total_vel_altitude / vel_data.size,
    };

    return result;
}

integrated_avg_pos_t get_avg_accel(aiming_input_telem_t *aiming_input_telem, int num_points) {
    integrated_pos_t accel_data;
    pos_to_accels(aiming_input_telem, num_points, &accel_data);

    float total_accel_easting = 0.0;
    float total_accel_northing = 0.0;
    float total_accel_altitude = 0.0;

    for (int i = 0; i < accel_data.size; i++) {
        total_accel_easting += accel_data.easting[i];
        total_accel_northing += accel_data.northing[i];
        total_accel_altitude += accel_data.altitude[i];
    }

    integrated_avg_pos_t result = (integrated_avg_pos_t) {
        total_accel_easting / accel_data.size,
        total_accel_northing / accel_data.size,
        total_accel_altitude / accel_data.size,
    };

    return result;
}