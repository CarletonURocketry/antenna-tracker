#include "utm.h"
#include "aiming.h"
#include "kinematics.h"
#include <stdlib.h>
#include <assert.h>

// # Constant Acceleration Equation
// def const_acc_eq(vel, time, acc, orig_pos):
//     return orig_pos + vel * time + 0.5 * acc * (time ** 2)

float const_accel_eq(double time_s, double vel, double accel, double orig_pos) {
    return orig_pos + vel * time_s + 0.5 * accel * (time_s * time_s);
}

integrated_pos_t pos_to_vels(aiming_input_telem_t *aiming_input_telem, int num_points) {
    // Minus 1 cuz we lose a point when calculating velocity
    int size = num_points - 1;

    double *easting_vel = malloc(size * sizeof(double));
    double *northing_vel = malloc(size * sizeof(double));
    double *alt_vel = malloc(size * sizeof(double));
    double *time_diffs = malloc(size * sizeof(double));

    assert(easting_vel != NULL);
    assert(northing_vel != NULL);
    assert(alt_vel != NULL);
    assert(time_diffs != NULL);

    // Skip first point cuz we need 2 points to calculate velocity
    for (int i = 1; i < num_points; i++) {
        UTMCoord pos = latlon_to_utm(aiming_input_telem[i].rocket_gnss.latitude, aiming_input_telem[i].rocket_gnss.longitude);
        UTMCoord prev_pos = latlon_to_utm(aiming_input_telem[i - 1].rocket_gnss.latitude, aiming_input_telem[i - 1].rocket_gnss.longitude);

        double time_diff = aiming_input_telem[i].rocket_gnss.timestamp - aiming_input_telem[i - 1].rocket_gnss.timestamp;

        easting_vel[i - 1] = (pos.easting - prev_pos.easting) / time_diff;
        northing_vel[i - 1] = (pos.northing - prev_pos.northing) / time_diff;
        alt_vel[i - 1] = (aiming_input_telem[i].rocket_gnss.altitude - aiming_input_telem[i - 1].rocket_gnss.altitude) / time_diff;
        time_diffs[i - 1] = time_diff;
    }

    return (integrated_pos_t) {
        easting_vel,
        northing_vel,
        alt_vel,
        time_diffs,
        size
    };
}

integrated_pos_t pos_to_accels(aiming_input_telem_t *aiming_input_telem, int num_points) {
    integrated_pos_t vel_data = pos_to_vels(aiming_input_telem, num_points);
    int size = num_points - 2;

    // Minus 2 cuz we lose another point when calculating acceleration
    double *easting_accel = malloc(size * sizeof(double));
    double *northing_accel = malloc(size * sizeof(double));
    double *alt_accel = malloc(size * sizeof(double));
    double *time_diffs = malloc(size * sizeof(double));

    assert(easting_accel != NULL);
    assert(northing_accel != NULL);
    assert(alt_accel != NULL);
    assert(time_diffs != NULL);

    // Skip first point cuz we need 2 points to calculate acceleration
    for (int i = 1; i < num_points - 1; i++) {
        double time_diff = vel_data.time_diffs[i] - vel_data.time_diffs[i - 1];

        easting_accel[i - 1] = (vel_data.easting[i] - vel_data.easting[i - 1]) / time_diff;
        northing_accel[i - 1] = (vel_data.northing[i] - vel_data.northing[i - 1]) / time_diff;
        alt_accel[i - 1] = (vel_data.altitude[i] - vel_data.altitude[i - 1]) / time_diff;
        time_diffs[i - 1] = time_diff;
    }

    free(vel_data.easting);
    free(vel_data.northing);
    free(vel_data.altitude);
    free(vel_data.time_diffs);

    vel_data.easting = NULL;
    vel_data.northing = NULL;
    vel_data.altitude = NULL;
    vel_data.time_diffs = NULL;


    return (integrated_pos_t) {
        easting_accel,
        northing_accel,
        alt_accel,
        time_diffs,
        size
    };
}


integrated_avg_pos_t get_avg_vel(aiming_input_telem_t *aiming_input_telem, int num_points) {
    integrated_pos_t vel_data = pos_to_vels(aiming_input_telem, num_points);
    double total_vel_easting = 0.0;
    double total_vel_northing = 0.0;
    double total_vel_altitude = 0.0;

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

    free(vel_data.easting);
    free(vel_data.northing);
    free(vel_data.altitude);
    free(vel_data.time_diffs);

    vel_data.easting = NULL;
    vel_data.northing = NULL;
    vel_data.altitude = NULL;
    vel_data.time_diffs = NULL;

    return result;
}

integrated_avg_pos_t get_avg_accel(aiming_input_telem_t *aiming_input_telem, int num_points) {
    integrated_pos_t accel_data = pos_to_accels(aiming_input_telem, num_points);
    double total_accel_easting = 0.0;
    double total_accel_northing = 0.0;
    double total_accel_altitude = 0.0;

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

    free(accel_data.easting);
    free(accel_data.northing);
    free(accel_data.altitude);
    free(accel_data.time_diffs);

    accel_data.easting = NULL;
    accel_data.northing = NULL;
    accel_data.altitude = NULL;
    accel_data.time_diffs = NULL;

    return result;
}