#include "utm.h"
#include "aiming.h"
#include "kinematics.h"

// # Constant Acceleration Equation
// def const_acc_eq(vel, time, acc, orig_pos):
//     return orig_pos + vel * time + 0.5 * acc * (time ** 2)

float const_accel_eq(double time_s, double vel, double accel, double orig_pos) {
    return orig_pos + vel * time_s + 0.5 * accel * (time_s * time_s);
}

integrated_pos_t pos_to_vels(aiming_input_telem_t *aiming_input_telem, int num_points) {
    // Minus 1 cuz we lose a point when calculating velocity
    int size = num_points - 1;

    double easting_vel[size];
    double northing_vel[size];
    double alt_vel[size];
    double time_diffs[size];

    // Skip first point cuz we need 2 points to calculate velocity
    for (int i = 1; i < num_points; i++) {
        UTMCoord pos = latlon_to_utm(aiming_input_telem[i].rocket_gnss.latitude, aiming_input_telem[i].rocket_gnss.longitude);
        UTMCoord prev_pos = latlon_to_utm(aiming_input_telem[i - 1].rocket_gnss.latitude, aiming_input_telem[i - 1].rocket_gnss.longitude);

        double time_diff = aiming_input_telem[i].rocket_gnss.timestamp - aiming_input_telem[i - 1].rocket_gnss.timestamp;

        easting_vel[i] = (pos.easting - prev_pos.easting) / time_diff;
        northing_vel[i] = (pos.northing - prev_pos.northing) / time_diff;
        alt_vel[i] = (aiming_input_telem[i].rocket_gnss.altitude - aiming_input_telem[i - 1].rocket_gnss.altitude) / time_diff;
        time_diffs[i] = time_diff;
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
    double easting_accel[size];
    double northing_accel[size];
    double alt_accel[size];
    double time_diffs[size];

    // Skip first point cuz we need 2 points to calculate acceleration
    for (int i = 1; i < num_points - 1; i++) {
        double time_diff = vel_data.time_diffs[i] - vel_data.time_diffs[i - 1];

        easting_accel[i] = (vel_data.easting[i] - vel_data.easting[i - 1]) / time_diff;
        northing_accel[i] = (vel_data.northing[i] - vel_data.northing[i - 1]) / time_diff;
        alt_accel[i] = (vel_data.altitude[i] - vel_data.altitude[i - 1]) / time_diff;
        time_diffs[i] = time_diff;
    }

    return (integrated_pos_t) {
        easting_accel,
        northing_accel,
        alt_accel,
        time_diffs,
        size
    };
}


double get_avg_vel(aiming_input_telem_t *aiming_input_telem, int num_points) {
    integrated_pos_t vel_data = pos_to_vels(aiming_input_telem, num_points);
    double total_vel = 0.0;

    for (int i = 0; i < vel_data.size; i++) {
        total_vel += vel_data.easting[i];
    }

    return total_vel / vel_data.size;
}

double get_avg_accel(aiming_input_telem_t *aiming_input_telem, int num_points) {
    integrated_pos_t accel_data = pos_to_accels(aiming_input_telem, num_points);
    double total_accel = 0.0;

    for (int i = 0; i < accel_data.size; i++) {
        total_accel += accel_data.easting[i];
    }

    return total_accel / accel_data.size;
}