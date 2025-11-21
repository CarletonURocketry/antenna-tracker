#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "stdint.h"

typedef struct {
    float *easting;
    float *northing;
    float *altitude;
    int size;
} integrated_pos_t;

typedef struct {
    float easting;
    float northing;
    float altitude;
} integrated_avg_pos_t;

float const_accel_eq(uint16_t time_s, float vel, float accel, float orig_pos);
int pos_to_vels(aiming_input_telem_t *aiming_input_telem, int num_points, integrated_pos_t *velocities);
int pos_to_accels(aiming_input_telem_t *aiming_input_telem, int num_points, integrated_pos_t *accelerations);
integrated_avg_pos_t get_avg_vel(aiming_input_telem_t *aiming_input_telem, int num_points);
integrated_avg_pos_t get_avg_accel(aiming_input_telem_t *aiming_input_telem, int num_points);

#endif // KINEMATICS_H