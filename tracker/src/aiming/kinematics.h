#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "stdint.h"

typedef struct {
    float x;
    float y;
    float z;
} pos_vec_t;

float const_accel_eq(uint16_t time_s, float vel, float accel, float orig_pos);
int pos_to_vels(aiming_input_telem_t *aiming_input_telem, int size, pos_vec_t *velocities);
int pos_to_accels(aiming_input_telem_t *aiming_input_telem, int size, pos_vec_t *accelerations);
int get_avg(pos_vec_t *vectors, int size, pos_vec_t *avg);

#endif // KINEMATICS_H