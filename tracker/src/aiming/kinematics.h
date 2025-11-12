#ifndef KINEMATICS_H
#define KINEMATICS_H

typedef struct {
    double *easting;
    double *northing;
    double *altitude;
    double *time_diffs;
    int size;
} integrated_pos_t;

typedef struct {
    double easting;
    double northing;
    double altitude;
} integrated_avg_pos_t;

double const_accel_eq(double time_s, double vel, double accel, double orig_pos);
integrated_pos_t pos_to_vels(aiming_input_telem_t *aiming_input_telem, int num_points);
integrated_pos_t pos_to_accels(aiming_input_telem_t *aiming_input_telem, int num_points);
integrated_avg_pos_t get_avg_vel(aiming_input_telem_t *aiming_input_telem, int num_points);
integrated_avg_pos_t get_avg_accel(aiming_input_telem_t *aiming_input_telem, int num_points);

#endif // KINEMATICS_H