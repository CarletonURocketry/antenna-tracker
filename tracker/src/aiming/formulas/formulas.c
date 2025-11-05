// # Constant Acceleration Equation
// def const_acc_eq(vel, time, acc, orig_pos):
//     return orig_pos + vel * time + 0.5 * acc * (time ** 2)

float const_accel_eq(double time_s, double vel, double accel, double orig_pos) {
    return orig_pos + vel * time_s + 0.5 * accel * (time_s * time_s);
}
