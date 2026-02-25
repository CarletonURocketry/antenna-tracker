#ifndef _INSPACE_TRACKER_MOTOR_H_
#define _INSPACE_TRACKER_MOTOR_H_

/**
 * @brief Sets up the pwm_state variable
 *
 * @return int 0 on success, -1 on failure
 */
int pwm_state_setup(void);

/**
 * @brief Moves the motor to a specific angle
 *
 * @param angle The angle to move the motor to
 * @param timer The timer or channel index to use for the motor
 * @return int 0 on success, -1 on failure
 */
int move_angle(int angle, int timer);

#endif
