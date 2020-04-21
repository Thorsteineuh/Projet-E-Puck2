#ifndef MOTOR_H
#define MOTOR_H


void motor_init(void);
void motors_set_speed(float speed_r, float speed_l);
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);
//bool motor_turn(float radius, int angle, float speed);
void motor_stop(void);

#endif /* MOTOR_H */
