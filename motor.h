#ifndef MOTOR_H
#define MOTOR_H
#include <stdbool.h>
#include <inttypes.h>

#define L_MOTOR 0
#define R_MOTOR 2

#define L_MOTOR_SPEED OCR3B
#define R_MOTOR_SPEED OCR3A

#define FORWARD false
#define BACKWARD true

#define FULL_SPEED 512 // Since duty cycle isn't proportional to the speed
#define HALF_SPEED 448 // of the motors, these defines can be used to
#define SLOW_SPEED 384 // set the speed more intuitively.
#define STOP_SPEED 0   

// 10kHz base frequency
#define MOTOR_PWM_PERIOD ((int16_t)(16000000UL / 1 / 10000 / 2))

void motor_init(); // Uses TIMER3 for pwm
int set_motor_dir(int lsb, bool forward);

#endif
