#include "motor.h"

#include <avr/io.h>

void motor_init() // Uses TIMER3 for pwm
{
	// Configure timer3 to be in mode 10 (spread across TCCR3{A,B})
	TCCR3A = 0b10100010; // OC3A & OC3B Outputting
	TCCR3B = 0b00010001; // 1 Prescaler
	ICR3 = MOTOR_PWM_PERIOD; // Set the top value for timer3
	DDRE |= 0b00011000; // PWM pins are outputting (cant clobber the whole port like with TCCRA{A..B})
	R_MOTOR_SPEED = (uint16_t)MOTOR_PWM_PERIOD * 8 / 10; // Initially 80% duty cycle
	L_MOTOR_SPEED = (uint16_t)MOTOR_PWM_PERIOD * 2 / 3;  // Initially 66% duty cycle

	////////// MOTOR CONTROL PORT A SETUP //////////
	DDRA  |= 0b00001111; // Set the motor control pins to output
	PORTA &= ~0b00001010; // Turn OFF these bits
	PORTA |=  0b00000101; // Turn ON  these bits
	// We can only have one in a pair set a time.

	// Use set_motor_dir(L_MOTOR, FORWARD) in code to set the direction.
}

int set_motor_dir(int lsb, bool forward)
{
	PORTA &= ~(0b11 << lsb); // Turn the motor off
	if(forward)
		PORTA |= (0b01 << lsb);
	else
		PORTA |= (0b10 << lsb);
}
