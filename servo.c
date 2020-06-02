#include <avr/io.h>
#include "servo.h"

#define SERVO_PWM_PERIOD  ((int16_t)(16000000LL / 50 / 64 / 2))
#define SERVO_COMPARE     OCR1B
#define SERVO_MIN 		  (SERVO_PWM_PERIOD / 20)
#define SERVO_MAX 		  (SERVO_PWM_PERIOD / 10)
void servo_init()
{
	TCCR1A = 0b00100010; // OCR4A Outputting + Mode 10
	TCCR1B = 0b00010011; // 64 Prescaler     + Mode 10
	ICR1 = SERVO_PWM_PERIOD;   // set the top value
	DDRB |= 0b1000000;        // set bit 6 of port B

	servo_pos(90);
}
void servo_pos(int16_t angle_deg)
{
	// clamp angle between 0 and 180
	if(angle_deg < 0)
		angle_deg = 0;
	else if(angle_deg > 180) 
		angle_deg = 180;

	SERVO_COMPARE = SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle_deg / 180;
}

void servo_off()
{
	SERVO_COMPARE = 0;
}
