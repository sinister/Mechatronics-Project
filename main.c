#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#include "serial.h" 
#include "adc.h"
#include "joystick.h"

#define ROOT2 1.4142135623f

// Distance from sensor to edge of robot in mm
const int16_t CENTER_OFFSET = 87;
const int16_t SIDE_OFFSET = 37;

volatile uint32_t centiseconds_global = 0;
ISR(TIMER1_COMPA_vect)
{ // Timer tick. Increment centiseconds by 1.
	centiseconds_global += 1;
}

volatile bool interrupt0 = false;
ISR(INT0_vect)
{
	static volatile uint32_t INT0_invalid_to = 0;
	uint32_t centi = centiseconds_global;
	if(INT0_invalid_to < centi)
	{
		interrupt0 = !interrupt0;
		INT0_invalid_to = centi + 30;
	}
}

int16_t front_to_mm(int16_t adc) { return -(53227500ll*adc-268558095607ll)/(2000*(2500ll*adc-78227)); }
int16_t side_to_mm(int16_t adc)  { return -(61377500ll*adc - 110758264243ll)/(1250ll*(2500ll*adc+68970)); }

int main(void)
{

///// BUTTON INTERRUPT ///////
//	EICRA  |= (1<<ISC11);
//	EICRA &= ~(1<<ISC10);
//	EIMSK |= (1<<INT1);

	EICRA  |= (1<<ISC01);
	EICRA &= ~(1<<ISC00);
	EIMSK |= (1<<INT0);
	DDRD = 0b00;
	PORTD = 0b11;
	
//////// CENTISECONDS SETUP /////////
	TCCR1A = 0x0;            // no output
	TCCR1B |= 0b1101;         // CTC, 1024 prescaler
	OCR1A = 16000000L/1024L/100; // When reached, 1cs has passed
	// Clear timer on compare for Output Compare A Match Register (OCR1A)
	TIMSK1 = (1<<OCIE1A);

/////// PWM SETUP ////////////
	uint16_t pwm_period = 16000000UL / 1 / 10000 / 2;

	// Configure timer3 to be in mode 10 (spread across TCCR3{A,B})
	TCCR3A = 0b10100010; // OC3A & OC3B Outputting
	TCCR3B = 0b00010001; // 1 Prescaler
	ICR3 = pwm_period; // Set the top value for timer3
	DDRE = 0b00011000; // PWM pins are outputting
	OCR3A = (uint16_t)pwm_period * 8 / 10; // Initially 80% duty cycle
	OCR3B = (uint16_t)pwm_period * 2 / 3;  // Initially 66% duty cycle

////////// MOTOR CONTROL PORT A SETUP //////////
	DDRA  = 0b00001111; // Set the motor control pins to output
	PORTA = 0b00000101; // only 1 set at a time! (use setMotorDirection)

	sei();
	adc_init();
	lcd_init();
	serial0_init();
	
	while(1)
	{
		char buffer[32];

		// sensor readings
		
		int16_t left_sensor = side_to_mm(adc_read(4)) - SIDE_OFFSET;
		int16_t center_sensor = side_to_mm(adc_read(5)) - CENTER_OFFSET;
		int16_t right_sensor = side_to_mm(adc_read(6)) - SIDE_OFFSET;

		lcd_goto(0);
		sprintf(buffer, "L:%4dmm R:%4dmm", left_sensor, right_sensor);
		lcd_puts(buffer);
		lcd_goto(0x40);
		sprintf(buffer, "C: %6dmm", center_sensor);
		lcd_puts(buffer);

		if(interrupt0) // manual mode
		{
			int16_t adc0 = adc_read(0) - (1024/2); // adc0 = [-512, 511]
			adc0 *= -1;
			int16_t adc1 = adc_read(1) - (1024/2); // adc1 = [-512, 511]

			transform(&adc0, &adc1); // adc{0,1} = [-512, 512], but rotated 45 deg and scaled appropriately

			setMotorDirection(MOTOR1, adc0 < 0); // set motor direction to the sign of adc0
			setMotorDirection(MOTOR2, adc1 < 0); // set motor direction to the sign of adc1

			adc0 = abs(adc0); // Now we can make them positive
			adc1 = abs(adc1); // as we've extracted the sign bit.

			// transform the range [0, 512] into [0, pwm_period]
			adc0 = squish(adc0, 0.0f, 512.0f, 0.0f, pwm_period);
			adc1 = squish(adc1, 0.0f, 512.0f, 0.0f, pwm_period);

			OCR3A = adc0;
			OCR3B = adc1;
		}
		else // autonomous mode
		{

			static enum state { 
				FOLLOW_LEFT_WALL = 0, 
				FOLLOW_RIGHT_WALL,
				TURNING_LEFT,
				TURNING_RIGHT,
				STATIONARY,
				REVERSING
			} current_state = FOLLOW_LEFT_WALL;
			int16_t left_motor_speed;
			int16_t right_motor_speed;
			static const int16_t WALL_DISTANCE = 15;

			switch(current_state)
			{
				case FOLLOW_LEFT_WALL:
						left_motor_speed = 512;
						right_motor_speed =512;
						static const int16_t WALL_DISTANCE = 100;
						static const int16_t TOLERANCE = 30;

						if(left_sensor < WALL_DISTANCE)
							right_motor_speed /= (WALL_DISTANCE - left_sensor);
						else if(left_sensor > WALL_DISTANCE + TOLERANCE)
							left_motor_speed /= (left_sensor - WALL_DISTANCE - TOLERANCE);

						setMotorDirection(MOTOR1, false); // set motor direction to the sign of adc0
						setMotorDirection(MOTOR2, false); // set motor direction to the sign of adc1
						OCR3B = squish(left_motor_speed, 0.0f, 512.0f, 0.0f, pwm_period);
						OCR3A = squish(right_motor_speed, 0.0f, 512.0f, 0.0f, pwm_period);

						if(center_sensor < WALL_DISTANCE)
							current_state = TURNING_RIGHT;

					break;
				case FOLLOW_RIGHT_WALL:
					break;
				case TURNING_LEFT:
						
					break;
				case TURNING_RIGHT:
						left_motor_speed = 512;
						right_motor_speed = 0;

						setMotorDirection(MOTOR1, false); // set motor direction to the sign of adc0
						setMotorDirection(MOTOR2, false); // set motor direction to the sign of adc1
						OCR3B = squish(left_motor_speed, 0.0f, 512.0f, 0.0f, pwm_period);
						OCR3A = squish(right_motor_speed, 0.0f, 512.0f, 0.0f, pwm_period);
						if(center_sensor > WALL_DISTANCE)
							current_state = FOLLOW_LEFT_WALL;
					break;

			}

		}

	}
}
