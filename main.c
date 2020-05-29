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
#define MOVING_AVG_POINTS 30

#define FULL_SPEED 512
#define HALF_SPEED 448
#define SLOW_SPEED 384
#define STOP_SPEED 256

#define SIGNIFICANT_READING_MM 300
#define SIGNIFICANT_READING_MM_SIDE 200
#define FRONT_CRITICAL_DIST 160
#define SIDE_CRITICAL_DIST 80
#define TOLERANCE 30
// Distance from sensor to edge of robot in mm
const int16_t CENTER_OFFSET = 87;
const int16_t SIDE_OFFSET = 37;

volatile uint32_t centiseconds_global = 0;
ISR(TIMER1_COMPA_vect)
{ // Timer tick. Increment centiseconds by 1.
	centiseconds_global += 1;
}

volatile bool interrupt0 = true;
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

int16_t rolling_average(int16_t new_point, int64_t* prev_sum, int16_t* prev_index, size_t point_count, int16_t points[])
{
	*prev_sum -= points[*prev_index];
	*prev_sum += new_point;
	points[*prev_index] = new_point;
	(*prev_index) = (*prev_index + 1) % point_count;
	return *prev_sum / point_count;
}

int16_t front_to_mm(int16_t adc) { return -(53227500ll*adc-268558095607ll)/(2000*(2500ll*adc-78227)); }
int16_t side_to_mm(int16_t adc)  { return -(61377500ll*adc - 110758264243ll)/(1250ll*(2500ll*adc+68970)); }

int main(void)
{

	///// BUTTON INTERRUPT ///////
	//	EICRA  |= (1<<ISC11);
	//	EICRA &= ~(1<<ISC10
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
		static int64_t l_prev_sum = 0, f_prev_sum = 0, r_prev_sum = 0;
		static int16_t l_prev_i = 0, f_prev_i = 0, r_prev_i = 0;
		static int16_t l_points[MOVING_AVG_POINTS], f_points[MOVING_AVG_POINTS], r_points[MOVING_AVG_POINTS];

		static int16_t left_sensor	 = 0;
		static int16_t front_sensor = 0;
		static int16_t right_sensor = 0;

		left_sensor  = side_to_mm(adc_read(4)) - SIDE_OFFSET;
		front_sensor = front_to_mm(adc_read(5)) - CENTER_OFFSET;
		right_sensor = side_to_mm(adc_read(6)) - SIDE_OFFSET;

		static int64_t sensor_read_invalid_to = 0;
		if(centiseconds_global > sensor_read_invalid_to)
		{
			left_sensor  = side_to_mm(adc_read(4)) - SIDE_OFFSET;
			front_sensor = front_to_mm(adc_read(5)) - CENTER_OFFSET;
			right_sensor = side_to_mm(adc_read(6)) - SIDE_OFFSET;

			left_sensor  = rolling_average(left_sensor,  &l_prev_sum, &l_prev_i, MOVING_AVG_POINTS, l_points);
			right_sensor = rolling_average(right_sensor, &r_prev_sum, &r_prev_i, MOVING_AVG_POINTS, r_points);
			front_sensor = rolling_average(front_sensor, &f_prev_sum, &f_prev_i, MOVING_AVG_POINTS, f_points);
			sensor_read_invalid_to = centiseconds_global;
		}
		
		static int64_t update_invalid_to = 0;
		if(centiseconds_global > update_invalid_to) // update current state 4times a second
		{
			lcd_goto(0);
			sprintf(buffer, "L:%4dmmR:%4dmm", left_sensor, right_sensor);
			lcd_puts(buffer);
			lcd_goto(0x40);
			sprintf(buffer, "C: %6dmm", front_sensor);
			lcd_puts(buffer);
			update_invalid_to = centiseconds_global + 25;
		}
		
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
			static enum sensor_states
			{
				OPEN_FORWARD = 0b000,
				FOLLOW_LEFT = 0b100,
				FOLLOW_RIGHT = 0b001,
				APPROACH_FRONT = 0b010,
				CAUTION_FRONT_LEFT = 0b110,
				CAUTION_FRONT_RIGHT = 0b011,
				CAUTION_LEFT_RIGHT = 0b101,
				DEAD_END = 0b111
			} current_state, old_state;
			int16_t left_speed = FULL_SPEED;
			int16_t right_speed = FULL_SPEED;

			bool left_significant =   left_sensor < SIGNIFICANT_READING_MM_SIDE;
			bool right_significant = right_sensor < SIGNIFICANT_READING_MM_SIDE;
			bool front_significant = front_sensor < SIGNIFICANT_READING_MM;
			static int64_t state_invalid_to = 0;
			if(centiseconds_global > state_invalid_to) // update current state 4times a second
			{
				current_state = (left_significant << 2) | (front_significant << 1) | (right_significant << 0);
				if(current_state != old_state)
				{
					switch(current_state)
					{
						case OPEN_FORWARD:
						serial0_print_string("OPEN_FORWARD\n"); break;
						case FOLLOW_LEFT:
						serial0_print_string("FOLLOW_LEFT\n"); break;
						case FOLLOW_RIGHT:
						serial0_print_string("FOLLOW_RIGHT\n"); break;
						case APPROACH_FRONT:
						serial0_print_string("APPROACH_FRONT\n"); break;
						case CAUTION_FRONT_LEFT:
						serial0_print_string("CAUTION_FRONT_LEFT\n"); break;
						case CAUTION_FRONT_RIGHT:
						serial0_print_string("CAUTION_FRONT_RIGHT\n"); break;
						case CAUTION_LEFT_RIGHT:
						serial0_print_string("CAUTION_LEFT_RIGHT\n"); break;
						case DEAD_END:
						serial0_print_string("DEAD_END\n"); break;
						default:
						serial0_print_string("default\n"); break;
					}
				}
				state_invalid_to = centiseconds_global + 25;
			}
			switch(current_state)
			{
				case OPEN_FORWARD:
				left_speed = FULL_SPEED;
				right_speed = FULL_SPEED;
				break;
				
				case FOLLOW_LEFT:
				left_speed  = FULL_SPEED;
				right_speed = FULL_SPEED;

				if(left_sensor < SIDE_CRITICAL_DIST)
					right_speed = STOP_SPEED;
				else if(left_sensor > SIDE_CRITICAL_DIST + TOLERANCE)
					left_speed = STOP_SPEED;
				break;
				
				case FOLLOW_RIGHT:
				left_speed  = FULL_SPEED;
				right_speed = FULL_SPEED;
				
				if(right_sensor < SIDE_CRITICAL_DIST)
					left_speed = STOP_SPEED;
				else if(right_sensor > SIDE_CRITICAL_DIST + TOLERANCE)
					right_speed = STOP_SPEED;
				break;
				
				case APPROACH_FRONT:
				if(front_sensor < FRONT_CRITICAL_DIST)
				{
					if(right_sensor > SIDE_CRITICAL_DIST * 3 && left_sensor > SIDE_CRITICAL_DIST * 3)
					{
						left_speed = STOP_SPEED;
						right_speed = FULL_SPEED;
					}
					else
					{
						if(left_sensor>right_sensor + TOLERANCE)
						{
							left_speed = STOP_SPEED;
							right_speed = FULL_SPEED;
						}
						else
						{
							left_speed = STOP_SPEED;
							right_speed = FULL_SPEED;
						}
					}
				}
				else
				{
					left_speed  = HALF_SPEED;
					right_speed = HALF_SPEED;
				}
				break;
				
				case CAUTION_FRONT_LEFT:
				if(front_sensor < (FRONT_CRITICAL_DIST + TOLERANCE)  || left_sensor < (SIDE_CRITICAL_DIST + TOLERANCE))
				{
					left_speed  = FULL_SPEED;
					right_speed = FULL_SPEED;
					if(right_sensor < SIDE_CRITICAL_DIST)
						left_speed = STOP_SPEED;
					else if(right_sensor > SIDE_CRITICAL_DIST + TOLERANCE)
						right_speed = STOP_SPEED;
				}
				else
				{
					//left_speed /= front_sensor < left_sensor ? front_sensor - FRONT_CRITICAL_DIST : left_sensor - FRONT_CRITICAL_DIST;
					left_speed = FULL_SPEED;
					right_speed = left_speed / 2;
				}
				break;
				
				case CAUTION_FRONT_RIGHT:
				if(front_sensor < (FRONT_CRITICAL_DIST + TOLERANCE) || right_sensor < (SIDE_CRITICAL_DIST+TOLERANCE) )
				{
					left_speed  = FULL_SPEED;
					right_speed = FULL_SPEED;

					if(left_sensor < SIDE_CRITICAL_DIST)
						right_speed = STOP_SPEED;
					else if(left_sensor > SIDE_CRITICAL_DIST + TOLERANCE)
						left_speed = STOP_SPEED;
				}
				else
				{
					//right_speed /= front_sensor < right_sensor ? front_sensor - FRONT_CRITICAL_DIST : right_sensor - FRONT_CRITICAL_DIST;
					right_speed = FULL_SPEED;
				}
				left_speed = right_speed / 2;
				break;
				
				case CAUTION_LEFT_RIGHT:
				if(left_sensor > right_sensor + TOLERANCE)
				{
					left_speed = STOP_SPEED;
					right_speed = FULL_SPEED;
				}
				else if (right_sensor > left_sensor + TOLERANCE)
				{
					right_speed = STOP_SPEED;
					left_speed = FULL_SPEED;
				}
				else
				{
					left_speed = FULL_SPEED;
					right_speed = FULL_SPEED;
				}
				break;
				
				case DEAD_END:
				setMotorDirection(MOTOR1, true);
				setMotorDirection(MOTOR2, true);
				OCR3B = squish(left_speed, 0.0f, 512.0f, 0.0f, pwm_period);
				OCR3A = squish(right_speed, 0.0f, 512.0f, 0.0f, pwm_period);
				break;
			}
			setMotorDirection(MOTOR1, false);
			setMotorDirection(MOTOR2, false);
			OCR3B = squish(left_speed, 0.0f, 512.0f, 0.0f, pwm_period);
			OCR3A = squish(right_speed, 0.0f, 512.0f, 0.0f, pwm_period);
			static int64_t speed_invalid_to = 0;
			old_state = current_state;
			if(centiseconds_global > speed_invalid_to) // update current state 4times a second
			{
				sprintf(buffer, "l: %4d, r: %4d\n", left_speed, right_speed);
				serial0_print_string(buffer);
				speed_invalid_to = centiseconds_global + 25;
			}
		}
	}
}
