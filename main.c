#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#include "hd44780.h"
#include "serial.h"
#include "adc.h"

#include "servo.h"
#include "util.h" // squish, transform rolling_average and a string buffer
#include "sensors.h"  // transformations from adc to distance in mm
#include "motor.h"   // defines L_MOTOR, R_MOTOR, FORWARD, BACKWARD and set_motor_dir
#include "autonomous.h" // sensors states enum and functions to print
#include "centiseconds.h" // global centisecond timer using TIMER1

#define MOVING_AVG_POINTS 30

// Distance from sensor to edge of robot in mm
const int16_t CENTER_OFFSET = 87;
const int16_t SIDE_OFFSET = 37;


volatile bool interrupt0 = true;
ISR(INT0_vect)
{	
	// static because we want it to keep its state.
	static volatile uint32_t INT0_invalid_to = 0;

 	// Avoid reading a volatile multiple times
	uint32_t centi = centiseconds;
	if(INT0_invalid_to < centi)
	{
		interrupt0 = !interrupt0;     // toggle the interrupt0 boolean
		INT0_invalid_to = centi + 30; // debounce for 0.3s
	}
}

volatile bool interrupt1 = true;
ISR(INT1_vect)
{	
	// static because we want it to keep its state.
	static volatile uint32_t INT1_invalid_to = 0;

 	// Avoid reading a volatile multiple times
	uint32_t centi = centiseconds;
	if(INT1_invalid_to < centi)
	{
		interrupt1 = !interrupt1;     // toggle the interrupt1 boolean
		INT1_invalid_to = centi + 30; // debounce for 0.3s
	}
}

void manual_tick()
{
	int16_t adc0 = adc_read(11) - (1024/2); // adc0 = [-512, 511]
	adc0 *= 1;
	int16_t adc1 = adc_read(12) - (1024/2); // adc1 = [-512, 511]
	adc1 *= -1;

	transform(&adc0, &adc1); // adc{0,1} = [-512, 512], but rotated 45 deg and scaled appropriately

	set_motor_dir(L_MOTOR, adc0 < 0); // set motor direction to the sign of adc0
	set_motor_dir(R_MOTOR, adc1 < 0); // set motor direction to the sign of adc1

	adc0 = abs(adc0); // Now we can make them positive
	adc1 = abs(adc1); // as we've extracted the sign bit.

	// transform the range [0, 512] into [0, MOTOR_PWM_PERIOD]
	adc0 = squish(adc0, 0.0f, 512.0f, 0.0f, MOTOR_PWM_PERIOD);
	adc1 = squish(adc1, 0.0f, 512.0f, 0.0f, MOTOR_PWM_PERIOD);

	L_MOTOR_SPEED = adc0;
	R_MOTOR_SPEED = adc1;
}


int main(void)
{

	///// BUTTON INTERRUPT ///////
	EICRA  |= (1<<ISC11);
	EICRA &= ~(1<<ISC10);
	EIMSK |= (1<<INT1);

	EICRA  |= (1<<ISC01);
	EICRA &= ~(1<<ISC00);
	EIMSK |= (1<<INT0);

	DDRD = 0b00;
	PORTD = 0b11;
	
	motor_init();   // Sets up pins for controlling the H-Bridge and PWM (TIMER3)
	centiseconds_init();  // Global timer named 'centiseconds' (Using TIMER4)
	adc_init();           // Initialise the analog to digital converter
	lcd_init();           // For displaying sensor readings on the LCD screen
	serial0_init();       // For writing to the serial monitor.
	servo_init();         // Sets up pins for controlling the servo motor (Using TIMER1)
	sei();                // Globally enable interrupts
	
	while(1)
	{
		if(interrupt1)
		{
			static int64_t invalid_to = 0;
			int64_t centi = centiseconds;
			if(centi > invalid_to)
			{
				servo_pos(squish(adc_read(14), 0, 1024, 0, 180));
				invalid_to = centi + 10;
			}
		}
		else
		{
			servo_off();
		}

		// Set joystick position
		
		// Variables for keeping track of the rolling average of the 3 range sensors.
		// Static because we want them to maintain their state.
		static int64_t l_prev_sum = 0, f_prev_sum = 0, r_prev_sum = 0;
		static int16_t l_prev_i = 0, f_prev_i = 0, r_prev_i = 0;
		static int16_t l_points[MOVING_AVG_POINTS], f_points[MOVING_AVG_POINTS], r_points[MOVING_AVG_POINTS];

		// Read data from ADC and convert it to mm.
		// Translate it so that it is distance from the edge of the robot.
		int16_t left_sensor  = side_to_mm( adc_read(0)) - SIDE_OFFSET;
		int16_t front_sensor = front_to_mm(adc_read(1)) - CENTER_OFFSET;
		int16_t right_sensor = side_to_mm( adc_read(2)) - SIDE_OFFSET;

		// Add the reading to the rolling average circular buffer and reassign left_sensor to be a de-noised value
		left_sensor  = rolling_average(left_sensor,  &l_prev_sum, &l_prev_i, MOVING_AVG_POINTS, l_points);
		right_sensor = rolling_average(right_sensor, &r_prev_sum, &r_prev_i, MOVING_AVG_POINTS, r_points);
		front_sensor = rolling_average(front_sensor, &f_prev_sum, &f_prev_i, MOVING_AVG_POINTS, f_points);

		static int64_t update_invalid_to = 0; // static to retain state
		if(centiseconds > update_invalid_to) // only update the display every 4 seconds
		{
			lcd_goto(0);
			sprintf(buffer, "L:%4dmmR:%4dmm", left_sensor, right_sensor);
			lcd_puts(buffer);
			lcd_goto(0x40);
			sprintf(buffer, "C: %6dmm", front_sensor);
			lcd_puts(buffer);
			update_invalid_to = centiseconds + 25; 
		}
		if(interrupt0) // manual mode
		{
			manual_tick();
		}
		else // autonomous mode
		{
			autonomous_tick(left_sensor, front_sensor, right_sensor);
		}
	}
}

