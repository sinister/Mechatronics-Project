#include <avr/io.h>
#include <stdio.h>

#include "autonomous.h"
#include "util.h"
#include "centiseconds.h"
#include "motor.h"
#include "serial.h"

#define SIGNIFICANT_READING_MM 300
#define SIGNIFICANT_READING_MM_SIDE 300
#define FRONT_CRITICAL_DIST 160
#define SIDE_CRITICAL_DIST 80
#define TOLERANCE 30

// Used for debugging.
void print_state(enum sensor_state state)
{
	switch(state)
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

void follow_left(int16_t* left_speed, int16_t* right_speed, int16_t left_sensor, int16_t right_sensor)
{
	*left_speed = *right_speed = FULL_SPEED;

	if(left_sensor < SIDE_CRITICAL_DIST) // if we are a bit close, stop the right wheel
		*right_speed = STOP_SPEED;
	else if(left_sensor > SIDE_CRITICAL_DIST + TOLERANCE) // if we are a bit far, stop the left wheel
		*left_speed = STOP_SPEED;
}


void caution_front_left(int16_t* left_speed, int16_t* right_speed, int16_t left_sensor, int16_t front_sensor, int16_t right_sensor)
{
	if(front_sensor < (FRONT_CRITICAL_DIST + TOLERANCE)  || left_sensor < (SIDE_CRITICAL_DIST + TOLERANCE))
	{
		*left_speed = *right_speed = FULL_SPEED;
		if(right_sensor < SIDE_CRITICAL_DIST)
			*left_speed = STOP_SPEED;
		else if(right_sensor > SIDE_CRITICAL_DIST + TOLERANCE)
			*right_speed = STOP_SPEED;
	}
	else
	{
		*left_speed = FULL_SPEED;
		*right_speed = STOP_SPEED;
	}
}

void autonomous_tick(int16_t left_sensor, int16_t front_sensor, int16_t right_sensor)
{
	static enum sensor_state current_state, old_state;
	int16_t left_speed = FULL_SPEED;
	int16_t right_speed = FULL_SPEED;

	bool left_significant =   left_sensor < SIGNIFICANT_READING_MM_SIDE;
	bool right_significant = right_sensor < SIGNIFICANT_READING_MM_SIDE;
	bool front_significant = front_sensor < SIGNIFICANT_READING_MM;
	static int64_t state_invalid_to = 0;
	if(centiseconds > state_invalid_to) // update current state 4times a second
	{
		current_state = (left_significant << 2) | (front_significant << 1) | (right_significant << 0);
		if(current_state != old_state)
		{
			print_state(current_state);
		}
		state_invalid_to = centiseconds + 25;
	}
	switch(current_state)
	{
		case OPEN_FORWARD: // there is nothing in the way
			left_speed = right_speed = FULL_SPEED;
			break;
		
		case FOLLOW_LEFT: // we want to follow the left wall
			follow_left(&left_speed, &right_speed, left_sensor, right_sensor);
			break;
		
		case FOLLOW_RIGHT: // following right is the same as left, but reflected
			follow_left(&right_speed, &left_speed, right_sensor, left_sensor);
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
				left_speed = right_speed = HALF_SPEED;
			}
			break;
		
		case CAUTION_FRONT_LEFT:
			// There is something to the front and left, so we must be careful as we get closer
//			caution_front_left(&left_speed, &right_speed, left_sensor, front_sensor, right_sensor);
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
			// same as CAUTION_FRONT_LEFT, but reflected
//			caution_front_left(&right_speed, &left_speed, right_sensor, front_sensor, left_sensor);
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
				left_speed = right_speed = FULL_SPEED;
			}
			break;
		
		case DEAD_END:
			set_motor_dir(L_MOTOR, BACKWARD);
			set_motor_dir(R_MOTOR, BACKWARD);
			L_MOTOR_SPEED = squish(left_speed, 0.0f,  FULL_SPEED, 0.0f, MOTOR_PWM_PERIOD);
			R_MOTOR_SPEED = squish(right_speed, 0.0f, FULL_SPEED, 0.0f, MOTOR_PWM_PERIOD);
			break;
	}

	set_motor_dir(L_MOTOR, FORWARD);
	set_motor_dir(R_MOTOR, FORWARD);
	L_MOTOR_SPEED = squish(left_speed, 0.0f,  FULL_SPEED, 0.0f, MOTOR_PWM_PERIOD);
	R_MOTOR_SPEED = squish(right_speed, 0.0f, FULL_SPEED, 0.0f, MOTOR_PWM_PERIOD);
	static int64_t speed_invalid_to = 0;
	old_state = current_state;
	if(centiseconds > speed_invalid_to) // update current state 4times a second
	{
		sprintf(buffer, "l: %4d, r: %4d\n", left_speed, right_speed);
		serial0_print_string(buffer);
		speed_invalid_to = centiseconds + 25;
	}

}
