#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H
#include <inttypes.h>


enum sensor_state
{
	OPEN_FORWARD = 0b000,        // nothing anywhere. Just send it
	FOLLOW_LEFT = 0b100,         // Something on the left, nothing in front or on the right
	FOLLOW_RIGHT = 0b001,        // Something on the right, nothing in front or to the left
	APPROACH_FRONT = 0b010,      // Only something in front. Approach carefully and then turn left.
	CAUTION_FRONT_LEFT = 0b110,  // Something in front and to the left. Start turning right
	CAUTION_FRONT_RIGHT = 0b011, // Something in front and to the right. Start turning left.
	CAUTION_LEFT_RIGHT = 0b101,  // Something on either side of the robot. Stay in the center
	DEAD_END = 0b111             // Surrounded on all sides.
}; 

void print_state(enum sensor_state state);
void follow_left(int16_t* left_speed, int16_t* right_speed, int16_t left_sensor, int16_t right_sensor);
void caution_front_left(int16_t* left_speed, int16_t* right_speed, 
                        int16_t left_sensor, int16_t front_sensor, int16_t right_sensor);
void autonomous_tick(int16_t left_sensor, int16_t front_sensor, int16_t right_sensor);

#endif
