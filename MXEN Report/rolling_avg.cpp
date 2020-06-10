#include <iostream>

int16_t rolling_average(int16_t new_point, int16_t* prev_sum, int16_t* prev_index, size_t point_count, int16_t points[])
{
	*prev_sum -= points[*prev_index];
	*prev_sum += new_point;
	points[*prev_index] = new_point;
	(*prev_index) = (*prev_index + 1) % point_count;
	return *prev_sum / point_count;
}

int main(void)
{
	int16_t value;
	while(true)
	{
		static const size_t MOVING_AVG_POINTS = 3;
		static int16_t l_prev_sum = 0, f_prev_sum = 0, r_prev_sum = 0;
		static int16_t l_prev_i = 0, f_prev_i = 0, r_prev_i = 0;
		static int16_t l_points[MOVING_AVG_POINTS], f_points[MOVING_AVG_POINTS], r_points[MOVING_AVG_POINTS];
		
		int16_t left_sensor; 
		std::cin >> left_sensor;
		left_sensor = rolling_average(left_sensor, &l_prev_sum, &l_prev_i, MOVING_AVG_POINTS, l_points);
		std::cout << "Moving average: " << left_sensor << std::endl;
	}
}
