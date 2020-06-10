#ifndef UTIL_H
#define UTIL_H
#include <stdlib.h>
#include <inttypes.h>
#define ROOT2 1.4142135623f

extern char buffer[64]; // Large buffer that can be used for formatting strings before writing to serial.

int16_t squish(int16_t value, float current_min, float current_max, float new_min, float new_max);
void transform(int16_t* restrict int_a, int16_t* restrict int_b);

int16_t rolling_average(int16_t new_point, int64_t* prev_sum, int16_t* prev_index,size_t point_count, int16_t points[]);

#endif

