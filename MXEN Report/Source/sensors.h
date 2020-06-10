#ifndef SENSORS_H
#define SENSORS_H
#include <inttypes.h>

int16_t front_to_mm(int16_t adc);
int16_t side_to_mm(int16_t adc);

#endif

