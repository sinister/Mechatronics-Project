#include "sensors.h"

int16_t front_to_mm(int16_t adc) 
{ 
	// Pretty much just magic numbers. Formula comes from the curve of best fit for the
	// sensor readings at different distances. Check sensor data for justification.
	return -(53227500ll*adc-268558095607ll)/(2000*(2500ll*adc-78227)); 
}
int16_t side_to_mm(int16_t adc)  
{ 
	// Same as front_to_mm, formula comes from a curve of best fit made using data from
	// sensors readings. See data for justification.
	return -(61377500ll*adc - 110758264243ll)/(1250ll*(2500ll*adc+68970)); 
}
