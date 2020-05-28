#define ROOT2 1.4142135623f

#define MOTOR1 0
#define MOTOR2 2


// Please note that this function only takes 16 bit integers, while the timer
// needs a 16 bit UNSIGNED integer. If you change the frequency so that it
// requires a greater timer range, you should change these parameters
int16_t squish(int16_t value, float current_min, float current_max, float new_min, float new_max)
{
	// the line passing through the points (current_min, new_min) and (current_max, new_max) can be 
	// expressed generally as y=mx+c where x is the current value and y is the new value. In matrix
	// form this could be expressed in the equation [1, current_min; 1, current_max][c; m] = [new_min, new_max].
	// Multiplying by the inverse gives 
	// c = (current_min*new_max - current_max * new_min)/(current_min - current_max)
	// m = (new_min - new_max)/(current_min - current_max)
	// Substituting these into the formula y=mx+c, we can easily find the new value y

	float c = (current_min*new_max - current_max*new_min)/(current_min - current_max);
	float m = (new_min - new_max)/(current_min - current_max);
	return m*value + c;
}

// Takes a pair of integers int_a + int_b*I and rotates
// them 45 degrees. It then scales the magnitude along the radius
// to make an output that, for a square input, will give a square output.
void transform(int16_t* restrict int_a, int16_t* restrict int_b)
{
	// convert int_a and int_b to be floating point 
	// numbers original a and original b
	const float oa = *int_a, ob = *int_b; 
	
	// ROTATE 45 DEGREES
	// a + b*i = (root2/2)*(1 + I) * (oa + ob*I)
	// Rotate the inputted value by 45 degrees (clockwise)
	// and store the result in the complex number a + b*I
	float a = (ROOT2 / 2.0f)*(oa - ob);
	float b = (ROOT2 / 2.0f)*(oa + ob);

	// CALCULATE THE SCALE FACTOR.
	float sf; // scale factor;
	// x & y are only used for solution simplification.
	// First Make a complex number x + y*I that is in the first octant 0 <= arg <= pi/4
	// fabs() makes the point in the first quadrant.
	float x = fabs(a), y = fabs(b); 
	// We then reflect if it is in the second octant
	// This allows us to exploit symmetry to only have to solve for 1 case.
	if( y > x ) { float temp = x; x = y; y = temp; }
	// set the scale factor (unless x is sufficiently small, to avoid divide by 0).
	sf = ( x < 0.0001 ? 1.0f : sqrt( (x+y)*(x+y)/(2*x*x) ) );
	// see ./Transformation/transformation equations.png for justification of formula

	// SCALE 
	a *= sf; // scale a by the scale factor
	b *= sf; // scale b by the scale factor

	// ASSIGN
	*int_a = a; // assign int_a its new value
	*int_b = b; // assign int_b its new value
	return;
}

int setMotorDirection(int lsb, bool forward)
{
	PORTA &= ~(0b11 << lsb); // Turn the motor off
	if(forward)
		PORTA |= (0b01 << lsb);
	else
		PORTA |= (0b10 << lsb);
}
