#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>

volatile uint32_t centiseconds = 0;

// Timer tick. Increment centiseconds by 1.
ISR(TIMER4_COMPA_vect)
{
	centiseconds += 1;
}

void centiseconds_init()
{
	TCCR4A = 0x0;            // no output
	TCCR4B |= 0b1101;         // CTC, 1024 prescaler
	OCR4A = 16000000L/1024L/100; // When reached, 1cs has passed
	// Clear timer on compare for Output Compare A Match Register (OCR4A)
	TIMSK4 = (1<<OCIE4A);
}
