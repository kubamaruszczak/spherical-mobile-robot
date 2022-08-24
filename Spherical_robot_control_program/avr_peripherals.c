/*
 * avr_peripherals.c
 *
 * Created: 18.01.2022 10:29:42
 *  Author: kubam
 */ 

#include <avr/io.h>
#include "avr_peripherals.h"

void timer1_100ms_init(void)	// Timer initialization function
{
	// Timera 16-bitowego in CTC mode
	TCCR1B |= (1<<WGM12);	// CTC mode
	TCCR1B |= (1<<CS12);	// prescaler = 256
	OCR1A = 6249;			// interrupt every 100 ms
	TIMSK1 |= (1<<OCIE1A);	// enabling interrupt on compare match
}

void adc_init(void)	// ADC converter initialization
{
	ADMUX |= (1<<REFS0);							// VREF = VCC = 5 V
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	// prescaler = 128 -> 16MHz / 128 = 125kHz
	ADCSRA |= (1<<ADEN);							// enabling ADC and interrupts
	DIDR0 |= (1<<ADC0D);							// block digital functions of pin 0
}

uint16_t adc_getValue(uint8_t channel)	// make measurement on particular channel
{
	ADMUX = (ADMUX & 0xF0) | channel;
	ADCSRA |= (1<<ADSC);				// start conversion
	while(ADCSRA & (1<<ADSC));			// wait for the end of conversion
	return ADC;
}
