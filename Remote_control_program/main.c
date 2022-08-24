/*
 * 4.13_JOYSTICK.c
 *
 * Created: 28.11.2021 15:37:44
 * Author : kubam
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "usart_328p.h"
#include "nrf24l01.h"
#include "nrf24l01-mnemonics.h"
#include "spi.h"

#define X_CHANNEL 4
#define Y_CHANNEL 5

void ADC_init(void);			
uint16_t ADC_getValue(uint8_t channel);

int main(void)
{
	// UART buffer
	char text[20];
	uint16_t potValueX, potValueY;
	// NRF24L01+ buffer
	uint8_t data[2];

	// initialize communication
	usart_init(9600);
	nrf24_init();
	ADC_init();
	
    while (1) 
    {
		// ADC measurement
		potValueX = ADC_getValue(X_CHANNEL);
		potValueY = ADC_getValue(Y_CHANNEL);
		
		data[0] = (uint8_t)(potValueX * 180.0 / 1023.0);
		data[1] = (uint8_t)(potValueY * 180.0 / 1023.0);
		
		sprintf(text, "%d %d", data[0], data[1]);
		nrf24_send_message(text); // send measurements via radio	
		_delay_ms(50);		
    }
}

void ADC_init(void)
{
	ADMUX |= (1<<REFS0);							// VREF = VCC = 5 V
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	// prescaler = 128 -> 16MHz / 128 = 125kHz
	ADCSRA |= (1<<ADEN);							// enabling ADC and interrupts
}

uint16_t ADC_getValue(uint8_t channel)
{
	ADMUX = (ADMUX & 0xF0) | channel;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADC; 
}
