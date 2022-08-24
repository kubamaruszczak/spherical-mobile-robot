/*
 * usart.c
 *
 * Created: 05.11.2021 16:11:54
 *  Author: kubam
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "usart_328p.h"		

void usart_init(uint32_t baud)
{
	// set baud rate
	uint32_t ubrr = (F_CPU/(16UL*baud))-1UL;
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;						
	UCSR0B |= (1<<RXEN0) | (1<<TXEN0);				// turn on rx and tx
	UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);			// 8 data bits, 1 stop bit, no parity control
}

char usart_getc(void) // send char
{
	while(!(UCSR0A & (1<<RXC0))); 
	return UDR0;
}

void usart_putc(char data)
{
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

void usart_puts(char *str)
{
	while( *str )
		usart_putc(*(str++));
}

void usart_putint(uint8_t value, uint8_t base)
{
	char str[16];
	itoa(value, str, base);
	usart_puts(str);
}
