/*
 * usart.h
 *
 * Created: 05.11.2021 16:11:38
 *  Author: kubam
 */ 


#ifndef USART_H_
#define USART_H_

#define F_CPU 16000000UL

void usart_init(uint32_t baud);					// initialize UART with given baud rate
char usart_getc(void);							// read char function
void usart_putc(char data);						// send char function
void usart_puts(char *str);						// send string function
void usart_putint(uint8_t value, uint8_t base);	// send int function

#endif /* USART_H_ */
