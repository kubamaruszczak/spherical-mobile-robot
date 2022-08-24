/*
 * avr_peripherals.h
 *
 * Created: 18.01.2022 10:29:19
 *  Author: kubam
 */ 


#ifndef AVR_PERIPHERALS_H_
#define AVR_PERIPHERALS_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// time stamp for PID algorithm
#define dt 0.1

void timer1_100ms_init(void);
void adc_init(void);
uint16_t adc_getValue(uint8_t channel);

#endif /* AVR_PERIPHERALS_H_ */
