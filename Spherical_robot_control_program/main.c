/*
 * Robot_kula.c
 *
 * Created: 18.01.2022 09:54:14
 * Author : Jakub Maruszczak
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// --------------- ACCUMULATOR ---------------
#define LIPO_ADC_CHANNEL 0
#define MIN_LIPO_VOLTAGE 7.0

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include "avr_peripherals.h"
#include "nrf24l01.h"
#include "nrf24l01-mnemonics.h"
#include "spi.h"
#include "drv8833_md.h"
#include "i2c.h"
#include "mpu6050.h"

// debug
#include "usart_328p.h"
#include <stdlib.h>

uint8_t mapValue(int toMap, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);	// map value function

double liPoVoltege;	// liPoVoltage function

int main(void)
{
	int joystickX, joystickY;
    int64_t motorM1_targetPos = 0, motorM2_targetPos = 0;
	
    // initialize timers and ADC
	timer1_100ms_init();
	adc_init();
	
	// initialize communication
	i2c_init();
	usart_init(9600);
	
	// initialize radio, motors and MPU-6050
	nrf24_init();
	motors_init();
	mpu6050_init();
	
	sei();	// enable interrupts
	
	mpu6050_calibrateGyro();	// gyro calibration
	nrf24_start_listening();	// put radio in listening mode
	
    while (1) 
    {			
		// UART variables
		char buffer[20], int_[10];
		
		liPoVoltege = adc_getValue(LIPO_ADC_CHANNEL) * 5.0 / 1023.0 * 2;	// check LiPo voltage
		if(liPoVoltege > MIN_LIPO_VOLTAGE)
		{
			if(nrf24_available())	// if radio data received
				sscanf(nrf24_read_message(), "%d %d", &joystickY, &joystickX);	// read data
			
			mpu6050_getAccelGryoValues();	// read MPU-6050 values
			
			if(joystickY >= 110 && (joystickX > 70 && joystickX < 110))
			{
				if(roll > -35)
				{
					motorM1_targetPos -= mapValue(joystickY, 110, 180, 0, 30);
					motorM2_targetPos -= mapValue(joystickY, 110, 180, 0, 30);
				}
				else
				{
					motorM1_targetPos += STABILIZATION;
					motorM2_targetPos += STABILIZATION;
				}
			}
			else if(joystickY <= 70 && (joystickX > 70 && joystickX < 110))
			{
				if(roll < 35)
				{
					motorM1_targetPos += mapValue(joystickY, 70, 0, 0, 30);
					motorM2_targetPos += mapValue(joystickY, 70, 0, 0, 30);
				}
				else
				{
					motorM1_targetPos -= STABILIZATION;
					motorM2_targetPos -= STABILIZATION;
				}
			}
			else if(joystickX >= 110 && (joystickY > 70 && joystickY < 110))
			{
				motorM1_targetPos -= mapValue(joystickX, 110, 180, 0, 30);
				motorM2_targetPos += mapValue(joystickX, 110, 180, 0, 30);
			}
			else if(joystickX <= 70 && (joystickY > 70 && joystickY < 110))
			{
				motorM1_targetPos += mapValue(joystickX, 70, 0, 0, 30);
				motorM2_targetPos -= mapValue(joystickX, 70, 0, 0, 30);
			}
			
			// PID algorithm for both motors
			motor_PID(1, motorM1_targetPos, 0.09, 0.005, 0.012);
			motor_PID(2, motorM2_targetPos, 0.07, 0.005, 0.012);
			
			// debug
			dtostrf(pos_M1, 5, 0, int_);
			sprintf(buffer, "%s\t", int_);
			usart_puts(buffer);
			
			dtostrf(motorM1_targetPos, 5, 0, int_);
			sprintf(buffer, "%s\t", int_);
			usart_puts(buffer);
			
			dtostrf(pos_M2, 5, 0, int_);
			sprintf(buffer, "%s\t", int_);
			usart_puts(buffer);
						
			dtostrf(motorM2_targetPos, 5, 0, int_);
			sprintf(buffer, "%s\r\n", int_);
			usart_puts(buffer);
			// end debug
		}
		else
		{
			motorM1_pwm(1, 0);
			motorM2_pwm(1, 0);
		}
    }
}

uint8_t mapValue(int toMap, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
	return (uint8_t)((toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

ISR(TIMER1_COMPA_vect)
{
	static int ePrev_M1, ePrev_M2;
	
	// motor M1 PID
	e_M1 = targetPos_M1 - pos_M1;
	eDerivative_M1 = (e_M1 - ePrev_M1) / dt;
	eIntegral_M1 += e_M1 * dt;
	ePrev_M1 = e_M1;
	
	// motor M2 PID
	e_M2 = targetPos_M2 - pos_M2;
	eDerivative_M2 = (e_M2 - ePrev_M2) / dt;
	eIntegral_M2 += e_M2 * dt;
	ePrev_M2 = e_M2;
	
	mpu6050_getRollPitch();						// calculate roll and pitch
}
