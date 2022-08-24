/*
 * drv8833_md.c
 *
 * Created: 29.11.2021 08:42:47
 *  Author: kubam
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "drv8833_md.h"

// software PWM channels variables
volatile uint8_t pwm_A1, pwm_A2, pwm_B1, pwm_B2;

// motors position variables
volatile int64_t pos_M1, pos_M2;

// motor M1 PID variables
volatile double e_M1, eIntegral_M1, eDerivative_M1;
volatile int64_t targetPos_M1;
double controlSignal_M1;

// motor M2 PID variables
volatile double e_M2, eIntegral_M2, eDerivative_M2;
volatile int64_t targetPos_M2;
double controlSignal_M2;

double liPoVoltege;

void drv8833_init(void)
{
	// set PWM channels as outputs
	DDR_AIN1 |= (1<<AIN1); DDR_AIN2 |= (1<<AIN2);
	DDR_BIN1 |= (1<<BIN1); DDR_BIN2 |= (1<<BIN2);
	
	// Timera0 - NORMAL mode
	TCCR0B |= (1<<CS00);	// prescaler = 1
	TIMSK0 |= (1<<TOIE0);	// overflow interrupt enable
}

void motorM1_pwm(uint8_t forwardDir, uint8_t pwmDutyCycle)	// control motor M1 with given PWM %
{	
	if(forwardDir)
	{
		if(pwmDutyCycle <= 100) pwm_A1 = (uint8_t)(255 * pwmDutyCycle / 100.0);
		else pwm_A1 = 255;
		pwm_A2 = 0;
	}	
	else
	{
		pwm_A1 = 0;
		if(pwmDutyCycle <= 100) pwm_A2 = (uint8_t)(255 * pwmDutyCycle / 100.0);
		else pwm_A2 = 255;
	}
}

void motorM2_pwm(uint8_t forwardDir, uint8_t pwmDutyCycle)	// control motor M2 with given PWM %
{	
	if(!forwardDir)
	{
		if(pwmDutyCycle <= 100) pwm_B1 = (uint8_t)(255 * pwmDutyCycle / 100.0);
		else pwm_B1 = 255;
		pwm_B2 = 0;
	}
	else
	{
		pwm_B1 = 0;
		if(pwmDutyCycle <= 100) pwm_B2 = (uint8_t)(255 * pwmDutyCycle / 100.0);
		else pwm_B2 = 255;
	}
}

void encoders_init(void)
{
	// external interrupt configuration - INT0
	EICRA |= (1<<ISC01) | (1<<ISC00);	// rising edge
	EIMSK |= (1<<INT0);					// enabling interrupt on INT0 pin

	// external interrupt configuration - INT1
	EICRA |= (1<<ISC11) | (1<<ISC10);	// rising edge
	EIMSK |= (1<<INT1);					// enabling interrupt on INT1 pin
}

void motors_init(void)
{
	drv8833_init();
	encoders_init();
}

void motor_PID(uint8_t motor, int64_t targetPos, double kp, double ki, double kd)
{
	switch(motor)
	{
		int pwmPID;
		case 1:
			// PID motor M1
			targetPos_M1 = targetPos;
			controlSignal_M1 = kp * e_M1 + ki * eIntegral_M1 + kd * eDerivative_M1;
			if(controlSignal_M1 < 0)
			{
				pwmPID = controlSignal_M1 * (-1);
				if(pwmPID >= 60) motorM1_pwm(BACKWARD, 60);
				else motorM1_pwm(BACKWARD, (uint8_t)pwmPID);
			}
			else
			{
				pwmPID = controlSignal_M1;
				if(pwmPID >= 60) motorM1_pwm(FORWARD, 60);
				else motorM1_pwm(FORWARD, (uint8_t)pwmPID);
			}
		
		case 2:
			// PID motor M2
			targetPos_M2 = targetPos;
			controlSignal_M2 = kp * e_M2 + ki * eIntegral_M2 + kd * eDerivative_M2;
			if(controlSignal_M2 < 0)
			{
				pwmPID = controlSignal_M2 * (-1);
				if(pwmPID >= 60) motorM2_pwm(BACKWARD, 60);
				else motorM2_pwm(BACKWARD, (uint8_t)pwmPID);
			}
			else
			{
				pwmPID = controlSignal_M2;
				if(pwmPID >= 60) motorM2_pwm(FORWARD, 60);
				else motorM2_pwm(FORWARD, (uint8_t)pwmPID);
			}
	}
}

ISR(TIMER0_OVF_vect)
{
	static uint8_t cnt;
	if(cnt >= pwm_A1) PORT_AIN1 &= ~(1<<AIN1); else PORT_AIN1 |= (1<<AIN1);
	if(cnt >= pwm_A2) PORT_AIN2 &= ~(1<<AIN2); else PORT_AIN2 |= (1<<AIN2); 
	if(cnt >= pwm_B1) PORT_BIN1 &= ~(1<<BIN1); else PORT_BIN1 |= (1<<BIN1); 
	if(cnt >= pwm_B2) PORT_BIN2 &= ~(1<<BIN2); else PORT_BIN2 |= (1<<BIN2); 
	cnt++;
}

ISR(INT0_vect)
{
	ADCSRA &= ~(1<<ADEN);
	if(PIN_ENCB_M1 & (1<<ENCB_M1)) pos_M1++; else pos_M1--;
	ADCSRA |= (1<<ADEN);
}

ISR(INT1_vect)
{
	ADCSRA &= ~(1<<ADEN);
	if(PIN_ENCB_M2 & (1<<ENCB_M2)) pos_M2--; else pos_M2++;
	ADCSRA |= (1<<ADEN);
}
