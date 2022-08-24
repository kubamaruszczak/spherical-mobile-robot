/*
 * drv8833_md.h
 *
 * Created: 29.11.2021 08:43:06
 *  Author: kubam
 */ 


#ifndef DRV8833_MD_H_
#define DRV8833_MD_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

// ---------------- MOTORS ----------------
// software PWM pins
#define AIN1 PB0	// pin 8
#define AIN2 PD7	// pin 7	
#define BIN1 PD5	// pin 5
#define BIN2 PD6	// pin 6
// DDR registers
#define DDR_AIN1 DDRB
#define DDR_AIN2 DDRD
#define DDR_BIN1 DDRD
#define DDR_BIN2 DDRD
// PORT registers
#define PORT_AIN1 PORTB
#define PORT_AIN2 PORTD
#define PORT_BIN1 PORTD
#define PORT_BIN2 PORTD
// rotation direction
#define FORWARD 1
#define BACKWARD 0

// --------------- ENCODERS ---------------
// encoders pins
#define ENCA_M1 PD2	// pin INT0
#define ENCA_M2 PD3	// pin INT1
#define ENCB_M1 PC2	// pin A2
#define ENCB_M2 PC3	// pin A3
// PIN registers
#define PIN_ENCA_M1 PIND
#define PIN_ENCA_M2 PIND
#define PIN_ENCB_M1 PINC
#define PIN_ENCB_M2 PINC

// motors position variables
extern volatile int64_t pos_M1, pos_M2;

// motor M1 PID variables
extern volatile double e_M1, eIntegral_M1, eDerivative_M1;
extern volatile int64_t targetPos_M1;
extern double controlSignal_M1;

// motor M2 PID variables
extern volatile double e_M2, eIntegral_M2, eDerivative_M2;
extern volatile int64_t targetPos_M2;
extern double controlSignal_M2;

void motorM1_pwm(uint8_t forwardDir, uint8_t pwmDutyCycle);						// funkcja steruj�ca silnikiem 1 z zadanym wype�nieniem PWM w procentach 
void motorM2_pwm(uint8_t forwardDir, uint8_t pwmDutyCycle);						// funkcja steruj�ca silnikiem 2 z zadanym wype�nieniem PWM w procentach
void motors_init(void);															// funkcja przygotowuj�ca silniki do pracy
void motor_PID(uint8_t motor, int64_t targetPos, double kp, double ki, double kd);	// funkcja realizuj�ca algorytm PID dla wybranego silnika z okre�lonymi wzmocnieniami

#endif /* DRV8833_MD_H_ */
