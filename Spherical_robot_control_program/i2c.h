/*
 * i2c.h
 *
 * Created: 08.11.2021 13:27:08
 *  Author: kubam
 */ 


#ifndef I2C_H_
#define I2C_H_

#define F_CPU 16000000UL

#define I2C_STARTERROR 1
#define I2C_NOACK 3
#define I2C_NONACK 4

#define F_SLC_KHZ 400	// I2C frequency

extern uint8_t i2c_error;	// error code variable

void i2c_init(void);							// I2C initialization
void i2c_start(void);							// send START bit
void i2c_stop(void);							// send STOP bit
void i2c_sendStartAndSelectDev(uint8_t addr);	// send START and choose device
void i2c_startWaitAndSelectDev(uint8_t addr);	// send START, choose device and use polling to wait for ready device
void i2c_sendByte(uint8_t byte);				// send byte
uint8_t i2c_readByte_NAKC(void);				// read byte and send NACK
uint8_t i2c_readByte_AKC(void);					// read byte and send ACK

#endif /* I2C_H_ */
