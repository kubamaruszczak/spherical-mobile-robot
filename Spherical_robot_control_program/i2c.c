/*
 * i2c.c
 *
 * Created: 08.11.2021 13:27:20
 *  Author: kubam
 */ 

#include <avr/io.h>
#include <util/twi.h>
#include "i2c.h"

uint8_t i2c_error;	// error code variable

inline void i2c_setError(uint8_t err) {i2c_error = err;};	

static inline void i2c_waitForComplete(void)	{while (!(TWCR & (1<<TWINT)));};

static inline void i2c_waitTillStopComplete(void)	{while (TWCR & (1<<TWSTO));};

static void i2c_setBitRate(uint16_t speed)
{
	TWSR = 0x00;						// prescaler = 1
	TWBR = ((F_CPU/speed/1000)-16)/2;	// set SCL frequency
}

void i2c_init(void)
{	
	i2c_setBitRate(F_SLC_KHZ);
}

void i2c_start(void)
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	i2c_waitForComplete();
	if(TW_STATUS != TW_START && TW_STATUS != TW_REP_START)
		i2c_setError(I2C_STARTERROR);
}

void i2c_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
	i2c_waitTillStopComplete();
}

static void i2c_sendAddr(uint8_t addr)
{
	uint8_t status;
	if((addr & 0x01) == 0) status = TW_MT_SLA_ACK;
	else status = TW_MR_SLA_ACK;
	TWDR = addr;
	TWCR = (1<<TWINT) | (1<<TWEN);
	i2c_waitForComplete();
	if(TW_STATUS != status) i2c_setError(I2C_NOACK);
}

void i2c_sendStartAndSelectDev(uint8_t addr)
{
	i2c_start();
	i2c_sendAddr(addr);
}

void i2c_startWaitAndSelectDev(uint8_t addr)
{
	while (1) 
	{
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		i2c_waitForComplete();
		if(TW_STATUS != TW_START)
			continue;
		TWDR = addr;
		TWCR = (1<<TWINT) | (1<<TWEN);
		i2c_waitForComplete();
		if(TW_STATUS != TW_MT_SLA_ACK)
		{
			i2c_stop();
			continue;
		}
		break;
	}
}

void i2c_sendByte(uint8_t byte)
{
	TWDR = byte;
	TWCR = (1<<TWINT) | (1<<TWEN);
	i2c_waitForComplete();
	if(TW_STATUS != TW_MT_DATA_ACK) i2c_setError(I2C_NOACK);
}

uint8_t i2c_readByte_NAKC(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	i2c_waitForComplete();
	if(TW_STATUS != TW_MR_DATA_NACK) i2c_setError(I2C_NONACK);
	return TWDR;
}

uint8_t i2c_readByte_AKC(void)
{
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
	i2c_waitForComplete();
	if(TW_STATUS != TW_MR_DATA_ACK) i2c_setError(I2C_NOACK);
	return TWDR;
}
