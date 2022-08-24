/*
 * mpu6050.c
 *
 * Created: 16.11.2021 10:50:45
 *  Author: kubam
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "mpu6050.h"
#include "i2c.h"

volatile double accX, accY, accZ, tempMPU, gyroX, gyroY, gyroZ;
volatile double accel_pitch, accel_roll;
volatile double gyro_pitch, gyro_roll;
volatile double ax_conf, ay_conf, az_conf;
volatile double gx_error, gy_error, gz_error;
volatile double pitch, roll;

static int Accel_X_RAW = 0;
static int Accel_Y_RAW = 0;
static int Accel_Z_RAW = 0;

static int Temp_RAW = 0;

static int Gyro_X_RAW = 0;
static int Gyro_Y_RAW = 0;
static int Gyro_Z_RAW = 0;

void mpu6050_init(void)
{
		_delay_ms(150);
		
		// set sending data frequency to 1kHz
		i2c_startWaitAndSelectDev(MPU6050_ADDR);
		i2c_sendByte(SMPLRT_DIV_REG);
		i2c_sendByte(0x07);
		i2c_stop();
		
		// wake up sensor
		i2c_startWaitAndSelectDev(MPU6050_ADDR);
		i2c_sendByte(PWR_MGMT_1_REG);
		i2c_sendByte(0x01);
		i2c_stop();
		
		// +/- 2000 deg/s
		i2c_startWaitAndSelectDev(MPU6050_ADDR);
		i2c_sendByte(GYRO_CONFIG_REG);
		i2c_sendByte(0x18);
		i2c_stop();
		
		// +/- 2g
		i2c_startWaitAndSelectDev(MPU6050_ADDR);
		i2c_sendByte(ACCEL_CONFIG_REG);
		i2c_sendByte(0x00);
		i2c_stop();
}

void mpu6050_readRawValues(void)
{
	i2c_startWaitAndSelectDev(MPU6050_ADDR);
	i2c_sendByte(ACCEL_XOUT_H_REG);
	i2c_sendStartAndSelectDev(MPU6050_ADDR + 1);
	Accel_X_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();
	Accel_Y_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();
	Accel_Z_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();

	Temp_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();

	Gyro_X_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();
	Gyro_Y_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();
	Gyro_Z_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_NAKC();
	i2c_stop();
}

void mpu6050_computeMeasurements(void)
{
	accX = Accel_X_RAW / 16384.0;
	accY = Accel_Y_RAW / 16384.0;
	accZ = Accel_Z_RAW / 16384.0;
	
	tempMPU = (Temp_RAW / 340.00) + 36.53;
	
	gyroX = Gyro_X_RAW / 16.4 - gx_error;
	gyroY = Gyro_Y_RAW / 16.4 - gy_error;
	gyroZ = Gyro_Z_RAW / 16.4 - gz_error;
}

void mpu6050_getAccelGryoValues(void)
{
	mpu6050_readRawValues();
	mpu6050_computeMeasurements();
}

void mpu6050_calibrateGyro(void)
{
	uint16_t counter = 1000;
	_delay_ms(3000);
	
	while(counter--)
	{
		i2c_startWaitAndSelectDev(MPU6050_ADDR);
		i2c_sendByte(ACCEL_XOUT_H_REG);
		i2c_sendStartAndSelectDev(MPU6050_ADDR + 1);
		Accel_X_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();
		Accel_Y_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();
		Accel_Z_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();

		Temp_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();
		
		Gyro_X_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();
		Gyro_Y_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_AKC();
		Gyro_Z_RAW = ((int)i2c_readByte_AKC() << 8) | (int)i2c_readByte_NAKC();
		i2c_stop();
		
		ax_conf += Accel_X_RAW / 16384.0;
		ay_conf += Accel_Y_RAW / 16384.0;
		az_conf += Accel_Z_RAW / 16384.0;
		
		gx_error += Gyro_X_RAW / 16.4;
		gy_error += Gyro_Y_RAW / 16.4;
		gz_error += Gyro_Z_RAW / 16.4;
	}
	
	ax_conf /= 1000.0;
	ay_conf /= 1000.0;
	az_conf /= 1000.0;
	gx_error /= 1000.0;
	gy_error /= 1000.0;
	gz_error /= 1000.0;
	
	// calculate roll and pitch from acc measurements
	accel_roll = (atan2(ay_conf, az_conf)*180.0)/M_PI;
	accel_pitch = -(atan2(ax_conf, sqrt(ay_conf*ay_conf + az_conf*az_conf))*180.0)/M_PI;
	
	// initialize gyro roll and pitch
	gyro_roll = accel_roll;
	gyro_pitch = accel_pitch;
}

void mpu6050_getRollPitch(void)
{
	// calculate roll and pitch from acc measurements
	accel_pitch = -(atan2(accX, sqrt(accY*accY + accZ*accZ))*180.0)/M_PI;
	accel_roll = (atan2(accY, accZ)*180.0)/M_PI;
	
	// calculate roll and pitch from gyro measurements
	gyro_pitch += gyroY * dt;
	gyro_roll += gyroX * dt;
	
	// complementary filter implementation
	roll = COMP_GAIN * gyro_roll + (1 - COMP_GAIN) * accel_roll;
	pitch = COMP_GAIN * gyro_pitch + (1 - COMP_GAIN) * accel_pitch;
}
