/*
 * mpu6050.h
 *
 * Created: 16.11.2021 10:50:32
 *  Author: kubam
 */ 

#ifndef MPU6050_H_
#define MPU6050_H_

#include "mpu6050_reg.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define dt 0.1
#define COMP_GAIN 0.98
#define STABILIZATION 20

extern volatile double accX, accY, accZ, tempMPU, gyroX, gyroY, gyroZ;
extern volatile double accel_pitch, accel_roll;
extern volatile double gyro_pitch, gyro_roll;
extern volatile double pitch, roll;
extern volatile double gx_error, gy_error, gz_error;

void mpu6050_init(void);				// initialize sensor
void mpu6050_readRawValues(void);		// read raw values
void mpu6050_computeMeasurements(void);	// compute raw measurements to g and deg/s
void mpu6050_getAccelGryoValues(void);	// read raw and compute
void mpu6050_calibrateGyro(void);		// calibrate gyro function
void mpu6050_getRollPitch(void);		// calculate roll and pitch

#endif /* MPU6050_H_ */
