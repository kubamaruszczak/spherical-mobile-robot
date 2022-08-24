/*
 * mpu6050_reg.h
 *
 * Created: 16.11.2021 12:46:49
 *  Author: kubam
 */ 


#ifndef MPU6050_REG_H_
#define MPU6050_REG_H_

#define MPU6050_ADDR				0xD0	// slave address
#define WHO_AM_I					0x75	// who am i address
#define SMPLRT_DIV_REG			0x19
#define CONFIG_REG				0x1A
#define GYRO_CONFIG_REG			0x1B
#define ACCEL_CONFIG_REG			0x1C
#define INT_ENABLE_REG			0x38
#define ACCEL_XOUT_H_REG			0x3B
#define TEMP_OUT_H_REG			0x41
#define GYRO_XOUT_H_REG			0x43
#define SIGNAL_PATH_RESET_REG	0x68	
#define PWR_MGMT_1_REG			0x6B

#endif /* MPU6050_REG_H_ */
