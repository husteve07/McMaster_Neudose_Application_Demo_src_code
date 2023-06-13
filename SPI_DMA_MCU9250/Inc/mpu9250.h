/*
 * mpu9250.h
 *
 *  Created on: Jun. 8, 2023
 *      Author: huste
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include <stdint.h>
#include "spi_dma.h"

#define MPU9250_USER_CTRL						0x6A

//accelerometer
#define MPU9250_ADDR_ACCELCONFIG		0x1C
	//accel full scale selecte [4:3]
#define ACC_FULL_SCALE_2_G				0x00
#define ACC_FULL_SCALE_4_G				0x08
#define ACC_FULL_SCALE_8_G				0x10
#define ACC_FULL_SCALE_16_G				0x18

#define MPU9250_ACCEL_XOUT_H			0x3B

//gyroscope
#define MPU9250_ADDR_GYROCONFIG			0x1B
	//gyro full scale select[4:3]
#define GYRO_FULL_SCALE_250_DPS				0x00
#define GYRO_FULL_SCALE_500_DPS				0x08
#define GYRO_FULL_SCALE_1000_DPS			0x10
#define GYRO_FULL_SCALE_2000_DPS			0x18

#define MPU9250_GYRO_XOUT_H					0x43
void mpu9250_config(uint8_t accel_mode, uint8_t gyro_mode);
void mpu9250_accel_update(void);
void mpu9250_gyro_update(void);
float mpu9250_get_accel_x(void);
float mpu9250_get_accel_y(void);
float mpu9250_get_accel_z(void);
float mpu9250_get_gyro_x(void);
float mpu9250_get_gyro_y(void);
float mpu9250_get_gyro_z(void);

void mpu9250_ncs_pin_config(void);
void mpu9250_ncs_pin_set(void);
void mpu9250_ncs_pin_reset(void);


#endif /* MPU9250_H_ */
