/*
 * adxl.h
 *
 *  Created on: Jun. 7, 2023
 *      Author: huste
 */

#ifndef ADXL_H_
#define ADXL_H_
#include "SPI.h"
#include <stdint.h>
//everything is found in the official adxl345 datasheet
#define DEVID_R				0x00
#define DEVICE_ADDR			(0x53)
#define DATA_FORMAT_R		(0x31)
#define POWER_CTRL_R		(0x2D)
#define DATA_START_ADDR		(0x32)

#define FOUR_G				(0x01)
#define RESET				(0x00)
#define SET_MEASURE_B		(0x08)


void adxl_init(void);
void adxl_read(uint8_t address, uint8_t* rxdata);
#endif /* ADXL_H_ */
