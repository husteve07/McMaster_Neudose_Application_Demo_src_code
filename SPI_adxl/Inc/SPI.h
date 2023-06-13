/*
 * SPI.h
 *
 *  Created on: Jun. 7, 2023
 *      Author: huste
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx.h"
#include <stdint.h>


void cs_enable(void);
void cs_disable(void);
void spi1_recieve(uint8_t* data, uint32_t size);
void spi1_transmit(uint8_t *data, uint32_t size);
void spi1_config(void);
void spi_gpio_init(void);

#endif /* SPI_H_ */
