/*
 * SPI.c
 *
 *  Created on: Jun. 7, 2023
 *      Author: huste
 */
#include "SPI.h"

#define SPI1EN 						(1U<<12)
//PA5(SCK), PA6(MISO), PA7(MOSI), PA9(SS)
#define GPIOAEN						(1U<<0)
#define SR_TXE						(1U<<1)
#define SR_RXNE						(1U<<0)
#define SR_BSY						(1U<<7)

void spi_gpio_init(void)
{
	//enable clock access to gpio
	RCC->AHB1ENR |= GPIOAEN;

	//PA5
	GPIOA->MODER &= ~(1U<<10);
	GPIOA->MODER |= (1U<<11);

	//PA6
	GPIOA->MODER &= ~(1U<<12);
	GPIOA->MODER |= (1U<<13);

	//PA7
	GPIOA->MODER &= ~(1U<<14);
	GPIOA->MODER |= (1U<<15);

	//PA9 to output
	GPIOA->MODER |= (1U<<18);
	GPIOA->MODER &= ~(1U<<19);

	//set them to AF05(SPI)
	//PA5
	GPIOA->AFR[0] |= (1U<<20);
	GPIOA->AFR[0] &= ~(1U<<21);
	GPIOA->AFR[0] |= (1U<<22);
	GPIOA->AFR[0] &= ~(1U<<23);

	//PA6
	GPIOA->AFR[0] |= (1U<<24);
	GPIOA->AFR[0] &= ~(1U<<25);
	GPIOA->AFR[0] |= (1U<<26);
	GPIOA->AFR[0] &= ~(1U<<27);

	//PA7
	GPIOA->AFR[0] |= (1U<<28);
	GPIOA->AFR[0] &= ~(1U<<29);
	GPIOA->AFR[0] |= (1U<<30);
	GPIOA->AFR[0] &= ~(1U<<31);
}

void spi1_config(void)
{
	RCC->APB2ENR |= SPI1EN;

	//set clk to
	SPI1->CR1 |= (1U<<3);
	SPI1->CR1 &= ~(1U<<4);
	SPI1->CR1 &= ~(1U<<5);

	//set CPOL to 1 and CPHA to 1
	SPI1->CR1 |= (1U<<0);
	SPI1->CR1 |= (1U<<1);

	//enable full duplex
	SPI1->CR1 &= ~(1U<<10);

	//set MSB first
	SPI1->CR1 &= ~(1U<<7);

	//set f411 to master
	SPI1->CR1 |= (1U<<2);

	//set data size 8 bit
	SPI1->CR1 &= ~(1U<<11);

	//enable SSM and SSI
	SPI1->CR1 |= (1U<<8);
	SPI1->CR1 |= (1U<<9);

	//enable SPI
	SPI1->CR1 |= (1U<<6);
}

void spi1_transmit(uint8_t *data, uint32_t size)
{
	uint8_t temp;
	uint32_t i = 0;
	while(i < size)
	{
		//wait until TXE is set
		while(!(SPI1->SR & SR_TXE)){}

		//write to dr
		SPI1->DR = data[i];
		i++;
	}

	//wait until TXE is set
	while(!(SPI1->SR & SR_TXE)){}

	//waait for busy flag to reset
	while((SPI1->SR & (SR_BSY))){}

	//clear OVR flag
	temp = SPI1->DR;
	temp = SPI1->SR;
}

void spi1_recieve(uint8_t* data, uint32_t size)
{
	while(size)
	{
		//send dummy data
		SPI1->DR = 0;

		//wait for RXNE flag to be set
		while(!(SPI1->SR & (SR_RXNE))) {}

		//read data from data register
		*data++ = (SPI1->DR);
		size--;
	}
}

void cs_enable(void)
{
	GPIOA->ODR &= ~(1U<<9);
}

void cs_disable(void)
{
	GPIOA->ODR |= (1U<<9);
}
