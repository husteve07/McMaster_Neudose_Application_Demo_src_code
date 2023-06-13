/*
 * spi_dma.c
 *
 *  Created on: Jun. 8, 2023
 *      Author: huste
 */
/**
 * PA5 SCK
 * PA6 MISO
 * PA7 MOSI
 * PA0 NCS
 */
#include "spi_dma.h"

#define GPIOAEN 			(1U<<0)
#define SPI1EN				(1U<<12)

#define CR1_SSM				(1U<<9)
#define CR1_SSI				(1U<<8)
#define CR1_MSTR			(1U<<2)
#define CR1_CPOL			(1U<<1)
#define CR1_CPHA			(1U<<0)
#define CR2_TXDMAEN			(1U<<1)
#define CR2_RXDMAEN			(1U<<0)
#define CR1_SPIEN			(1U<<6)

/********DMA**************/
#define DMA2EN				(1U<<22)

#define DMA_SCR_EN  		(1U<<0)
#define DMA_SCR_MINC		(1U<<10)
#define DMA_SCR_PINC		(1U<<9)
#define DMA_SCR_CIRC		(1U<<8)
#define DMA_SCR_TCIE		(1U<<4)
#define DMA_SCR_TEIE		(1U<<2)
#define DMA_SFCR_DMDIS		(1U<<2)

#define HIFCR_CDMEIF5		(1U<<8)
#define HIFCR_CTEIF5		(1U<<9)
#define HIFCR_CTCIF5		(1U<<11)

#define HIFCR_CDMEIF6		(1U<<18)
#define HIFCR_CTEIF6		(1U<<19)
#define HIFCR_CTCIF6		(1U<<21)

#define HIFSR_TCIF5			(1U<<11)
#define HIFSR_TCIF6			(1U<<21)

#define LIFCR_CTCIF2		(1U<<21)
#define LIFCR_CTCIF3		(1U<<27)

#define LIFCR_CTEIF2		(1U<<19)
#define LIFCR_CTEIF3		(1U<<25)

void spi_dma_init(void)
{
	/********GPIO config********/
	// enable clok to port of spi pins
	RCC->AHB1ENR |= GPIOAEN;

	//set spi pins to spi mode (AF5)
	//PA5 set GPIO to alt function
	GPIOA->MODER &= ~(1U<<10);
	GPIOA->MODER |= (1U<<11);
	//6
	GPIOA->MODER &= ~(1U<<12);
	GPIOA->MODER |= (1U<<13);
	//7
	GPIOA->MODER &= ~(1U<<14);
	GPIOA->MODER |= (1U<<15);

	//PA5 alt function set to AF5(SPI)
	GPIOA->AFR[0] |= (1U<<20);
	GPIOA->AFR[0] &= ~(1U<<21);
	GPIOA->AFR[0] |= (1U<<22);
	GPIOA->AFR[0] &= ~(1U<<23);
	//6
	GPIOA->AFR[0] |= (1U<<24);
	GPIOA->AFR[0] &= ~(1U<<25);
	GPIOA->AFR[0] |= (1U<<26);
	GPIOA->AFR[0] &= ~(1U<<27);
	//7
	GPIOA->AFR[0] |= (1U<<28);
	GPIOA->AFR[0] &= ~(1U<<29);
	GPIOA->AFR[0] |= (1U<<30);
	GPIOA->AFR[0] &= ~(1U<<31);


	/********SPI config*********/
	//enable clk access to spi1 module
	RCC->APB2ENR |= SPI1EN;

	//set sftware slave management
	SPI1->CR1 |= CR1_SSM|CR1_SSI;

	//set SPI to master mode
	SPI1->CR1 |= CR1_MSTR;

	//set cpha and cpol
	SPI1->CR1 |= CR1_CPHA|CR1_CPOL;

	//set clock dividor(baudrate control, pclck/4)
	SPI1->CR1 |= (1U<<3);
	SPI1->CR1 &= ~(1U<<4);
	SPI1->CR1 &= ~(1U<<5);

	// select to use DMA
	SPI1->CR2 |= CR2_TXDMAEN|CR2_RXDMAEN;

	//enable spi
	SPI1->CR1 |= CR1_SPIEN;
}

void dma2_stream3_spi_tx_init(void)
{
	/********DMA config*********/
	//enable clk to DMA2
	RCC->AHB1ENR |= DMA2EN;

	//disable DMA stream
	DMA2_Stream3->CR = 0;

	//wait till DMA2 is disabled
	while((DMA2_Stream3->CR & DMA_SCR_EN)){}

	/********configure DMA stream parameters*******/
	//enable memory addr increment
	DMA2_Stream3->CR |= DMA_SCR_MINC;

	//select channels
	DMA2_Stream3->CR |= (1U<<25);
	DMA2_Stream3->CR |= (1U<<26);
	DMA2_Stream3->CR &= ~(1U<<27);

	//set transfer direction, M2P
	DMA2_Stream3->CR |= (1U<<6);
	DMA2_Stream3->CR &= ~(1U<<7);

	//enable transfer complete interrupt
	DMA2_Stream3->CR |= DMA_SCR_TCIE;

	//enable transfer error interrupt
	DMA2_Stream3->CR |= DMA_SCR_TEIE;

	//disable direct mode
	DMA2_Stream3->FCR |= DMA_SFCR_DMDIS;

	//set DMA2 FIFO threshold, full
	DMA2_Stream3->FCR |= (1U<<0);
	DMA2_Stream3->FCR |= (1U<<1);

	//enable DMA int in NVIC
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

void dma2_stream2_spi_rx_init(void)
{
	//same DMA config/stream parameter config as tx
	RCC->AHB1ENR |= DMA2EN;
	DMA2_Stream2->CR = 0;
	while((DMA2_Stream2->CR & DMA_SCR_EN)){}
	DMA2_Stream2->CR |= DMA_SCR_MINC;

	//select channels
	DMA2_Stream2->CR |= (1U<<25);
	DMA2_Stream2->CR |= (1U<<26);
	DMA2_Stream2->CR &= ~(1U<<27);

	//set transfer direction to P2M, default is already 0x00 but jsut to be sure
	DMA2_Stream2->CR &= ~(1U<<6);
	DMA2_Stream2->CR &= ~(1U<<7);

	DMA2_Stream2->CR |= DMA_SCR_TCIE;
	DMA2_Stream2->CR |= DMA_SCR_TEIE;

	DMA2_Stream2->FCR |= DMA_SFCR_DMDIS;

	DMA2_Stream2->FCR |= (1U<<0);
	DMA2_Stream2->FCR |= (1U<<1);

	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

void dma2_stream3_spi_transfer(uint32_t data, uint32_t data_len)
{
	//clear interrupt flags
	DMA2->LIFCR = LIFCR_CTCIF3 | LIFCR_CTCIF3;

	//set periph addr
	DMA2_Stream3->PAR = (uint32_t)(&(SPI1->DR));

	//set memory addr
	DMA2_Stream3->M0AR = data;

	//set transfer length
	DMA2_Stream3->NDTR = data_len;

	//enable DMA stream
	DMA2_Stream3->CR |= DMA_SCR_EN;

}

void dma2_stream2_spi_recieve(uint32_t recieved_data, uint32_t recieved_data_len)
{
	DMA2->LIFCR = LIFCR_CTCIF2 | LIFCR_CTCIF2;

	//set periph addr
	DMA2_Stream2->PAR = (uint32_t)(&(SPI1->DR));

	//set memory addr
	DMA2_Stream2->M0AR = recieved_data;

	//set transfer length
	DMA2_Stream2->NDTR = recieved_data_len;

	//enable DMA stream
	DMA2_Stream2->CR |= DMA_SCR_EN;
}
