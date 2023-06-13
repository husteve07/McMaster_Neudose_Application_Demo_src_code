/*
 * spi_dma.h
 *
 *  Created on: Jun. 8, 2023
 *      Author: huste
 */

#ifndef SPI_DMA_H_
#define SPI_DMA_H_

#include "stm32f4xx.h"
#include <stdint.h>

#define LISR_TCIF3				(1U<<27)
#define LISR_TCIF2				(1U<<21)
#define LISR_TEIF3				(1U<<25)
#define LISR_TEIF2				(1U<<19)

#define LIFCR_CTCIF2		(1U<<21)
#define LIFCR_CTCIF3		(1U<<27)
#define LIFCR_CTEIF2		(1U<<19)
#define LIFCR_CTEIF3		(1U<<25)

void spi_dma_init(void);
void dma2_stream3_spi_tx_init(void);
void dma2_stream2_spi_rx_init(void);
void dma2_stream3_spi_transfer(uint32_t data, uint32_t data_len);
void dma2_stream2_spi_recieve(uint32_t recieved_data, uint32_t recieved_data_len);


#endif /* SPI_DMA_H_ */
