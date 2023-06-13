/*
 * adc.c
 *
 *  Created on: Jun. 4, 2023
 *      Author: huste
 */
#include "stm32f4xx.h"
#include "adc.h"


//block diagram -> locate bus(APB2) ->enable register bit
#define ADC1EN				(1U<<8)
//find pins connected to ADC1 channels(datasheet), use PA1 for now
#define GPIOAEN				(1U<<0)
//locate the channel that we will be using
#define ADC_CH1				(1U<<0)
//sequence length
#define ADC_SEQ_LENG_1		(0x00)
//turn on adc
#define CR2_ADON			(1U<<0)
//start adc
#define CR2_SWSTART			(1U<<30)
//enable continuous conversion
#define CR2_CONT			(1U<<1)
//status register to check for end of conversion
#define SR_EOC				(1U<<1)


void pa1_adc_init(void)
{
	//enable clk to adc pin
	RCC->AHB1ENR |= GPIOAEN;
	//set pa1 to analog mode
	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	//configure adc module
	//enable clk access to adc
	RCC->APB2ENR |= ADC1EN;

	//config adc parameters;
	//conversion sequence start
	//look at adc sequence register(ref)
	ADC1->SQR3 = ADC_CH1;
	//conversion sequence length
	ADC1->SQR1 = ADC_SEQ_LENG_1;
	//enable adc module
	ADC1->CR2 |= CR2_ADON;
}

void start_conversion(void)
{
	//start ADC conversion
	ADC1->CR2 |= CR2_SWSTART;
	ADC1->CR2 |= CR2_CONT;
}

uint32_t adc_read(void)
{
	//wait for conversion to complete, use the adc status register
	while(!(ADC1->SR & SR_EOC))
	{

	}
	//read converted result
	return ADC1->DR;
}
