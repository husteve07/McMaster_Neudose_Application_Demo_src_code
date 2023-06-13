/*
 * tim.c
 *
 *  Created on: Jun. 5, 2023
 *      Author: huste
 */

#include "stm32f4xx.h"
#include "tim.h"

//find TIMs on block diagrams, use TIM2

#define TIM2EN					(1U<<0)
#define CR1_CEN					(1U<<0)
#define DIER_UIE				(1U<<0)

void tim2_1hz_init(void)
{
	//enable clk access to tim2
	RCC->APB1ENR |= TIM2EN;
	//set prescalar value
	TIM2->PSC = 1600 - 1;  //sys clk 16M/1600 = 10000
	//set auto reload value
	TIM2->ARR = 10000 - 1 ; // sysclk  10000/10000 = 1
	//clear counter
	TIM2->CNT = 0;
	//enable timer(ref)
	TIM2->CR1 = CR1_CEN;
}

void tim2_1hz_interrupt_init(void)
{
	//enable clk access to tim2
	RCC->APB1ENR |= TIM2EN;
	//set prescalar value
	TIM2->PSC = 1600 - 1;  //sys clk 16M/1600 = 10000
	//set auto reload value
	TIM2->ARR = 10000 - 1 ; // sysclk  10000/10000 = 1
	//clear counter
	TIM2->CNT = 0;
	//enable timer(ref)
	TIM2->CR1 = CR1_CEN;

	//enable TIM interrupt
	TIM2->DIER |= DIER_UIE;
	//enable TIM interrupt in NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
}

//
