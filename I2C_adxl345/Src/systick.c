/*
 * systick.c
 *
 *  Created on: Jun. 5, 2023
 *      Author: huste
 */

#include "stm32f4xx.h"

//systick is a arm cortex m core peripheral, goto the arm documentation for specifications
#define SYSTICK_LOAD_VAL 			16000
#define CTRL_ENABLE					(1U<<0)
#define CTRL_CLKSRC					(1U<<2)
#define CTRL_COUNTFLAG				(1U<<16)

void systickDelayMs(int delay)
{
	//configure systick
	//reload with number of clocks per millisecond
	SysTick->LOAD = SYSTICK_LOAD_VAL;

	//clear systick current value register
	SysTick->VAL = 0;

	//enable systick and select internal clk src
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

	for(int i = 0; i < delay; i++)
	{
		//wait until count flag is set
		while((SysTick->CTRL & CTRL_COUNTFLAG) == 0) {}
	}
	SysTick->CTRL = 0;

}
