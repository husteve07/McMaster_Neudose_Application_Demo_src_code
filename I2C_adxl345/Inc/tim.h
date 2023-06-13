/*
 * tim.h
 *
 *  Created on: Jun. 5, 2023
 *      Author: huste
 */

#ifndef TIM_H_
#define TIM_H_

void tim2_1hz_init(void);
void tim2_1hz_interrupt_init(void);

#define SR_UIF (1U<<0)	//UPDATE INTERRUPT FLAG ONE SECOND

#endif /* TIM_H_ */
