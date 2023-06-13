/*
 * adc.h
 *
 *  Created on: Jun. 4, 2023
 *      Author: huste
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include "uart.h"

void pa1_adc_init(void);
uint32_t adc_read(void);
void start_conversion(void);
void pa1_adc_init(void);
void pa1_adc_interrupt_init(void);
#define SR_EOC				(1U<<1)
#endif /* ADC_H_ */
