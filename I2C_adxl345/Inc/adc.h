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


#endif /* ADC_H_ */
