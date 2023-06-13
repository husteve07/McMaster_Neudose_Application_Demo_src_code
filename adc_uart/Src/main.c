#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "uart.h"
#include "adc.h"

static void adc_callback(void);
void ADC_IRQHandler(void);

uint32_t sensor_value;

int main(void)
{
	uart2_tx_init();
	//remember to initialize then convert you idiot
	pa1_adc_interrupt_init();
	start_conversion();
	while(1)
	{
		//start_conversion();
		sensor_value =  adc_read();
		printf("Sensor value : %d \n\r",(int)sensor_value);
	}

}

static void adc_callback(void)
{
	//start_conversion();
	sensor_value = ADC1->DR;
	printf("Sensor value : %d \n\r",(int)sensor_value);
}

void ADC_IRQHandler(void)
{
	//check for EOC is SR
	if((ADC1->SR & SR_EOC) != 0)
	{
		//clear EOC
		ADC1->SR &= ~SR_EOC;

		//do stuff
		adc_callback();
	}
}





