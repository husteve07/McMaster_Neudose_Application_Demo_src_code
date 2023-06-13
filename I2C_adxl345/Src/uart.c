/*
 * uart.c
 *
 *  Created on: Jun. 3, 2023
 *      Author: huste
 */
#include <stdio.h>
#include "uart.h"
static void uart_set_baudrate(USART_TypeDef* USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t Baudrate);
void uart2_write(int ch);

#define GPIOAEN				(1U << 0)

#define SYS_FREQ			16000000
#define APB1_CLK			SYS_FREQ

#define UART_BAUDRATE		115200


//find the uart module (block diagram)
//use the one with the usb
//APB1 bus
//ready to enable RCC
#define UART2EN				(1U<<17)

#define CR1_TE				(1U<<3)
#define CR1_UE				(1U<<13)
#define CR1_RE				(1U<<2)

#define SR_TXE				(1U<<7)
#define SR_RXNE				(1U<<5)
//extern variable from syscalls.c
int __io_putchar(int ch)
{
	uart2_write(ch);
	return ch;
}

void uart2_tx_init(void)
{
	//enable clock acess to GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	//find TX RX GPIO pins (datasheet -> alternate function)
	//PA2 is TX
	//set PA2 alternate function type of UART_TX(AF07), first configure the mode register
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);

	//then configure the alternate function register
	//again since its UART2, we use the AFRL2, set it to AF7(PA 2 corresponds to the UART2 TX on data sheet)
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);

	//configure uart moduel

	//enable RCC to uart2
	RCC->APB1ENR |= UART2EN;
	//configure baudrate
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);
	//configure transfer direction(reference manual to find the enable bit) use the control reg
	USART2->CR1 = CR1_TE;
	/**this automatically sets up the uart control reg, word width, start, stop, parity etc, check ref man.**/
	//finally enable UART module (UE)
	USART2->CR1 |= CR1_UE;
}

static void uart_set_baudrate(USART_TypeDef* USARTx, uint32_t PeriphClk, uint32_t BaudRate)
{
	//set baudrate register
	USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);


}


void uart2_rxtx_init(void)
{
	//enable clock acess to GPIOA
	RCC->AHB1ENR |= GPIOAEN;
	//find TX RX GPIO pins (datasheet -> alternate function)
	//PA2 is TX
	//set PA2 alternate function type of UART_TX(AF07), first configure the mode register
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);

	//then configure the alternate function register
	//again since its UART2, we use the AFRL2, set it to AF7(PA 2 corresponds to the UART2 TX on data sheet)
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);

	//set PA3(RX) to alternate fucntion mode
	GPIOA->MODER &= ~(1U<<6);
	GPIOA->MODER |= (1U<<7);
	//set PA3 alternate fucntion type to UART_RX(AF07)
	//AFRL3 for RX
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &= ~(1U<<15);

	//configure uart module
	//enable RCC to uart2
	RCC->APB1ENR |= UART2EN;
	//configure baudrate
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);
	//configure transfer direction(reference manual to find the enable bit) use the control reg
	USART2->CR1 = (CR1_TE|CR1_RE);
	/**this automatically sets up the uart control reg, word width, start, stop, parity etc, check ref man.**/
	//finally enable UART module (UE)
	USART2->CR1 |= CR1_UE;


}

char uart2_read(void)
{
	//make sure recieve data reg is NOT empty
	while(!(USART2->SR & SR_RXNE))
	{
	}
	return USART2->DR;
}



void uart2_write(int ch)
{
	//make sure transfer data reg is empty, ref manual, status reg
	while(!(USART2->SR & SR_TXE))
	{

	}
	//write to transmit data reg
	USART2->DR = (ch & 0xFF);

}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t Baudrate)
{
	return ((PeriphClk + (Baudrate/2U))/Baudrate);
}
