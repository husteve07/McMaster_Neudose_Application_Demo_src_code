/*
 * mpu9250.c
 *
 *  Created on: Jun. 8, 2023
 *      Author: huste
 */
#include "mpu9250.h"
#define GPIOAEN									(1U<<0)


#define SPI_DATA_BUFF_LEN						2
#define USER_CTRL_I2C_IF_DIS					(1U<<4)//switch to SPI
#define MAX_TRANSFER_LEN						6
#define READ_FLAG								0x80

double accel_range;
double gyro_range;


uint8_t dummy_buff[MAX_TRANSFER_LEN+1];
uint8_t accel_buff[MAX_TRANSFER_LEN+1];
uint8_t gyro_buff[MAX_TRANSFER_LEN+1];

uint8_t spi_data_buff[2];
uint8_t g_tx_cmplt;
uint8_t g_rx_cmplt;

void mpu9250_ncs_pin_config(void)
{
	RCC->AHB1ENR |= GPIOAEN;
	GPIOA->MODER |= (1U<<0);
	GPIOA->MODER &= ~(1U<<1);
}

void mpu9250_ncs_pin_set(void)
{
	GPIOA->ODR |= (1U<<0);
}

void mpu9250_ncs_pin_reset(void)
{
	GPIOA->ODR &= ~(1U<<0);
}

void mpu9250_config(uint8_t accel_mode, uint8_t gyro_mode)
{
	switch(accel_mode)
	{
		case ACC_FULL_SCALE_2_G:
			accel_range = 2.0;
			break;
		case ACC_FULL_SCALE_4_G:
			accel_range = 4.0;
			break;
		case ACC_FULL_SCALE_8_G:
			accel_range = 8.0;
			break;
		case ACC_FULL_SCALE_16_G:
			accel_range = 16.0;
			break;
		default:
			break;
	}

	switch(gyro_mode)
	{
		case GYRO_FULL_SCALE_250_DPS:
			gyro_range = 250.0;
			break;
		case GYRO_FULL_SCALE_500_DPS:
			gyro_range = 500.0;
			break;
		case GYRO_FULL_SCALE_1000_DPS:
			gyro_range = 1000.0;
			break;
		case GYRO_FULL_SCALE_2000_DPS:
			gyro_range = 2000.0;
			break;
		default:
			break;
	}
	//set to SPI mode only
	spi_data_buff[0] = MPU9250_USER_CTRL;
	spi_data_buff[1] = USER_CTRL_I2C_IF_DIS;
	//sent configuration to MPU
	dma2_stream3_spi_transfer((uint32_t)spi_data_buff, (uint32_t)SPI_DATA_BUFF_LEN);

	//wait for transfer completion
	while(!g_tx_cmplt){}

	//transfer complete, reset flg
	g_tx_cmplt = 0;

	//configure Accel Range
	spi_data_buff[0] = MPU9250_ADDR_ACCELCONFIG;
	spi_data_buff[1] = accel_mode;

	dma2_stream3_spi_transfer((uint32_t)spi_data_buff, (uint32_t)SPI_DATA_BUFF_LEN);

	//wait for transfer completion
	while(!g_tx_cmplt){}

	//transfer complete, reset flg
	g_tx_cmplt = 0;

	//configure gyro Range
	spi_data_buff[0] = MPU9250_ADDR_GYROCONFIG;
	spi_data_buff[1] = gyro_mode;

	dma2_stream3_spi_transfer((uint32_t)spi_data_buff, (uint32_t)SPI_DATA_BUFF_LEN);

	//wait for transfer completion
	while(!g_tx_cmplt){}

	//transfer complete, reset flg
	g_tx_cmplt = 0;
}

void mpu9250_accel_update(void)
{
	dummy_buff[0] = MPU9250_ACCEL_XOUT_H | READ_FLAG;
	dma2_stream2_spi_recieve((uint32_t)accel_buff, (uint32_t)(MAX_TRANSFER_LEN +1));
	dma2_stream3_spi_transfer((uint32_t) dummy_buff, (uint32_t)(MAX_TRANSFER_LEN+ 1));

	//wait for reception completion
	while(!g_rx_cmplt){}

	//transfer complete, reset flg
	g_tx_cmplt = 0;
}

void mpu9250_gyro_update(void)
{
	dummy_buff[0] = MPU9250_GYRO_XOUT_H | READ_FLAG;
	dma2_stream2_spi_recieve((uint32_t)gyro_buff, (uint32_t)(MAX_TRANSFER_LEN +1));
	dma2_stream3_spi_transfer((uint32_t) gyro_buff, (uint32_t)(MAX_TRANSFER_LEN+ 1));

	//wait for reception completion
	while(!g_rx_cmplt){}

	//transfer complete, reset flg
	g_tx_cmplt = 0;
}

float mpu9250_accel_get(uint8_t high_idx, uint8_t low_idx)
{
	int16_t res;

	res = accel_buff[high_idx]<<8 | accel_buff[low_idx];
	if(res)
	{
		return ((float)-res * accel_range/(float)0x8000);
	}
	else
	{
		return 0.0;
	}
}

float mpu9250_gyro_get(uint8_t high_idx, uint8_t low_idx)
{
	int16_t res;

	res = gyro_buff[high_idx]<<8 | gyro_buff[low_idx];
	if(res)
	{
		return ((float)-res * gyro_range/(float)0x106);
	}
	else
	{
		return 0.0;
	}
}

float mpu9250_get_accel_x(void)
{
	return mpu9250_accel_get(1,2);
}
float mpu9250_get_accel_y(void)
{
	return mpu9250_accel_get(3,4);
}
float mpu9250_get_accel_z(void)
{
	return mpu9250_accel_get(5,6);
}

float mpu9250_get_gyro_x(void)
{
	return mpu9250_gyro_get(1,2);
}
float mpu9250_get_gyro_y(void)
{
	return mpu9250_gyro_get(3,4);
}
float mpu9250_get_gyro_z(void)
{
	return mpu9250_gyro_get(5,6);
}


void DMA2_Stream3_IRQHandler(void)
{
	if((DMA2->LISR)&LISR_TCIF3)
	{
		g_tx_cmplt = 1;
		DMA2->LIFCR |= LIFCR_CTCIF3;
	}

	else if((DMA2->LISR)&LISR_TEIF3)
	{
		//free to implement
		DMA2->LIFCR |= LIFCR_CTEIF3;
	}
}

void DMA2_Stream2_IRQHandler(void)
{
	if((DMA2->LISR)&LISR_TCIF2)
	{
		g_rx_cmplt = 1;
		DMA2->LIFCR |= LIFCR_CTCIF2;
	}

	else if((DMA2->LISR)&LISR_TEIF2)
	{
		DMA2->LIFCR |= LIFCR_CTEIF2;
	}
}
