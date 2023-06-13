#include "stm32f4xx.h"
#include <stdint.h>
#include "spi_dma.h"
#include "mpu9250.h"

float acc_x,acc_y, acc_z, gyro_x, gyro_y, gyro_z;

int main(void)
{
	//enable spi
	spi_dma_init();
	//configure ncs
	mpu9250_ncs_pin_config();
	//enable FPU
	SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
	//enable tx rx
	dma2_stream3_spi_tx_init();
	dma2_stream2_spi_rx_init();

	//reset ncs
	mpu9250_ncs_pin_reset();
	//config accel gyro
	mpu9250_config(ACC_FULL_SCALE_2_G, GYRO_FULL_SCALE_250_DPS);


	while(1)
	{
		//reset ncs
		mpu9250_ncs_pin_reset();
		//get accel
		mpu9250_accel_update();
		acc_x = mpu9250_get_accel_x();
		acc_y = mpu9250_get_accel_y();
		acc_z = mpu9250_get_accel_z();
		//get gyro
		mpu9250_gyro_update();
		gyro_x = mpu9250_get_gyro_x();
		gyro_y = mpu9250_get_gyro_y();
		gyro_z = mpu9250_get_gyro_z();
		//set ncs
		mpu9250_ncs_pin_set();
	}
}





