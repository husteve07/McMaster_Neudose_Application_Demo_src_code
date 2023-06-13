#include "adxl.h"

#define MULTI_BYTE_EN			(0x40)
#define READ_OPERATION			(0x80)

void adxl_read(uint8_t address, uint8_t* rxdata)
{
	/*set read operation*/
	address |= READ_OPERATION;

	/*Enable multi-byte, place address into buffer*/
	address |= MULTI_BYTE_EN;

	/*pull cs line low to enable slaves*/
	cs_enable();

	/*send address*/
	spi1_transmit(&address, 1);

	/*read 6 bytes*/
	spi1_recieve(rxdata,6);

	/*pull cs line high to disable slave*/
	cs_disable();
}

void adxl_write (uint8_t address, char value)
{
	uint8_t data[2];
	/*enable multi byte, place adress into buffer*/
	data[0] = address | MULTI_BYTE_EN;

	/*place data in buffer*/
	data[1] = value;

	/*pull cs line low to enable slaves*/
	cs_enable();

	/*transmit data and address*/
	spi1_transmit(data,2);

	/*pull cs line high to disable slave*/
	cs_disable();
}

void adxl_init (void)
{
	/*Enable SPI gpio*/
	spi_gpio_init();
	/*config SPI */
	spi1_config();

	/*Set data format range to +-4g*/
	adxl_write (DATA_FORMAT_R, FOUR_G);

	/*Reset all bits*/
	adxl_write (POWER_CTRL_R, RESET);

	/*Configure power control measure bit*/
	adxl_write (POWER_CTRL_R, SET_MEASURE_B);
}

