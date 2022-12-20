/*
 * spi_tx_test.c
 *
 *  Created on: Jul 29, 2022
 *      Author: kunal.chauhan
 */

// PB14 is MISO
// PB15 is MOSI
// PB13 is SCLK
// PB12 is NSS
// alt fun is 5

#include <stdint.h>
#include <string.h>
#include "stm32f407_spi_driver.h"
#include "stm32f407_gpio_driver.h"

void SPI2_GPIOpin_init(void)
{
	GPIO_handle_t SPIpin;

	SPIpin.pGPIOx_base_addr = GPIOB;
	SPIpin.GPIO_pin_config.GPIO_pin_mode = GPIO_MODE_ALTFUN;
	SPIpin.GPIO_pin_config.GPIO_pin_altfuncmode = 5;
	SPIpin.GPIO_pin_config.GPIO_pin_optype = GPIO_OP_TYPE_PP;
	SPIpin.GPIO_pin_config.GPIO_pin_pupdcontrol = GPIO_NO_PUPD;
	SPIpin.GPIO_pin_config.GPIO_pin_speed = GPIO_SPEED_FAST;

	// enable clck
	GPIO_clk_ctrl(GPIOB, ENABLE);

	// clock
	SPIpin.GPIO_pin_config.GPIO_pin_number = GPIO_PIN_13;
	GPIO_init(&SPIpin);

	// mosi
	SPIpin.GPIO_pin_config.GPIO_pin_number = GPIO_PIN_15;
	GPIO_init(&SPIpin);

	// miso
	SPIpin.GPIO_pin_config.GPIO_pin_number = GPIO_PIN_14;
	GPIO_init(&SPIpin);

	// nss
	SPIpin.GPIO_pin_config.GPIO_pin_number = GPIO_PIN_12;
	GPIO_init(&SPIpin);
}

void SPI2_init(void)
{
	SPI_handle_t spi2handle;

	spi2handle.pSPIx_base_addr = SPI2;
	spi2handle.SPI_config.SPI_bus_config = SPI_BUS_FULL_DUPLX;
	spi2handle.SPI_config.SPI_device_mode = SPI_MASTER_MODE;
	spi2handle.SPI_config.SPI_sclk_speed = SPI_SCLK_DIV2;
	spi2handle.SPI_config.SPI_dff = SPI_DFF_8BITS;
	spi2handle.SPI_config.SPI_CPOL = SPI_CPOL_LOW;
	spi2handle.SPI_config.SPI_CPHA = SPI_CPHA_LOW;
	spi2handle.SPI_config.SPI_SSM = SPI_SSM_EN;

	// clock enable
	SPI_clk_ctrl(SPI2, ENABLE);

	SPI_init(&spi2handle);
}
int main(void)
{
	char user_data[] = "Hello World";
	SPI2_GPIOpin_init();

	SPI2_init();

	// since we use software slave mgmt, set SSI bit to drive NSS to high
	SPI_SSI_config(SPI2, ENABLE);

	// after all control registers are init , then enable the peripheral
	SPI_peripheral_control(SPI2, ENABLE);
	SPI_send_data(SPI2, (uint8_t*)user_data, strlen(user_data));

	// disable after SPI is idle
	while (get_spi_status(SPI2, SPI_BUSY_FLAG));
	SPI_peripheral_control(SPI2, DISABLE);
	while(1);
	return 0;
}
