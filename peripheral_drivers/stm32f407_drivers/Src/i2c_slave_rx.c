/*
 * i2c_rx_test.c
 *
 *  Created on: Oct 7, 2022
 *      Author: kunal.chauhan
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407_i2c_driver.h"
#include "stm32f407_gpio_driver.h"



#define SLAVE_ADDR  0x69
#define MY_ADDR SLAVE_ADDR

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_handle_t I2C1Handle;

//some data
uint8_t tx_buffer[32] = "STM32 slave send data";
/*
 * PB6-> SCL
 * PB9 or PB7 -> SDA
 */


void I2C1_GPIOInits(void)
{
	GPIO_handle_t I2CPins;

	/*Note : Internal pull-up resistors are used */

	I2CPins.pGPIOx_base_addr = GPIOB;
	I2CPins.GPIO_pin_config.GPIO_pin_mode = GPIO_MODE_ALTFUN;
	I2CPins.GPIO_pin_config.GPIO_pin_optype = GPIO_OP_TYPE_OD;
	/*
	 * Note : In the below line use GPIO_NO_PUPD option if you want to use external pullup resistors, then you have to use 3.3K pull up resistors
	 * for both SDA and SCL lines
	 */
	I2CPins.GPIO_pin_config.GPIO_pin_pupdcontrol = GPIO_PIN_PU;
	I2CPins.GPIO_pin_config.GPIO_pin_altfuncmode = 4;
	I2CPins.GPIO_pin_config.GPIO_pin_speed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_pin_config.GPIO_pin_number = GPIO_PIN_6;
	GPIO_init(&I2CPins);


	//sda
	//Note : since we found a glitch on PB9 , you can also try with PB7
	I2CPins.GPIO_pin_config.GPIO_pin_number = GPIO_PIN_7;

	GPIO_init(&I2CPins);


}
void I2C1_Inits(void)
{
	I2C1Handle.pI2C_base_addr = I2C1;
	I2C1Handle.I2C_config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_init(&I2C1Handle);

}



int main(void)
{

	GPIO_clk_ctrl(GPIOB, ENABLE);
	//GPIO_deinit(GPIOB);
	I2C1_GPIOInits();

	//clk control
	I2C_clk_ctrl(I2C1, ENABLE);
	//i2c peripheral configuration
	I2C1_Inits();

	//enable interrupts
	I2C_irq_interrupt_config(IRQ_I2C1_EV, ENABLE);
	I2C_irq_interrupt_config(IRQ_I2C1_ER, ENABLE);

	I2C_slave_en_di_callback_ev(I2C1, ENABLE);

	//enable the i2c peripheral
	I2C_peripheral_control(I2C1,ENABLE);

	// enable ack bit when PE=1. peripheral is enalbed.
	I2C_manage_ack(I2C1, ENABLE);

	while(1)
	{

	}

}

void I2C1_EV_IRQHandler(void)
{
	I2C_event_irq_handling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_error_irq_handling(&I2C1Handle);
}

// application callback
void I2C_application_callback(I2C_handle_t *pI2C_handle, uint8_t event )
{
	static uint8_t commandcode=0;
	static uint8_t index = 0;
	if (event == I2C_EV_DATA_REQ)
	{
		// master request data from slave. slave sends
		if (commandcode == 0x51)
		{
			I2C_slave_tx(pI2C_handle->pI2C_base_addr, strlen((char*)tx_buffer));
		}
		else if (commandcode == 0x52)
		{
			I2C_slave_tx(pI2C_handle->pI2C_base_addr, tx_buffer[index++]);
		}

	}
	else if (event == I2C_EV_DATA_RCV)
	{
		// master sends data. slave reads
		commandcode = I2C_slave_rx(pI2C_handle->pI2C_base_addr);
	}
	else if (event == I2C_ERROR_AF)
	{
		// mster send nack so stop
		commandcode = 0xff;
		index = 0;
	}
	else if (event == I2C_EV_STOP)
	{
		// master ended i2c. only during slave rx
	}
}

