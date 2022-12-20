/*
 * buttonisr.c
 *
 *  Created on: Jul 26, 2022
 *      Author: kunal.chauhan
 */

#include <stdint.h>
#include "stm32f407_gpio_driver.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// delay by software
void delay(void)
{
	for ( uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	// on board led config
	GPIO_handle_t gpio_led;

	gpio_led.pGPIOx_base_addr = 						GPIOD;
	gpio_led.GPIO_pin_config.GPIO_pin_number = 			GPIO_PIN_13;
	gpio_led.GPIO_pin_config.GPIO_pin_mode = 			GPIO_MODE_OP;
	gpio_led.GPIO_pin_config.GPIO_pin_speed = 			GPIO_SPEED_FAST;
	gpio_led.GPIO_pin_config.GPIO_pin_optype = 			GPIO_OP_TYPE_PP;
	gpio_led.GPIO_pin_config.GPIO_pin_pupdcontrol = 	GPIO_NO_PUPD;

	GPIO_clk_ctrl(GPIOD, ENABLE);
	GPIO_init(&gpio_led);

	// on board button config
	GPIO_handle_t gpio_button;

	gpio_button.pGPIOx_base_addr = 							GPIOA;
	gpio_button.GPIO_pin_config.GPIO_pin_number = 			GPIO_PIN_0;
	gpio_button.GPIO_pin_config.GPIO_pin_mode = 			GPIO_MODE_IP_FT;
	gpio_button.GPIO_pin_config.GPIO_pin_speed = 			GPIO_SPEED_FAST;
	gpio_button.GPIO_pin_config.GPIO_pin_pupdcontrol = 		GPIO_NO_PUPD;

	GPIO_clk_ctrl(GPIOA, ENABLE);
	GPIO_init(&gpio_button);


	// irq configurations
	GPIO_irq_interrupt_config(IRQ_EXTI0, ENABLE);

    /* Loop forever */
	while(1);
}

void EXTI0_IRQHandler(void)
{
	// irq handling api from driver
	GPIO_irq_handling(GPIO_PIN_0);
	GPIO_toggle_output_pin(GPIOD, GPIO_PIN_13);
	delay();
}
