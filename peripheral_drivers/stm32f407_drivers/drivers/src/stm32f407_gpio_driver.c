/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Jul 20, 2022
 *      Author: kunal.chauhan
 */

#include "stm32f407_gpio_driver.h"

/*********************** API supported through GPIO driver ***************************/

// Initialize and reset
void GPIO_init(GPIO_handle_t *pGPIO_handle)
{
	uint32_t value=0;
	// configure gpio mode
	if (pGPIO_handle->GPIO_pin_config.GPIO_pin_mode <= GPIO_MODE_ANALOG)
	{
		value = (pGPIO_handle->GPIO_pin_config.GPIO_pin_mode << (pGPIO_handle->GPIO_pin_config.GPIO_pin_number *2));
		//clear
		pGPIO_handle->pGPIOx_base_addr->MODER &= ~(0x3 << (pGPIO_handle->GPIO_pin_config.GPIO_pin_number *2));
		//set
		pGPIO_handle->pGPIOx_base_addr->MODER |= value;
	}
	else
	{
		// interrupt mode
		// 1. set the edge detection
		if (pGPIO_handle->GPIO_pin_config.GPIO_pin_mode == GPIO_MODE_IP_FT)
		{
			// set FTSR bit
			EXTI->FTSR |= (1 << pGPIO_handle->GPIO_pin_config.GPIO_pin_number);
			// for safety clear RTSR corresponding bit if set by some other code
			EXTI->RTSR &= ~(1 << pGPIO_handle->GPIO_pin_config.GPIO_pin_number);
		}
		else if (pGPIO_handle->GPIO_pin_config.GPIO_pin_mode == GPIO_MODE_IP_RT)
		{
			// set RTSR bit
			EXTI->RTSR |= (1 << pGPIO_handle->GPIO_pin_config.GPIO_pin_number);
			// for safety clear FTSR corresponding bit if set by some other code
			EXTI->FTSR &= ~(1 << pGPIO_handle->GPIO_pin_config.GPIO_pin_number);
		}
		else if (pGPIO_handle->GPIO_pin_config.GPIO_pin_mode == GPIO_MODE_IP_RFT)
		{
			// set FTSR and RTSR bit
			EXTI->FTSR |= (1 << pGPIO_handle->GPIO_pin_config.GPIO_pin_number);
			EXTI->RTSR |= (1 << pGPIO_handle->GPIO_pin_config.GPIO_pin_number);
		}

		// 2. configure GPIO port selection in syscfgr
		// 2.1 enable syscfg clk
		SYSCFG_PCLK_EN();
		// 2.2
		uint8_t reg_number = pGPIO_handle->GPIO_pin_config.GPIO_pin_number / 4;
		uint8_t position = pGPIO_handle->GPIO_pin_config.GPIO_pin_number % 4;
		uint8_t portcode = GET_PORT_CODE_FROM_GPIO_BASE_ADDR(pGPIO_handle->pGPIOx_base_addr); // specific to EXTI
		SYSCFG->EXTICR[reg_number] |= (portcode << (4*position));

		// 3. configure the interrupt mask
		EXTI->IMR |= (1 << pGPIO_handle->GPIO_pin_config.GPIO_pin_number);
	}


	// configure speed
	value = (pGPIO_handle->GPIO_pin_config.GPIO_pin_speed << (pGPIO_handle->GPIO_pin_config.GPIO_pin_number *2));
	//clear
	pGPIO_handle->pGPIOx_base_addr->OSPEEDR &= ~(0x3 << (pGPIO_handle->GPIO_pin_config.GPIO_pin_number *2));
	//set
	pGPIO_handle->pGPIOx_base_addr->OSPEEDR |= value;


	// configure pull up/pull down
	value = (pGPIO_handle->GPIO_pin_config.GPIO_pin_pupdcontrol << (pGPIO_handle->GPIO_pin_config.GPIO_pin_number *2));
	//clear
	pGPIO_handle->pGPIOx_base_addr->PUPDR &= ~(0x3 << (pGPIO_handle->GPIO_pin_config.GPIO_pin_number *2));
	//set
	pGPIO_handle->pGPIOx_base_addr->PUPDR |= value;

	// configure o/p type
	value = (pGPIO_handle->GPIO_pin_config.GPIO_pin_optype << (pGPIO_handle->GPIO_pin_config.GPIO_pin_number));
	//clear
	pGPIO_handle->pGPIOx_base_addr->OTYPER &= ~(0x1 << pGPIO_handle->GPIO_pin_config.GPIO_pin_number);
	//set
	pGPIO_handle->pGPIOx_base_addr->OTYPER |= value;

	// configure alternate functionality
	if (pGPIO_handle->GPIO_pin_config.GPIO_pin_mode == GPIO_MODE_ALTFUN)
	{
		uint8_t alt_reg = pGPIO_handle->GPIO_pin_config.GPIO_pin_number / 8;
		uint8_t position = pGPIO_handle->GPIO_pin_config.GPIO_pin_number % 8;
		value = (pGPIO_handle->GPIO_pin_config.GPIO_pin_altfuncmode <<(4*position));
		//clear
		pGPIO_handle->pGPIOx_base_addr->AFR[alt_reg] &= ~(0xF << (4*position));
		//set
		pGPIO_handle->pGPIOx_base_addr->AFR[alt_reg] |= value;
	}

}
void GPIO_deinit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

// clock control
void GPIO_clk_ctrl(GPIO_RegDef_t *pGPIOx, uint8_t en_or_dis)
{
	if(en_or_dis == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

// read functions
uint8_t  GPIO_read_input_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number)
{
	return (uint8_t)((pGPIOx->IDR >> pin_number) & 0x00000001U);
}

uint16_t GPIO_read_input_port(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)(pGPIOx->IDR);
}

// write functions
void GPIO_write_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		// write 1
		pGPIOx->ODR |= (1 << pin_number);
	}
	else
	{
		// write 0
		pGPIOx->ODR &= ~(1 << pin_number);
	}
}

void GPIO_write_output_port(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

//toggle pin
void GPIO_toggle_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number)
{
	pGPIOx->ODR ^= (1 << pin_number);
}

// interrupt handling and config
void GPIO_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_dis)
{
	if(en_or_dis == ENABLE)
	{
		if(irq_number <= 31)
		{
			// configure ISER0 reg
			*NVIC_ISER0 |= (1 << irq_number);
		}
		else if(irq_number > 31 && irq_number < 64)
		{
			// configure ISER1 reg
			*NVIC_ISER1 |= (1 << (irq_number % 32));
		}
		else if(irq_number >= 64 && irq_number < 96)
		{
			// configure ISER2 reg
			*NVIC_ISER2 |= (1 << (irq_number % 64));
		}
	}
	else
	{
		if(irq_number <= 31)
		{
			// configure ICER0 reg
			*NVIC_ICER0 |= (1 << irq_number);
		}
		else if(irq_number > 31 && irq_number < 64)
		{
			// configure ICER1 reg
			*NVIC_ICER1 |= (1 << (irq_number % 32));
		}
		else if(irq_number >= 64 && irq_number < 96)
		{
			// configure ICER2 reg
			*NVIC_ICER2 |= (1 << (irq_number % 64));
		}
	}
}
void GPIO_irq_priority_config(uint8_t irq_number, uint8_t irq_priority)
{
	uint8_t reg_number = irq_number / 4;
	uint8_t bit_position = irq_number % 4;
	uint8_t shift_amount = (8* bit_position) + ( 8 - PRIORITY_BITS_IMPLEMENTED);// only upper nibble of each byte is allocated for priority
	*(NVIC_IPR_BASE_ADDR + (4*reg_number)) |= (irq_priority << shift_amount);
}
void GPIO_irq_handling(uint8_t pin_number)
{
	// pending bit is automatically set when interrupt occurs
	// clear pending bit in EXTI reg to avoid infinite ISR. specific case for EXTI peripheral
	if (EXTI->PR & (1<<pin_number))
	{
		EXTI->PR |= (1<<pin_number);
	}

}
