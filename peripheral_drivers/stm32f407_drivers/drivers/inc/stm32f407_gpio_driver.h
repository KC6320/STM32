/*
 * stm32f407_gpio_driver.h
 *
 *  Created on: Jul 20, 2022
 *      Author: kunal.chauhan
 */

#ifndef INC_STM32F407_GPIO_DRIVER_H_
#define INC_STM32F407_GPIO_DRIVER_H_

#include "stm32f407.h"

// GPIO config struct
typedef struct
{
	uint8_t GPIO_pin_number;
	uint8_t GPIO_pin_mode;
	uint8_t GPIO_pin_speed;
	uint8_t GPIO_pin_pupdcontrol;
	uint8_t GPIO_pin_optype;
	uint8_t GPIO_pin_altfuncmode;
}GPIO_pin_config_t;


// GPIO handle struct
typedef struct
{
	GPIO_RegDef_t 		*pGPIOx_base_addr;			// pointer to base address of GPIO peripheral where the pin belongs
	GPIO_pin_config_t 	 GPIO_pin_config;			// config struct
}GPIO_handle_t;

// GPIO pin numbers
#define GPIO_PIN_0				0U
#define GPIO_PIN_1				1U
#define GPIO_PIN_2				2U
#define GPIO_PIN_3				3U
#define GPIO_PIN_4				4U
#define GPIO_PIN_5				5U
#define GPIO_PIN_6				6U
#define GPIO_PIN_7				7U
#define GPIO_PIN_8				8U
#define GPIO_PIN_9				9U
#define GPIO_PIN_10				10U
#define GPIO_PIN_11				11U
#define GPIO_PIN_12				12U
#define GPIO_PIN_13				13U
#define GPIO_PIN_14				14U
#define GPIO_PIN_15				15U

// GPIO pin modes
#define GPIO_MODE_IN			0U 				// input
#define GPIO_MODE_OP			1U				// output
#define GPIO_MODE_ALTFUN		2U				// alternate function
#define GPIO_MODE_ANALOG		3U				// analog
#define GPIO_MODE_IP_FT			4U				// interrupt falling trigger
#define	GPIO_MODE_IP_RT			5U				// interrupt rising trigger
#define GPIO_MODE_IP_RFT		6U				// interrupt falling and rising trigger

// GPIO output type
#define GPIO_OP_TYPE_PP			0U				// push pull
#define GPIO_OP_TYPE_OD			1U				// open drain

// GPIO speed
#define GPIO_SPEED_LOW			0U
#define GPIO_SPEED_MED			1U
#define GPIO_SPEED_FAST			2U
#define GPIO_SPEED_HIGH			3U

// GPIO pullup / pull down
#define GPIO_NO_PUPD			0U 				// no pull up no pull down
#define GPIO_PIN_PU				1U				// pull up
#define GPIO_PIN_PD				2U				// pull down



/*********************** API supported through GPIO driver ***************************/

// Initialize and reset
void GPIO_init(GPIO_handle_t *pGPIO_handle);
void GPIO_deinit(GPIO_RegDef_t *pGPIOx);

// clock control
void GPIO_clk_ctrl(GPIO_RegDef_t *pGPIOx, uint8_t en_or_dis);

// read functions
uint8_t  GPIO_read_input_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number);
uint16_t GPIO_read_input_port(GPIO_RegDef_t *pGPIOx);

// write functions
void GPIO_write_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number, uint8_t value);
void GPIO_write_output_port(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_toggle_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number);

// interrupt handling and config
void GPIO_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_dis);
void GPIO_irq_priority_config(uint8_t irq_number, uint8_t irq_priority);
void GPIO_irq_handling(uint8_t pin_number);


#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
