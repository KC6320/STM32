/*
 * stm32f407.h
 *
 *  Created on: Jul 19, 2022
 *      Author: kunal.chauhan
 */
#include <stdint.h>
#include <stddef.h>

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_

/**************************** ARM Processer specific details *********************************/

// NVIC interrupt enable registers
#define NVIC_ISER0					((volatile uint32_t*)0xE000E100U)
#define NVIC_ISER1					((volatile uint32_t*)0xE000E104U)
#define NVIC_ISER2					((volatile uint32_t*)0xE000E108U)
#define NVIC_ISER3					((volatile uint32_t*)0xE000E10CU)
#define NVIC_ISER4					((volatile uint32_t*)0xE000E110U)
#define NVIC_ISER5					((volatile uint32_t*)0xE000E114U)
#define NVIC_ISER6					((volatile uint32_t*)0xE000E118U)
#define NVIC_ISER7					((volatile uint32_t*)0xE000E11CU)

// NVIC interrupt clear registers
#define NVIC_ICER0					((volatile uint32_t*)0xE000E180U)
#define NVIC_ICER1					((volatile uint32_t*)0xE000E184U)
#define NVIC_ICER2					((volatile uint32_t*)0xE000E188U)
#define NVIC_ICER3					((volatile uint32_t*)0xE000E18CU)
#define NVIC_ICER4					((volatile uint32_t*)0xE000E190U)
#define NVIC_ICER5					((volatile uint32_t*)0xE000E194U)
#define NVIC_ICER6					((volatile uint32_t*)0xE000E198U)
#define NVIC_ICER7					((volatile uint32_t*)0xE000E19CU)

// NVIC priority registers
#define NVIC_IPR_BASE_ADDR			((volatile uint32_t*)0xE000E400U)
#define PRIORITY_BITS_IMPLEMENTED	4U							 	// lower nibble is not allocated


#define FLASH_BASE_ADDR				0x08000000U						 // base address of flash memory
#define SRAM1_BASE_ADDR				0x20000000U						 // base address of sram 1, size 112KB
#define SRAM2_BASE_ADDR				(SRAM1_BASE_ADDR + 0x0001C000U)	 // base address of sram 2, size 16KB after SRAM1 end
#define SYSTEM_MEM_BASE_ADDR		0x1FFF0000U						 // base address of system memory ROM. part of flash mem

// AHB high speed and APB low speed peripheral base addresses

#define PERIPHERAL_BASE_ADDR 		0x40000000U								// peripheral start in memory map
#define APB1_PERIPH_BASE_ADDR		PERIPHERAL_BASE_ADDR					// peripherals hanging on APB1 bus
#define APB2_PERIPH_BASE_ADDR		(PERIPHERAL_BASE_ADDR + 0x00010000U)	// peripherals hanging on APB2 bus
#define AHB1_PERIPH_BASE_ADDR		(PERIPHERAL_BASE_ADDR + 0x00020000U)	// peripherals hanging on AHB1 bus
#define AHB2_PERIPH_BASE_ADDR		(PERIPHERAL_BASE_ADDR + 0x10000000U)	// peripherals hanging on AHB2 bus

// base addresses of peripheral on AHB1 bus
#define GPIOA_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x0000U)
#define GPIOB_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x0400U)
#define GPIOC_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x0800U)
#define GPIOD_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x0C00U)
#define GPIOE_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x1000U)
#define GPIOF_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x1400U)
#define GPIOG_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x1800U)
#define GPIOH_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x1C00U)
#define GPIOI_BASE_ADDR 			(AHB1_PERIPH_BASE_ADDR + 0x2000U)
#define RCC_BASE_ADDR				(AHB1_PERIPH_BASE_ADDR + 0x3800U)

// base addresses of peripheral on APB1 bus
#define I2C1_BASE_ADDR 				(APB1_PERIPH_BASE_ADDR + 0x5400U)
#define I2C2_BASE_ADDR 				(APB1_PERIPH_BASE_ADDR + 0x5800U)
#define I2C3_BASE_ADDR 				(APB1_PERIPH_BASE_ADDR + 0x5C00U)

#define SPI2_BASE_ADDR 				(APB1_PERIPH_BASE_ADDR + 0x3800U)
#define SPI3_BASE_ADDR 				(APB1_PERIPH_BASE_ADDR + 0x3C00U)

#define USART2_BASE_ADDR 			(APB1_PERIPH_BASE_ADDR + 0x4400U)
#define USART3_BASE_ADDR 			(APB1_PERIPH_BASE_ADDR + 0x4800U)

#define UART4_BASE_ADDR 			(APB1_PERIPH_BASE_ADDR + 0x4C00U)
#define UART5_BASE_ADDR 			(APB1_PERIPH_BASE_ADDR + 0x5000U)

// base addresses of peripheral in APB2 bus
#define SPI1_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x3000U)

#define USART1_BASE_ADDR			(APB2_PERIPH_BASE_ADDR + 0x1000U)
#define USART6_BASE_ADDR			(APB2_PERIPH_BASE_ADDR + 0x1400U)

#define EXTI_BASE_ADDR				(APB2_PERIPH_BASE_ADDR + 0x3C00U)

#define SYSCFG_BASE_ADDR			(APB2_PERIPH_BASE_ADDR + 0x3800U)

/******************************* peripheral registers definition ********************************/

// peripheral register defintion for GPIO
typedef struct
{
	volatile uint32_t MODER;		// GPIO port mode register
	volatile uint32_t OTYPER;		// GPIO port output type register
	volatile uint32_t OSPEEDR;		// GPIO port output speed register
	volatile uint32_t PUPDR;		// GPIO port pull-up/pull-down register
	volatile uint32_t IDR;			// GPIO port input data register
	volatile uint32_t ODR;			// GPIO port output data register
	volatile uint32_t BSRR;			// GPIO port bit set/reset register
	volatile uint32_t LCKR;			// GPIO port configuration lock register
	volatile uint32_t AFR[2];		// AFR[0] GPIO alternate function low register, AFR[1] GPIO alternate function high register
}GPIO_RegDef_t;

// peripheral register definition for RCC
typedef struct
{
	volatile uint32_t CR;			//RCC clock control register
	volatile uint32_t PLLCFGR;		//RCC PLL configuration register
	volatile uint32_t CFGR;			//RCC clock configuration register
	volatile uint32_t CIR;			//RCC clock interrupt register
	volatile uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register
	volatile uint32_t AHB2RSTR;		//RCC AHB2 peripheral reset register
	volatile uint32_t AHB3RSTR;		//RCC AHB3 peripheral reset register
			 uint32_t RESERVED1;
	volatile uint32_t APB1RSTR;		//RCC APB1 peripheral reset register
	volatile uint32_t APB2RSTR;		//RCC APB2 peripheral reset register
			 uint32_t RESERVED2;
			 uint32_t RESERVED3;
	volatile uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register
	volatile uint32_t AHB2ENR;		//RCC AHB2 peripheral clock enable register
	volatile uint32_t AHB3ENR;		//RCC AHB3 peripheral clock enable register
			 uint32_t RESERVED4;
	volatile uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register
	volatile uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register
			 uint32_t RESERVED5;
			 uint32_t RESERVED6;
	volatile uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register
	volatile uint32_t AHB2LPENR;	//RCC AHB2 peripheral clock enable in low power mode register
	volatile uint32_t AHB3LPENR;	//RCC AHB3 peripheral clock enable in low power mode register
			 uint32_t RESERVED7;
	volatile uint32_t APB1LPENR;	//RCC APB1 peripheral clock enable in low power mode register
	volatile uint32_t APB2LPENR;	//RCC APB2 peripheral clock enable in low power mode register
			 uint32_t RESERVED8;
			 uint32_t RESERVED9;
	volatile uint32_t BDCR;			//RCC Backup domain control register
	volatile uint32_t CSR;			//RCC clock control & status register
			 uint32_t RESERVED10;
			 uint32_t RESERVED11;
	volatile uint32_t SSCGR;		//RCC spread spectrum clock generation register
	volatile uint32_t PLLI2SCFGR;	//RCC PLLI2S configuration register
}RCC_RegDef_t;

// peripheral register defintion for EXTI
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

// peripheral register defintion for SYSCFG
typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
			 uint32_t RESERVED0;
			 uint32_t RESERVED1;
	volatile uint32_t CMPCR;
}SYSCFG_RegDef_t;

// peripheral register defintion for SPI
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

// peripheral register defintion for I2C
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;


//uart

typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_RegDef_t;


// peripheral definition macro typcasted
#define GPIOA						((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB						((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC						((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD						((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE						((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF						((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG						((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH						((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI						((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASE_ADDR)
#define EXTI						((EXTI_RegDef_t*)EXTI_BASE_ADDR)
#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

#define SPI1						((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASE_ADDR)

#define I2C1						((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2						((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define I2C3						((I2C_RegDef_t*)I2C3_BASE_ADDR)

#define USART1  					((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2  					((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3  					((USART_RegDef_t*)USART3_BASE_ADDR)
#define UART4  						((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5  						((USART_RegDef_t*)UART5_BASE_ADDR)
#define USART6  					((USART_RegDef_t*)USART6_BASE_ADDR)

/********************** Clock Enable Macros ******************************/

//GPIO
#define GPIOA_PCLK_EN()				(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()				(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()				(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()				(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()				(RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()				(RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()				(RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()				(RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()				(RCC->AHB1ENR |= (1<<8))

//I2C
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1<<23))

//SPI
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))

//USART
#define USART1_PCL_EN()				(RCC->APB2ENR |= (1<<4))
#define USART2_PCL_EN()				(RCC->APB1ENR |= (1<<17))
#define USART3_PCL_EN()				(RCC->APB1ENR |= (1<<18))
#define UART4_PCL_EN()				(RCC->APB1ENR |= (1<<19))
#define UART5_PCL_EN()				(RCC->APB1ENR |= (1<<20))
#define USART6_PCL_EN()				(RCC->APB2ENR |= (1<<5))

//SYSCFG
#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1<<14))

/********************** Clock disable Macros ******************************/

//GPIO
#define GPIOA_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()				(RCC->AHB1ENR &= ~(1<<8))

//I2C
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<23))

//SPI
#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1<<15))

//USART
#define USART1_PCL_DI()				(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCL_DI()				(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCL_DI()				(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCL_DI()				(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCL_DI()				(RCC->APB1ENR &= ~(1<<20))
#define USART6_PCL_DI()				(RCC->APB2ENR &= ~(1<<5))

//SYSCFG
#define SYSCFG_PCLK_DI()			(RCC->APB2ENR &= ~(1<<14))

// reset GPIO peripherals
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));}while(0)

// reset SPI peripherals
#define SPI1_REG_RESET()			do{ (RCC->APB2RSTR |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0)

// reset I2C peripherals
#define I2C1_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<21)); (RCC->APB2RSTR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23));}while(0)

// reset USART/UART peripherals
#define USART1_REG_RESET()			do{ (RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4));}while(0)
#define USART2_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<17)); (RCC->APB1RSTR &= ~(1<<17));}while(0)
#define USART3_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<18)); (RCC->APB1RSTR &= ~(1<<18));}while(0)
#define UART4_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<19)); (RCC->APB2RSTR &= ~(1<<19));}while(0)
#define UART5_REG_RESET()			do{ (RCC->APB1RSTR |= (1<<20)); (RCC->APB1RSTR &= ~(1<<20));}while(0)
#define USART6_REG_RESET()			do{ (RCC->APB2RSTR |= (1<<5)); (RCC->APB1RSTR &= ~(1<<5));}while(0)
// port code macro for EXTI and SYSCFG configuration
#define GET_PORT_CODE_FROM_GPIO_BASE_ADDR(x) 			(x==GPIOA)? 0U:\
														(x==GPIOB)? 1U:\
														(x==GPIOC)? 2U:\
														(x==GPIOD)? 3U:\
														(x==GPIOE)? 4U:\
														(x==GPIOF)? 5U:\
														(x==GPIOG)? 6U:\
														(x==GPIOH)? 7U:\
														(x==GPIOI)? 8U:0U
/**************** BIT position definition of SPI peripheral ***********************/
// CR1 register
#define SPI_CR1_CPHA				0U
#define SPI_CR1_CPOL				1U
#define SPI_CR1_MSTR				2U
#define SPI_CR1_BR					3U
#define SPI_CR1_SPE					6U
#define SPI_CR1_SSI					8U
#define SPI_CR1_SSM					9U
#define SPI_CR1_RXONLY				10U
#define SPI_CR1_DFF					11U
#define SPI_CR1_BIDIOE				14U
#define SPI_CR1_BIDIMODE			15U

// CR1 register
#define SPI_CR2_RXDMAEN				0U
#define SPI_CR2_TXDMAEN				1U
#define SPI_CR2_SSOE				2U
#define SPI_CR2_FRF					4U
#define SPI_CR2_ERREIE				5U
#define SPI_CR2_RXNEIE				6U
#define SPI_CR2_TXEIE				7U

// SPI status register
#define SPI_SR_RXNE					0U
#define SPI_SR_TXE					1U
#define SPI_SR_CHSIDE				2U
#define SPI_SR_UDR					3U
#define SPI_SR_CRCERR				4U
#define SPI_SR_MODF					5U
#define SPI_SR_OVR					6U
#define SPI_SR_BSY					7U
#define SPI_SR_FRE					8U

/**************** BIT position definition of I2C peripheral ***********************/

 //I2C_CR1
#define I2C_CR1_PE						0U
#define I2C_CR1_NOSTRETCH  				7U
#define I2C_CR1_START 					8U
#define I2C_CR1_STOP  				 	9U
#define I2C_CR1_ACK 				 	10U
#define I2C_CR1_SWRST  				 	15U

//I2C_CR2
#define I2C_CR2_FREQ				 	0U
#define I2C_CR2_ITERREN				 	8U
#define I2C_CR2_ITEVTEN				 	9U
#define I2C_CR2_ITBUFEN 			    10

//I2C_OAR1
#define I2C_OAR1_ADD0    				 0U
#define I2C_OAR1_ADD71 				 	 1U
#define I2C_OAR1_ADD98  			 	 8U
#define I2C_OAR1_ADDMODE   			 	15U

//I2C_SR1
#define I2C_SR1_SB 					 	0U
#define I2C_SR1_ADDR 				 	1U
#define I2C_SR1_BTF 					2U
#define I2C_SR1_ADD10 					3U
#define I2C_SR1_STOPF 					4U
#define I2C_SR1_RXNE 					6U
#define I2C_SR1_TXE 					7U
#define I2C_SR1_BERR 					8U
#define I2C_SR1_ARLO 					9U
#define I2C_SR1_AF 					 	10U
#define I2C_SR1_OVR 					11U
#define I2C_SR1_TIMEOUT 				14U


//I2C_SR2
#define I2C_SR2_MSL						0U
#define I2C_SR2_BUSY 					1U
#define I2C_SR2_TRA 					2U
#define I2C_SR2_GENCALL 				4U
#define I2C_SR2_DUALF 					7U

//I2C_CCR
#define I2C_CCR_CCR 					 0U
#define I2C_CCR_DUTY 					14U
#define I2C_CCR_FS  				 	15U


/**************** BIT position definition of USART peripheral ***********************/

//USART_CR1
#define USART_CR1_SBK					0U
#define USART_CR1_RWU 					1U
#define USART_CR1_RE  					2U
#define USART_CR1_TE 					3U
#define USART_CR1_IDLEIE 				4U
#define USART_CR1_RXNEIE  				5U
#define USART_CR1_TCIE					6U
#define USART_CR1_TXEIE					7U
#define USART_CR1_PEIE 					8U
#define USART_CR1_PS 					9U
#define USART_CR1_PCE 					10U
#define USART_CR1_WAKE  				11U
#define USART_CR1_M 					12U
#define USART_CR1_UE 					13U
#define USART_CR1_OVER8  				15U



//USART_CR2
#define USART_CR2_ADD   				0U
#define USART_CR2_LBDL   				5U
#define USART_CR2_LBDIE  				6U
#define USART_CR2_LBCL   				8U
#define USART_CR2_CPHA   				9U
#define USART_CR2_CPOL   				10U
#define USART_CR2_STOP   				12U
#define USART_CR2_LINEN   				14U


//USART_CR3
#define USART_CR3_EIE   				0U
#define USART_CR3_IREN   				1U
#define USART_CR3_IRLP  				2U
#define USART_CR3_HDSEL   				3U
#define USART_CR3_NACK   				4U
#define USART_CR3_SCEN   				5U
#define USART_CR3_DMAR  				6U
#define USART_CR3_DMAT   				7U
#define USART_CR3_RTSE   				8U
#define USART_CR3_CTSE   				9U
#define USART_CR3_CTSIE   				10U
#define USART_CR3_ONEBIT   				11U

//USART_SR
#define USART_SR_PE        				0U
#define USART_SR_FE        				1U
#define USART_SR_NE        				2U
#define USART_SR_ORE       				3U
#define USART_SR_IDLE       			4U
#define USART_SR_RXNE        			5U
#define USART_SR_TC        				6U
#define USART_SR_TXE        			7U
#define USART_SR_LBD        			8U
#define USART_SR_CTS        			9U

// macros for IRQ numbers
#define IRQ_EXTI0					6U
#define IRQ_EXTI1					7U
#define IRQ_EXTI2					8U
#define IRQ_EXTI3					9U
#define IRQ_EXTI4					10U
#define IRQ_EXTI9_5					23U
#define IRQ_EXTI15_10				40U
#define IRQ_I2C1_EV     			31U
#define IRQ_I2C1_ER     			32U
#define IRQ_NO_USART1	    		37U
#define IRQ_NO_USART2	    		38U
#define IRQ_NO_USART3	    		39U
#define IRQ_NO_UART4	    		52U
#define IRQ_NO_UART5	    		53U
#define IRQ_NO_USART6	    		71U

// shared macros
#define ENABLE 						1U
#define DISABLE						0U
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				ENABLE
#define GPIO_PIN_RESET				DISABLE
#endif /* INC_STM32F407_H_ */
