/*
 * stm32f407_uart_driver.h
 *
 *  Created on: Oct 9, 2022
 *      Author: kunal.chauhan
 */

#ifndef INC_STM32F407_UART_DRIVER_H_
#define INC_STM32F407_UART_DRIVER_H_

#include "stm32f407.h"

//config struct
typedef struct
{
	uint8_t 	USART_Mode;
	uint32_t 	USART_Baud;
	uint8_t 	USART_NumberOfStopBits;
	uint8_t 	USART_WordLength;
	uint8_t 	USART_ParityControl;
	uint8_t 	USART_HWFlowControl;
}USART_config_t;


//handle struct
typedef struct
{
	USART_RegDef_t *pUSARTx_base_addr;
	USART_config_t  USART_config;
	uint32_t 		TxLen;
	uint32_t 		RxLen;
	uint8_t 		TxBusyState;
	uint8_t 		RxBusyState;
	uint8_t 	   *pTxBuffer;
	uint8_t 	   *pRxBuffer;
}USART_handle_t;

// USART mode
#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2

// USART Speed
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


// USART parity control
#define USART_PARITY_EN_ODD    2U
#define USART_PARITY_EN_EVEN   1U
#define USART_PARITY_DISABLE   0U

// USART word length
#define USART_WORDLEN_8BITS  0U
#define USART_WORDLEN_9BITS  1U

// USART Number oif stiop bits
#define USART_STOPBITS_1     0U
#define USART_STOPBITS_0_5   1U
#define USART_STOPBITS_2     2U
#define USART_STOPBITS_1_5   3


// USART HW Flow control
#define USART_HW_FLOW_CTRL_NONE    	0U
#define USART_HW_FLOW_CTRL_CTS    	1U
#define USART_HW_FLOW_CTRL_RTS    	2U
#define USART_HW_FLOW_CTRL_CTS_RTS	3U


// USART Flags
#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

// USART APP states
#define USART_BUSY_IN_RX 	1U
#define USART_BUSY_IN_TX 	2U
#define USART_READY 		0U

// USART events
#define 	USART_EVENT_TX_CMPLT   0U
#define		USART_EVENT_RX_CMPLT   1U
#define		USART_EVENT_IDLE       2U
#define		USART_EVENT_CTS        3U
#define		USART_EVENT_PE         4U
#define		USART_ERR_FE     	   5U
#define		USART_ERR_NE    	   6U
#define		USART_ERR_ORE    	   7U
/*********************** API supported through USART driver ***************************/

// Initialize and reset
void USART_init(USART_handle_t *pUSART_handle);
void USART_deinit(USART_RegDef_t *pUSART_base_addr);

// clock control
void USART_clk_ctrl(USART_RegDef_t *pUSART_base_addr, uint8_t en_or_dis);
uint32_t get_rcc_pclk1_val(void);
uint32_t get_rcc_pclk2_val(void);
// Data send and receive
void USART_master_tx(USART_handle_t *pUSART_handle, uint8_t *pTxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start);
void USART_master_rx(USART_handle_t *pUSART_handle, uint8_t *pRxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start);
uint8_t USART_master_tx_interrupt(USART_handle_t *pUSART_handle, uint8_t *pTxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start);
uint8_t USART_master_rx_interrupt(USART_handle_t *pUSART_handle, uint8_t *pRxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start);


// interrupt handling
void USART_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_dis);
void USART_irq_priority_config(uint8_t irq_number, uint8_t irq_priority);
void USART_irq_handling(USART_handle_t *pUSART_handle);

// control APIs
uint8_t get_USART_status(USART_RegDef_t *pUSART_base_addr, uint32_t flag_name);
void USART_peripheral_control(USART_RegDef_t *pUSART_base_addr, uint8_t en_or_dis);
void USART_set_baud_rate(USART_RegDef_t *pUSART_base_addr, uint32_t baud_rate);

// application callback
void USART_application_callback(USART_handle_t *pUSART_handle, uint8_t event );
#endif /* INC_STM32F407_UART_DRIVER_H_ */
