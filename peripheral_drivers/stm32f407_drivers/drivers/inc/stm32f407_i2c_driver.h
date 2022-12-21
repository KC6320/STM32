/*
 * stm32f407_i2c_driver.h
 *
 *  Created on: Aug 31, 2022
 *      Author: kunal.chauhan
 */

#ifndef INC_STM32F407_I2C_DRIVER_H_
#define INC_STM32F407_I2C_DRIVER_H_

#include "stm32f407.h"


typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

}I2C_config_t;


//Handle structure for I2Cx peripheral
typedef struct
{
	I2C_RegDef_t 	*pI2C_base_addr;
	I2C_config_t 	I2C_config;
	uint8_t 		*pTxBuffer;
	uint8_t 		*pRxBuffer;
	uint32_t 		TxLen;
	uint32_t 		RxLen;
	uint8_t 		TxRxState; // only one state since i2c is half duplex
	uint8_t 		DevAddr;   // slave or device address
    uint32_t        RxSize;
    uint8_t         Sr;        // repeated start
}I2C_handle_t;

//application states
#define I2C_READY 					0U
#define I2C_BUSY_IN_RX 				1U
#define I2C_BUSY_IN_TX 				2U

// speed
#define I2C_SCL_SPEED_SM 	100000U
#define I2C_SCL_SPEED_FM4K 	400000U
#define I2C_SCL_SPEED_FM2K  200000U

//ACK control
#define I2C_ACK_ENABLE        1U
#define I2C_ACK_DISABLE       0U

// FMDutyCycle
#define I2C_FM_DUTY_2        0U
#define I2C_FM_DUTY_16_9     1U

//status flags definitions
#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

#define I2C_MASTER_WRITE		1U
#define I2C_MASTER_READ			0U
#define I2C_NO_REPEATED_START 	RESET
#define I2C_REPEATED_START 		SET

//application events macros
#define I2C_EV_TX_CMPLT  	 	0U
#define I2C_EV_RX_CMPLT  	 	1U
#define I2C_EV_STOP       		2U
#define I2C_ERROR_BERR 	 		3U
#define I2C_ERROR_ARLO  		4U
#define I2C_ERROR_AF    		5U
#define I2C_ERROR_OVR   		6U
#define I2C_ERROR_TIMEOUT 		7U
#define I2C_EV_DATA_REQ         8U
#define I2C_EV_DATA_RCV         9U

/*********************** API supported through I2C driver ***************************/

// Initialize and reset
void I2C_init(I2C_handle_t *pI2C_handle);
void I2C_deinit(I2C_RegDef_t *pI2C_base_addr);

// clock control
void I2C_clk_ctrl(I2C_RegDef_t *pI2C_base_addr, uint8_t en_or_dis);

// Data send and receive
void I2C_master_tx(I2C_handle_t *pI2C_handle, uint8_t *pTxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start);
void I2C_master_rx(I2C_handle_t *pI2C_handle, uint8_t *pRxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start);
uint8_t I2C_master_tx_interrupt(I2C_handle_t *pI2C_handle, uint8_t *pTxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start);
uint8_t I2C_master_rx_interrupt(I2C_handle_t *pI2C_handle, uint8_t *pRxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start);

void I2C_slave_tx(I2C_RegDef_t *pI2C_base_addr, uint8_t data);
uint8_t I2C_slave_rx(I2C_RegDef_t *pI2C_base_addr);

// interrupt handling
void I2C_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_dis);
void I2C_irq_priority_config(uint8_t irq_number, uint8_t irq_priority);
void I2C_event_irq_handling(I2C_handle_t *pI2C_handle);
void I2C_error_irq_handling(I2C_handle_t *pI2C_handle);

// control APIs
uint8_t get_I2C_status(I2C_RegDef_t *pI2C_base_addr, uint32_t flag_name);
void I2C_peripheral_control(I2C_RegDef_t *pI2C_base_addr, uint8_t en_or_dis);
void I2C_manage_ack(I2C_RegDef_t *pI2C_base_addr, uint8_t en_or_dis);
void I2C_close_tx(I2C_handle_t *pI2C_handle);
void I2C_close_rx(I2C_handle_t *pI2C_handle);
void I2C_generate_stop(I2C_RegDef_t *pI2C_base_addr);
void I2C_slave_en_di_callback_ev(I2C_RegDef_t *pI2C_base_addr, uint8_t en_or_dis);

// application callback
void I2C_application_callback(I2C_handle_t *pI2C_handle, uint8_t event );
#endif /* INC_STM32F407_I2C_DRIVER_H_ */
