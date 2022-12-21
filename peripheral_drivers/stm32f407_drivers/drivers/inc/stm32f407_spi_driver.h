/*
 * stm32f407_spi_driver.h
 *
 *  Created on: Jul 28, 2022
 *      Author: kunal.chauhan
 */

#ifndef INC_STM32F407_SPI_DRIVER_H_
#define INC_STM32F407_SPI_DRIVER_H_

#include "stm32f407.h"

// SPI config struct
typedef struct
{
	uint8_t SPI_device_mode;
	uint8_t SPI_bus_config;
	uint8_t SPI_sclk_speed;
	uint8_t SPI_dff;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_config_t;


// SPI handle struct
typedef struct
{
	SPI_RegDef_t 		*pSPIx_base_addr;		// pointer to base address of SPI peripheral
	SPI_config_t 	 	 SPI_config;			// config struct
	uint8_t				*pTxbuffer;				// store tx buffer addr
	uint8_t				*pRxbuffer;				// store rx buffer addr
	uint32_t			 tx_len;				// store tx len
	uint32_t			 rx_len;				// store rx len
	uint8_t				 tx_state;				// store tx state
	uint8_t				 rx_state;				// store tx state
}SPI_handle_t;

// device mode
#define SPI_MASTER_MODE		1U
#define SPI_SLAVE_MODE		0U

// bus config
#define SPI_BUS_FULL_DUPLX  1U
#define SPI_BUS_HALF_DUPLX	2U
#define SPI_BUS_SIMPLEX_RX	3U

// clock speed
#define SPI_SCLK_DIV2		0U
#define SPI_SCLK_DIV4		1U
#define SPI_SCLK_DIV8		2U
#define SPI_SCLK_DIV16		3U
#define SPI_SCLK_DIV32		4U
#define SPI_SCLK_DIV64		5U
#define SPI_SCLK_DIV128		6U
#define SPI_SCLK_DIV256		7U

// Data frame format
#define SPI_DFF_8BITS		0U
#define SPI_DFF_16BITS		1U

// CPOL
#define SPI_CPOL_HIGH		1U
#define SPI_CPOL_LOW		0U

//CPHA
#define SPI_CPHA_HIGH		1U
#define SPI_CPHA_LOW		0U

// slave select
#define SPI_SSM_DI			0U
#define SPI_SSM_EN			1U

// SPI flags
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)

// SPI application states
#define SPI_READY			0U
#define SPI_IN_TX			1U
#define SPI_IN_RX			2U

// SPI events
#define SPI_EVENT_TX_CMPLT 	1U
#define SPI_EVENT_RX_CMPLT 	2U
#define SPI_EVENT_OVR_ERR 	3U


/*********************** API supported through SPI driver ***************************/

// Initialize and reset
void SPI_init(SPI_handle_t *pSPI_handle);
void SPI_deinit(SPI_RegDef_t *pSPIx_base_addr);

// clock control
void SPI_clk_ctrl(SPI_RegDef_t *pSPIx_base_addr, uint8_t en_or_dis);

// Data send and receive
void SPI_send_data(SPI_RegDef_t *pSPIx_base_addr, uint8_t *pTxBuffer, uint32_t length);
void SPI_receive_data(SPI_RegDef_t *pSPIx_base_addr, uint8_t *pRxBuffer, uint32_t length);
// interrupt based send and receive
uint8_t SPI_interrupt_send_data(SPI_handle_t *pSPI_handle, uint8_t *pTxBuffer, uint32_t length);
uint8_t SPI_interrupt_receive_data(SPI_handle_t *pSPI_handle, uint8_t *pRxBuffer, uint32_t length);

// interrupt handling
void SPI_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_dis);
void SPI_irq_priority_config(uint8_t irq_number, uint8_t irq_priority);
void SPI_irq_handling(SPI_handle_t *pSPI_handle);

// control APIs
uint8_t get_spi_status(SPI_RegDef_t *pSPIx_base_addr, uint32_t flag_name);
void SPI_peripheral_control(SPI_RegDef_t *pSPIx_base_addr, uint8_t en_or_dis);
void SPI_SSI_config(SPI_RegDef_t *pSPIx_base_addr, uint8_t en_or_dis);
void SPI_SSI_config(SPI_RegDef_t *pSPIx_base_addr, uint8_t en_or_dis);
void SPI_close_tx(SPI_handle_t *pSPI_handle);
void SPI_close_rx(SPI_handle_t *pSPI_handle);

// application callback
void spi_application_callback(SPI_handle_t *pSPI_handle, uint8_t en_or_dis );

#endif /* INC_STM32F407_SPI_DRIVER_H_ */
