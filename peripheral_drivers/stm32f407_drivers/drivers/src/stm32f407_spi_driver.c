/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Jul 28, 2022
 *      Author: kunal.chauhan
 */

#include "stm32f407_spi_driver.h"

// helper functions
static void spi_tx_interrupt_handle(SPI_handle_t *pSPI_handle);
static void spi_rx_interrupt_handle(SPI_handle_t *pSPI_handle);
static void spi_error_interrupt_handle(SPI_handle_t *pSPI_handle);

// Initialize and reset
void SPI_init(SPI_handle_t *pSPI_handle)
{
	if (pSPI_handle->SPI_config.SPI_device_mode == SPI_MASTER_MODE)
	{
		pSPI_handle->pSPIx_base_addr->CR1 |= (1<<SPI_CR1_MSTR);
	}

	if (pSPI_handle->SPI_config.SPI_bus_config == SPI_BUS_FULL_DUPLX)
	{
		// bi directional mode should be cleared
		pSPI_handle->pSPIx_base_addr->CR1 &= ~(1<<SPI_CR1_BIDIMODE);
	}
	else if (pSPI_handle->SPI_config.SPI_bus_config == SPI_BUS_HALF_DUPLX)
	{
		// bi directional mode should be set
		pSPI_handle->pSPIx_base_addr->CR1 |= (1<<SPI_CR1_BIDIMODE);
		// somehow configure BIDIOE bit ?
	}
	else if (pSPI_handle->SPI_config.SPI_bus_config == SPI_BUS_SIMPLEX_RX)
	{
		// bi directional mode should be cleared
		// set rx only bit
		pSPI_handle->pSPIx_base_addr->CR1 &= ~(1<<SPI_CR1_BIDIMODE);
		pSPI_handle->pSPIx_base_addr->CR1 |= (1<<SPI_CR1_RXONLY);
	}

	pSPI_handle->pSPIx_base_addr->CR1 |= (pSPI_handle->SPI_config.SPI_sclk_speed << SPI_CR1_BR);
	pSPI_handle->pSPIx_base_addr->CR1 |= (pSPI_handle->SPI_config.SPI_dff << SPI_CR1_DFF);
	pSPI_handle->pSPIx_base_addr->CR1 |= (pSPI_handle->SPI_config.SPI_CPOL << SPI_CR1_CPOL);
	pSPI_handle->pSPIx_base_addr->CR1 |= (pSPI_handle->SPI_config.SPI_CPHA << SPI_CR1_CPHA);
	pSPI_handle->pSPIx_base_addr->CR1 |= (pSPI_handle->SPI_config.SPI_SSM << SPI_CR1_SSM);
}

void SPI_deinit(SPI_RegDef_t *pSPIx_base_addr)
{
	if (pSPIx_base_addr == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx_base_addr == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx_base_addr == SPI3)
	{
		SPI3_REG_RESET();
	}
}

// clock control
void SPI_clk_ctrl(SPI_RegDef_t *pSPIx_base_addr, uint8_t en_or_dis)
{
	if(en_or_dis == ENABLE)
	{
		if(pSPIx_base_addr == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx_base_addr == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx_base_addr == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx_base_addr == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx_base_addr == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx_base_addr == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

// Data send and receive
// blocking type send i.e. polling type
void SPI_send_data(SPI_RegDef_t *pSPIx_base_addr, uint8_t *pTxBuffer, uint32_t length)
{
	while(length > 0)
	{
		// wait until tx buffer is available
		while(get_spi_status(pSPIx_base_addr, SPI_TXE_FLAG) == RESET);

		// check DFF
		if (pSPIx_base_addr->CR1 & (1<<SPI_CR1_DFF))
		{
			// 16 bit DFF
			pSPIx_base_addr->DR = *((uint16_t*)pTxBuffer);
			length--;
			length--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit
			pSPIx_base_addr->DR = *pTxBuffer;
			length--;
			pTxBuffer++;
		}
	}
}
void SPI_receive_data(SPI_RegDef_t *pSPIx_base_addr, uint8_t *pRxBuffer, uint32_t length)
{
	while(length > 0)
	{
		// wait until rx buffer is available
		while(get_spi_status(pSPIx_base_addr, SPI_RXNE_FLAG) == RESET);

		// check DFF
		if (pSPIx_base_addr->CR1 & (1<<SPI_CR1_DFF))
		{
			// 16 bit DFF
			*((uint16_t*)pRxBuffer) = (uint16_t)pSPIx_base_addr->DR ;
			length--;
			length--;
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			// 8 bit
			*pRxBuffer = (uint8_t)pSPIx_base_addr->DR;
			length--;
			pRxBuffer++;
		}
	}
}

// interrupt handling
void SPI_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_dis)
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

void SPI_irq_priority_config(uint8_t irq_number, uint8_t irq_priority)
{
	uint8_t reg_number = irq_number / 4;
	uint8_t bit_position = irq_number % 4;
	uint8_t shift_amount = (8* bit_position) + ( 8 - PRIORITY_BITS_IMPLEMENTED);// only upper nibble of each byte is allocated for priority
	*(NVIC_IPR_BASE_ADDR + (4*reg_number)) |= (irq_priority << shift_amount);
}
void SPI_irq_handling(SPI_handle_t *pSPI_handle)
{
	uint8_t status_reg, control_reg;

	// check for tx interrupt
	status_reg  = pSPI_handle->pSPIx_base_addr->SR & ( 1<< SPI_SR_TXE);
	control_reg	= pSPI_handle->pSPIx_base_addr->CR2 & ( 1<< SPI_CR2_TXEIE);
	if (status_reg && control_reg)
	{
		spi_tx_interrupt_handle(pSPI_handle);
	}

	// check for rx interrupt
	status_reg  = pSPI_handle->pSPIx_base_addr->SR & ( 1<< SPI_SR_RXNE);
	control_reg	= pSPI_handle->pSPIx_base_addr->CR2 & ( 1<< SPI_CR2_RXNEIE);
	if (status_reg && control_reg)
	{
		spi_rx_interrupt_handle(pSPI_handle);
	}

	// check for over run error
	status_reg  = pSPI_handle->pSPIx_base_addr->SR & ( 1<< SPI_SR_OVR);
	control_reg	= pSPI_handle->pSPIx_base_addr->CR2 & ( 1<< SPI_CR2_ERREIE);
	if (status_reg && control_reg)
	{
		spi_error_interrupt_handle(pSPI_handle);
	}
}

uint8_t SPI_interrupt_send_data(SPI_handle_t *pSPI_handle, uint8_t *pTxBuffer, uint32_t length)
{
	if (pSPI_handle->tx_state != SPI_IN_TX )
	{
		// save tx buffer addr and len
		pSPI_handle->pTxbuffer = pTxBuffer;
		pSPI_handle->tx_len = length;

		// set spi as busy to avoid take over
		pSPI_handle->tx_state = SPI_IN_TX;

		// enable TXEIE control to get interrupt when TXE flag is set
		pSPI_handle->pSPIx_base_addr->CR2 |= (1 << SPI_CR2_TXEIE);

		// call isr

	}

	return pSPI_handle->tx_state;
}
uint8_t SPI_interrupt_receive_data(SPI_handle_t *pSPI_handle, uint8_t *pRxBuffer, uint32_t length)
{
	if (pSPI_handle->rx_state != SPI_IN_RX )
	{
		// save tx buffer addr and len
		pSPI_handle->pRxbuffer = pRxBuffer;
		pSPI_handle->rx_len = length;

		// set spi as busy to avoid take over
		pSPI_handle->rx_state = SPI_IN_RX;

		// enable TXEIE control to get interrupt when TXE flag is set
		pSPI_handle->pSPIx_base_addr->CR2 |= (1 << SPI_CR2_RXNEIE);

		// call isr

	}

	return pSPI_handle->tx_state;
}

// control APIs
uint8_t get_spi_status(SPI_RegDef_t *pSPIx_base_addr, uint32_t flag_name)
{
	if (pSPIx_base_addr->SR & flag_name)
	{
		return SET;
	}
	return RESET;
}

void SPI_peripheral_control(SPI_RegDef_t *pSPIx_base_addr, uint8_t en_or_dis)
{
	if (en_or_dis == ENABLE)
	{
		pSPIx_base_addr->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx_base_addr->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

// software ss pin manage
void SPI_SSI_config(SPI_RegDef_t *pSPIx_base_addr, uint8_t en_or_dis)
{
	if (en_or_dis == ENABLE)
	{
		pSPIx_base_addr->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx_base_addr->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
// hardware ss pin manage
void  SPI_SSOE_config(SPI_RegDef_t *pSPIx_base_addr, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx_base_addr->CR2 |=  (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx_base_addr->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}


}

void SPI_close_tx(SPI_handle_t *pSPI_handle)
{
	pSPI_handle->pSPIx_base_addr->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPI_handle->pTxbuffer = NULL;
	pSPI_handle->tx_len = 0;
	pSPI_handle->tx_state = SPI_READY;
}

void SPI_close_rx(SPI_handle_t *pSPI_handle)
{
	pSPI_handle->pSPIx_base_addr->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPI_handle->pRxbuffer = NULL;
	pSPI_handle->rx_len = 0;
	pSPI_handle->rx_state = SPI_READY;
}

// helper function
static void spi_tx_interrupt_handle(SPI_handle_t *pSPI_handle)
{
	// check DFF
	if (pSPI_handle->pSPIx_base_addr->CR1 & (1<<SPI_CR1_DFF))
	{
		// 16 bit DFF
		pSPI_handle->pSPIx_base_addr->DR = *((uint16_t*)pSPI_handle->pTxbuffer);
		pSPI_handle->tx_len--;
		pSPI_handle->tx_len--;
		(uint16_t*)pSPI_handle->pTxbuffer++;
	}
	else
	{
		// 8 bit
		pSPI_handle->pSPIx_base_addr->DR = *pSPI_handle->pTxbuffer;
		pSPI_handle->tx_len--;
		pSPI_handle->pTxbuffer++;
	}

	if (!pSPI_handle->tx_len)
	{
		// tx len zero. close spi and inform application
		pSPI_handle->pSPIx_base_addr->CR2 &= ~(1<< SPI_CR2_TXEIE);
		pSPI_handle->pTxbuffer = NULL;
		pSPI_handle->tx_len = 0;
		pSPI_handle->tx_state = SPI_READY;
		spi_application_callback(pSPI_handle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rx_interrupt_handle(SPI_handle_t *pSPI_handle)
{
	// check DFF
	if (pSPI_handle->pSPIx_base_addr->CR1 & (1<<SPI_CR1_DFF))
	{
		// 16 bit DFF
		*((uint16_t*)pSPI_handle->pRxbuffer) = (uint16_t) pSPI_handle->pSPIx_base_addr->DR;
		pSPI_handle->rx_len--;
		pSPI_handle->rx_len--;
		(uint16_t*)pSPI_handle->pRxbuffer++;
	}
	else
	{
		// 8 bit
		*pSPI_handle->pRxbuffer = (uint8_t)pSPI_handle->pSPIx_base_addr->DR;
		pSPI_handle->rx_len--;
		pSPI_handle->pRxbuffer++;
	}

	if (!pSPI_handle->rx_len)
	{
		// rx len zero. close spi and inform application
		pSPI_handle->pSPIx_base_addr->CR2 &= ~(1<< SPI_CR2_RXNEIE);
		pSPI_handle->pRxbuffer = NULL;
		pSPI_handle->rx_len = 0;
		pSPI_handle->rx_state = SPI_READY;
		spi_application_callback(pSPI_handle, SPI_EVENT_RX_CMPLT);
	}
}
static void spi_error_interrupt_handle(SPI_handle_t *pSPI_handle)
{
	// clear ovr flag by reading DR and SR. Dont read if app is busy in tx as it might need the data.
	uint8_t temp_read;
	if(pSPI_handle->tx_state != SPI_IN_TX)
	{
		temp_read = pSPI_handle->pSPIx_base_addr->DR;
		temp_read = pSPI_handle->pSPIx_base_addr->SR;
	}
	(void)temp_read;
	// inform app
	spi_application_callback(pSPI_handle, SPI_EVENT_OVR_ERR);
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_handle_t *pSPI_handle,uint8_t AppEv)
{

	//weak implementation . application must override this function.
}
