/*
 * stm32f407_uart_driver.c
 *
 *  Created on: Oct 9, 2022
 *      Author: kunal.chauhan
 */
#include "stm32f407_uart_driver.h"
#include "stm32f407_rcc_driver.h"


// Initialize and reset
void USART_init(USART_handle_t *pUSART_handle)
{

	uint32_t tempreg=0;

	USART_clk_ctrl(pUSART_handle->pUSARTx_base_addr,ENABLE);

	// CR1

	//Enable USART Tx and Rx engine
	if ( pUSART_handle->USART_config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSART_handle->USART_config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSART_handle->USART_config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

	//configure the Word length
	tempreg |= pUSART_handle->USART_config.USART_WordLength << USART_CR1_M ;

	// configure parity control
	if ( pUSART_handle->USART_config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		// even parity is by default set so no need to confifure

	}else if (pUSART_handle->USART_config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//enable ODD parity
		tempreg |= ( 1 << USART_CR1_PS);

	}

	//CR1 register set
	pUSART_handle->pUSARTx_base_addr->CR1 = tempreg;

	//CR2

	tempreg=0;

	// stop bits
	tempreg |= pUSART_handle->USART_config.USART_NumberOfStopBits << USART_CR2_STOP;

	//CR2 register set
	pUSART_handle->pUSARTx_base_addr->CR2 = tempreg;

	//CR3

	tempreg=0;

	//hardware flow control
	if ( pUSART_handle->USART_config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);

	}
	else if (pUSART_handle->USART_config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= ( 1 << USART_CR3_RTSE);

	}
	else if (pUSART_handle->USART_config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE);
	}


	pUSART_handle->pUSARTx_base_addr->CR3 = tempreg;

	//Baudrate register
	USART_set_baud_rate(pUSART_handle->pUSARTx_base_addr,pUSART_handle->USART_config.USART_Baud);
}

void USART_deinit(USART_RegDef_t *pUSART_base_addr)
{
	if (pUSART_base_addr == USART1)
	{
		USART1_REG_RESET();
	}
	else if (pUSART_base_addr == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSART_base_addr == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSART_base_addr == UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSART_base_addr == UART5)
	{
		UART5_REG_RESET();
	}
	else if(pUSART_base_addr == USART6)
	{
		USART6_REG_RESET();
	}
}

// clock control
void USART_clk_ctrl(USART_RegDef_t *pUSART_base_addr, uint8_t en_or_dis)
{
	if (en_or_dis == ENABLE)
	{
		if (pUSART_base_addr == USART1)
		{
			USART1_PCL_EN();
		}
		else if (pUSART_base_addr == USART2)
		{
			USART2_PCL_EN();
		}
		else if(pUSART_base_addr == USART3)
		{
			USART3_PCL_EN();
		}
		else if(pUSART_base_addr == UART4)
		{
			UART4_PCL_EN();
		}
		else if(pUSART_base_addr == UART5)
		{
			UART5_PCL_EN();
		}
		else if(pUSART_base_addr == USART6)
		{
			USART6_PCL_EN();
		}
	}
	else
	{
		if (pUSART_base_addr == USART1)
		{
			USART1_PCL_DI();
		}
		else if (pUSART_base_addr == USART2)
		{
			USART2_PCL_DI();
		}
		else if(pUSART_base_addr == USART3)
		{
			USART3_PCL_DI();
		}
		else if(pUSART_base_addr == UART4)
		{
			UART4_PCL_DI();
		}
		else if(pUSART_base_addr == UART5)
		{
			UART5_PCL_DI();
		}
		else if(pUSART_base_addr == USART6)
		{
			USART6_PCL_DI();
		}
	}

}

// baud rate setting
void USART_set_baud_rate(USART_RegDef_t *pUSART_base_addr, uint32_t baud_rate)
{
	uint32_t PCLKx,usartdiv, M_part,F_part; // apb clk, USARTDIV, mantissa, fraction

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSART_base_addr == USART1 || pUSART_base_addr == USART6)
	{
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = get_rcc_pclk2_val();
	}else
	{
	   PCLKx = get_rcc_pclk1_val();
	}

	//Check for OVER8 configuration bit
	if(pUSART_base_addr->CR1 & (1 << USART_CR1_OVER8))
	{
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *baud_rate));
	}else
	{
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *baud_rate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSART_base_addr->CR1 & ( 1 << USART_CR1_OVER8))
	{
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

	}else
	{
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSART_base_addr->BRR = tempreg;
}
// Data send and receive
void USART_tx(USART_handle_t *pUSART_handle, uint8_t *pTxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start)
{
	uint16_t *pdata;

	for(uint32_t i = 0 ; i < length; i++)
	{
		//wait until TXE flag is set
		while(! get_USART_status(pUSART_handle->pUSARTx_base_addr,USART_FLAG_TXE));

		//Check for 9 bit or 8 bit in a frame
		if(pUSART_handle->USART_config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//9 BIT so load the DR with 2bytes and mask bits other than first 9 bits
			pdata = (uint16_t*) pTxbuffer;
			pUSART_handle->pUSARTx_base_addr->DR = (*pdata & (uint16_t)0x01FF);

			//check for parity control
			if(pUSART_handle->USART_config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer , so 9bits of user data will be sent
				pTxbuffer++;
				pTxbuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxbuffer++;
			}
		}
		else
		{
			//8bit data transfer
			pUSART_handle->pUSARTx_base_addr->DR = *pTxbuffer;
			pTxbuffer++;
		}
	}

	//wait until TC flag is set in the SR
	while( ! get_USART_status(pUSART_handle->pUSARTx_base_addr,USART_FLAG_TC));
}
void USART_rx(USART_handle_t *pUSART_handle, uint8_t *pRxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start)
{

	for(uint32_t i = 0 ; i < length; i++)
	{
		//wait until RXNE flag is set
		while(! get_USART_status(pUSART_handle->pUSARTx_base_addr,USART_FLAG_RXNE));

		//Check WordLength to decide receive 9bit of data or 8 bit
		if(pUSART_handle->USART_config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//receive 9bit data
			//check Parity Control
			if(pUSART_handle->USART_config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is usedall 9bits user data
				//mask the DR with 0x01FF
				*((uint16_t*) pRxbuffer) = (pUSART_handle->pUSARTx_base_addr->DR  & (uint16_t)0x01FF);

				pRxbuffer++;
				pRxbuffer++;
			}
			else
			{
				//Parity is used 8bits user data and 1 bit is parity
				*pRxbuffer = (pUSART_handle->pUSARTx_base_addr->DR  & (uint8_t)0xFF);
				pRxbuffer++;
			}
		}
		else
		{
			//receive 8bit data
			//check Parity Control
			if(pUSART_handle->USART_config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used all 8bits user data
				 *pRxbuffer = (uint8_t) (pUSART_handle->pUSARTx_base_addr->DR & 0xFF);
			}
			else
			{
				//Parity is used 7 bits user data and 1 bit parity
				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxbuffer = (uint8_t) (pUSART_handle->pUSARTx_base_addr->DR  & (uint8_t)0x7F);
			}

			pRxbuffer++;
		}
	}
}
uint8_t USART_tx_interrupt(USART_handle_t *pUSART_handle, uint8_t *pTxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start)
{
	uint8_t txstate = pUSART_handle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSART_handle->TxLen = length;
		pUSART_handle->pTxBuffer = pTxbuffer;
		pUSART_handle->TxBusyState = USART_BUSY_IN_TX;

		//enable interrupt for TXE
		pUSART_handle->pUSARTx_base_addr->CR1 |= ( 1 << USART_CR1_TXEIE);

		//enable interrupt for TC
		pUSART_handle->pUSARTx_base_addr->CR1 |= ( 1 << USART_CR1_TCIE);
	}

	return txstate;
}
uint8_t USART_rx_interrupt(USART_handle_t *pUSART_handle, uint8_t *pRxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start)
{
	uint8_t rxstate = pUSART_handle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSART_handle->RxLen = length;
		pUSART_handle->pRxBuffer = pRxbuffer;
		pUSART_handle->RxBusyState = USART_BUSY_IN_RX;

		// clear RXNE
		//(void)pUSART_handle->pUSARTx_base_addr->DR;

		//enable interrupt for RXNE
		pUSART_handle->pUSARTx_base_addr->CR1 |= ( 1 << USART_CR1_RXNEIE);

	}

	return rxstate;
}


// interrupt handling
void USART_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_dis)
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
void USART_irq_priority_config(uint8_t irq_number, uint8_t irq_priority)
{
	uint8_t reg_number = irq_number / 4;
	uint8_t bit_position = irq_number % 4;
	uint8_t shift_amount = (8* bit_position) + ( 8 - PRIORITY_BITS_IMPLEMENTED);// only upper nibble of each byte is allocated for priority
	*(NVIC_IPR_BASE_ADDR + (4*reg_number)) |= (irq_priority << shift_amount);
}
void USART_irq_handling(USART_handle_t *pUSART_handle)
{
	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata;

	// TC flag
	temp1 = pUSART_handle->pUSARTx_base_addr->SR & ( 1 << USART_SR_TC);
	temp2 = pUSART_handle->pUSARTx_base_addr->CR1 & ( 1 << USART_CR1_TCIE);
	if(temp1 && temp2 )
	{
		//close tx call application callback if TxLen is zero
		if ( pUSART_handle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(! pUSART_handle->TxLen )
			{
				//clear TC flag
				pUSART_handle->pUSARTx_base_addr->SR &= ~( 1 << USART_SR_TC);

				//clear the TCIE control bit
				pUSART_handle->pUSARTx_base_addr->CR1 &=  ~( 1 << USART_CR1_TCIE);

				//Reset the application state
				pUSART_handle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSART_handle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSART_handle->TxLen = 0;

				//application call back
				USART_application_callback(pUSART_handle,USART_EVENT_TX_CMPLT);
			}
		}
	}

	// TXE flag
	temp1 = pUSART_handle->pUSARTx_base_addr->SR & ( 1 << USART_SR_TXE);
	temp2 = pUSART_handle->pUSARTx_base_addr->CR1 & ( 1 << USART_CR1_TXEIE);

	if(temp1 && temp2 )
	{
		if(pUSART_handle->TxBusyState == USART_BUSY_IN_TX)
		{
			if(pUSART_handle->TxLen > 0)
			{
				//9BIT or 8BIT in a frame
				if(pUSART_handle->USART_config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSART_handle->pTxBuffer;
					pUSART_handle->pUSARTx_base_addr->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSART_handle->USART_config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						pUSART_handle->pTxBuffer++;
						pUSART_handle->pTxBuffer++;
						pUSART_handle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSART_handle->pTxBuffer++;
						pUSART_handle->TxLen-=1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSART_handle->pUSARTx_base_addr->DR = *pUSART_handle->pTxBuffer;

					pUSART_handle->pTxBuffer++;
					pUSART_handle->TxLen-=1;
				}

			}
			if (pUSART_handle->TxLen == 0 )
			{
				// disable interrupt for TXE flag
				pUSART_handle->pUSARTx_base_addr->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

	// RXNE flag
	temp1 = pUSART_handle->pUSARTx_base_addr->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSART_handle->pUSARTx_base_addr->CR1 & ( 1 << USART_CR1_RXNEIE);

	if(temp1 && temp2 )
	{
		if(pUSART_handle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSART_handle->RxLen > 0)
			{
				//9bit of data in a frame or 8 bit
				if(pUSART_handle->USART_config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//Parity Control
					if(pUSART_handle->USART_config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity all 9bits user data
						*((uint16_t*) pUSART_handle->pRxBuffer) = (pUSART_handle->pUSARTx_base_addr->DR  & (uint16_t)0x01FF);

						pUSART_handle->pRxBuffer++;
						pUSART_handle->pRxBuffer++;
						pUSART_handle->RxLen-=2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSART_handle->pRxBuffer = (pUSART_handle->pUSARTx_base_addr->DR  & (uint8_t)0xFF);
						 pUSART_handle->pRxBuffer++;
						 pUSART_handle->RxLen-=1;
					}
				}
				else
				{
					//receive 8bit data
					//Parity Control
					if(pUSART_handle->USART_config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity 8bits  user data
						 *pUSART_handle->pRxBuffer = (uint8_t) (pUSART_handle->pUSARTx_base_addr->DR  & (uint8_t)0xFF);
					}
					else
					{
						//Parity is used 7 bits user data and 1 bit is parity
						 *pUSART_handle->pRxBuffer = (uint8_t) (pUSART_handle->pUSARTx_base_addr->DR  & (uint8_t)0x7F);
					}

					pUSART_handle->pRxBuffer++;
					pUSART_handle->RxLen-=1;
				}


			}
			if(! pUSART_handle->RxLen)
			{
				//disable the rxne
				pUSART_handle->pUSARTx_base_addr->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSART_handle->RxBusyState = USART_READY;
				USART_application_callback(pUSART_handle,USART_EVENT_RX_CMPLT);
			}
		}
	}

	//CTS flag
	//not applicable for UART4 and UART5

	temp1 = pUSART_handle->pUSARTx_base_addr->SR & ( 1 << USART_SR_CTS);
	temp2 = pUSART_handle->pUSARTx_base_addr->CR3 & ( 1 << USART_CR3_CTSE);
	temp3 = pUSART_handle->pUSARTx_base_addr->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 && temp3)
	{
		//clear the CTS flag in SR
		pUSART_handle->pUSARTx_base_addr->SR &=  ~( 1 << USART_SR_CTS);

		//notify app of CTS since im not using CTS for this project
		USART_application_callback(pUSART_handle,USART_EVENT_CTS);
	}

	// IDLE flag
	temp1 = pUSART_handle->pUSARTx_base_addr->SR & ( 1 << USART_SR_IDLE);
	temp2 = pUSART_handle->pUSARTx_base_addr->CR1 & ( 1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		//clear the IDLE flag read SR and DR
		temp1 = pUSART_handle->pUSARTx_base_addr->SR;
		temp1 = pUSART_handle->pUSARTx_base_addr->DR;

		USART_application_callback(pUSART_handle,USART_EVENT_IDLE);
	}

	// OVERRUN flag
	temp1 = pUSART_handle->pUSARTx_base_addr->SR & USART_SR_ORE;
	temp2 = pUSART_handle->pUSARTx_base_addr->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		// Let app clear overrun flag based on requirement
		USART_application_callback(pUSART_handle,USART_ERR_ORE);
	}

	// all other error flags Noise Flag, Overrun error and Framing Error are used in multibuffer comm.
	// not currently working on it
}

// control APIs
uint8_t get_USART_status(USART_RegDef_t *pUSART_base_addr, uint32_t flag_name)
{
	if (pUSART_base_addr->SR & flag_name)
	{
		return SET;
	}
	return RESET;
}
void USART_peripheral_control(USART_RegDef_t *pUSART_base_addr, uint8_t en_or_dis)
{
	if (en_or_dis == ENABLE)
	{
		pUSART_base_addr->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSART_base_addr->CR1 &= ~(1 << USART_CR1_UE);
	}
}


// application callback
void USART_application_callback(USART_handle_t *pUSART_handle, uint8_t event )
{

}
