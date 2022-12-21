/*
 * stm32f407_i2c_driver.c
 *
 *  Created on: Aug 31, 2022
 *      Author: kunal.chauhan
 */
#include "stm32f407_i2c_driver.h"
#include "stm32f407_rcc_driver.h"

static void I2C_generate_start(I2C_RegDef_t *pI2C_base_addr);
static void I2C_address_phase(I2C_RegDef_t *pI2C_base_addr, uint8_t slave_addr, uint8_t read_or_write);
static void I2C_clear_addr_flag(I2C_handle_t *pI2C_handle);

// Initialize and reset
void I2C_init(I2C_handle_t *pI2C_handle)
{
	uint32_t temp = 0;

	// ack/nack
	// TODO actually this code is incorrect. ACK bit is set only when PE=1( peripheral is enabled)
	// when PE = 0, ack bit is cleared by hardware automatically so ack=1 here doesnt matter.
	// so enable ACK after enable PE in application side.
	temp |= (pI2C_handle->I2C_config.I2C_AckControl << 10);
	pI2C_handle->pI2C_base_addr->CR1 = temp;

	// FREQ
	temp = 0;
	temp |= get_rcc_pclk1_val()/1000000U;
	pI2C_handle->pI2C_base_addr->CR2 = (temp & 0x3F);

	// slave addr. by default 7 bit addressing mode is used
	temp = 0;
	temp |= pI2C_handle->I2C_config.I2C_DeviceAddress << 1;
	temp |= (1<<14); // RM recommmend to keep 14 bit as 1 by SW
	pI2C_handle->pI2C_base_addr->OAR1 = temp;

	// CCR config
	uint16_t ccr = 0;
	temp = 0;
	if(pI2C_handle->I2C_config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// standarad mode
		// T(high) = CCR * T(pclk1), T(low) = CCR * T(pclk1)
		// for std mode thigh = tlow. so ccr = scl speed/ (2*T(pclk1))
		ccr = (get_rcc_pclk1_val()/ (2 * pI2C_handle->I2C_config.I2C_SCLSpeed));
		temp |= (ccr & 0xFFF);
	}
	else
	{
		// fast mode
		temp |= (1<<15); // set FM mode
		temp |= (pI2C_handle->I2C_config.I2C_FMDutyCycle << 14); // duty cycle
		if (pI2C_handle->I2C_config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr = (get_rcc_pclk1_val()/ (3 * pI2C_handle->I2C_config.I2C_SCLSpeed));
		}
		else
		{
			ccr = (get_rcc_pclk1_val()/ (25 * pI2C_handle->I2C_config.I2C_SCLSpeed));
		}
		temp |= (ccr & 0xFFF);
	}

	pI2C_handle->pI2C_base_addr->CCR = temp;

	//  Rise time TRISE
	temp=0;
	if (pI2C_handle->I2C_config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// standard mode
		temp = (get_rcc_pclk1_val()/1000000U) + 1;
	}
	else
	{
		// fast mode
		temp = (get_rcc_pclk1_val() * 300 /1000000000U) + 1;
	}
	pI2C_handle->pI2C_base_addr->TRISE = (temp & 0x3F);

	pI2C_handle->TxRxState = I2C_READY;
}

void I2C_deinit(I2C_RegDef_t *pI2C_base_addr)
{
	if (pI2C_base_addr == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2C_base_addr == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2C_base_addr == I2C3)
	{
		I2C3_REG_RESET();
	}
}

// clock control
void I2C_clk_ctrl(I2C_RegDef_t *pI2C_base_addr, uint8_t en_or_dis)
{
	if(en_or_dis == ENABLE)
	{
		if(pI2C_base_addr == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2C_base_addr == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2C_base_addr == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2C_base_addr == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2C_base_addr == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2C_base_addr == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

// Master Data send and receive
void I2C_master_tx(I2C_handle_t *pI2C_handle, uint8_t *pTxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start)
{
	// start condition
	I2C_generate_start(pI2C_handle->pI2C_base_addr);

	// wait until start is generated. Check SB flag in status reg
	// clk will pulled until SB flag is cleared
	while(!get_I2C_status(pI2C_handle->pI2C_base_addr, I2C_FLAG_SB));

	// send slave addr with R/W bit
	I2C_address_phase(pI2C_handle->pI2C_base_addr, slave_addr, I2C_MASTER_WRITE);

	// check if ADDR flag is set
	while(!get_I2C_status(pI2C_handle->pI2C_base_addr, I2C_FLAG_ADDR));

	//clear ADDR flag
	I2C_clear_addr_flag(pI2C_handle);

	// send data till length = 0
	while ( length > 0)
	{
		//wait for TXNE flag to be set
		while(! get_I2C_status(pI2C_handle->pI2C_base_addr, I2C_FLAG_TXE));
		pI2C_handle->pI2C_base_addr->DR = *pTxbuffer;
		pTxbuffer++;
		length--;
	}

	// close I2C
	// after length becomes zero, wait for TXE and BTF flag to be set.
	while(!get_I2C_status(pI2C_handle->pI2C_base_addr, I2C_FLAG_TXE));
	while(!get_I2C_status(pI2C_handle->pI2C_base_addr, I2C_FLAG_BTF));

	// generate stop condition only when required by the application using r_start var
	// otherwise keep using repeated start
	if(r_start == I2C_NO_REPEATED_START)
	{
		I2C_generate_stop(pI2C_handle->pI2C_base_addr);
	}
}
void I2C_master_rx(I2C_handle_t *pI2C_handle, uint8_t *pRxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start)
{
	// generate start condition
	I2C_generate_start(pI2C_handle->pI2C_base_addr);

	// wait until start is generated. Check SB flag in status reg
	// clk will pulled until SB flag is cleared
	while(!get_I2C_status(pI2C_handle->pI2C_base_addr, I2C_FLAG_SB));

	// send slave addr with R/W bit
	I2C_address_phase(pI2C_handle->pI2C_base_addr, slave_addr,I2C_MASTER_READ);

	// check if ADDR flag is set
	while(!get_I2C_status(pI2C_handle->pI2C_base_addr, I2C_FLAG_ADDR));

	// read data
	// read one byte only
	if(length == 1)
	{
		// disable ack
		I2C_manage_ack(pI2C_handle->pI2C_base_addr,I2C_ACK_DISABLE);

		// clear ADDR flag
		I2C_clear_addr_flag(pI2C_handle);


		//wait for RXNE flag to be set
		while(! get_I2C_status(pI2C_handle->pI2C_base_addr, I2C_FLAG_RXNE));

		// generate stop
		if(r_start == I2C_NO_REPEATED_START)
		{
			I2C_generate_stop(pI2C_handle->pI2C_base_addr);
		}

		// read
		*pRxbuffer = pI2C_handle->pI2C_base_addr->DR;
	}

	if(length > 1)
	{
		// clear ADDR flag
		I2C_clear_addr_flag(pI2C_handle);

		// read until length becomes zero
		for ( uint32_t curr_len = length ; curr_len > 0; curr_len--)
		{
			//wait for RXNE flag to be set
			while(! get_I2C_status(pI2C_handle->pI2C_base_addr, I2C_FLAG_RXNE));

			if (curr_len == 2)
			{
				// disable ack
				I2C_manage_ack(pI2C_handle->pI2C_base_addr, I2C_ACK_DISABLE);

				//generate stop
				if(r_start == I2C_NO_REPEATED_START)
				{
					I2C_generate_stop(pI2C_handle->pI2C_base_addr);
				}
			}

			// read
			*pRxbuffer = pI2C_handle->pI2C_base_addr->DR;
			pRxbuffer++;
		}
	}

	// re-enable
	if(pI2C_handle->I2C_config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_manage_ack(pI2C_handle->pI2C_base_addr, I2C_ACK_ENABLE);
	}
}

uint8_t I2C_master_tx_interrupt(I2C_handle_t *pI2C_handle, uint8_t *pTxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start)
{
	uint8_t busystate = pI2C_handle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2C_handle->pTxBuffer = pTxbuffer;
		pI2C_handle->TxLen = length;
		pI2C_handle->TxRxState = I2C_BUSY_IN_TX;
		pI2C_handle->DevAddr = slave_addr;
		pI2C_handle->Sr = r_start;

		// generate start condition
		I2C_generate_start(pI2C_handle->pI2C_base_addr);

		// enable ITBUFEN Control Bit
		pI2C_handle->pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// enable ITEVFEN Control Bit
		pI2C_handle->pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//enable ITERREN Control Bit
		pI2C_handle->pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}
uint8_t I2C_master_rx_interrupt(I2C_handle_t *pI2C_handle, uint8_t *pRxbuffer, uint32_t length, uint8_t slave_addr, uint8_t r_start)
{
	uint8_t busystate = pI2C_handle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2C_handle->pRxBuffer = pRxbuffer;
		pI2C_handle->RxLen = length;
		pI2C_handle->TxRxState = I2C_BUSY_IN_RX;
		pI2C_handle->DevAddr = slave_addr;
		pI2C_handle->Sr = r_start;

		// generate start condition
		I2C_generate_start(pI2C_handle->pI2C_base_addr);

		// enable ITBUFEN Control Bit
		pI2C_handle->pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// enable ITEVFEN Control Bit
		pI2C_handle->pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//enable ITERREN Control Bit
		pI2C_handle->pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

// slave data send and recieeve
void I2C_slave_tx(I2C_RegDef_t *pI2C_base_addr, uint8_t data)
{
	pI2C_base_addr->DR = data;
}
uint8_t I2C_slave_rx(I2C_RegDef_t *pI2C_base_addr)
{
	return (uint8_t)pI2C_base_addr->DR;
}
// interrupt handling
void I2C_irq_interrupt_config(uint8_t irq_number, uint8_t en_or_dis)
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

void I2C_irq_priority_config(uint8_t irq_number, uint8_t irq_priority)
{
	uint8_t reg_number = irq_number / 4;
	uint8_t bit_position = irq_number % 4;
	uint8_t shift_amount = (8* bit_position) + ( 8 - PRIORITY_BITS_IMPLEMENTED);// only upper nibble of each byte is allocated for priority
	*(NVIC_IPR_BASE_ADDR + (4*reg_number)) |= (irq_priority << shift_amount);
}
void I2C_event_irq_handling(I2C_handle_t *pI2C_handle)
{
	//Interrupt handling for both master and slave mode of a device

	uint8_t status_reg, control_reg_temp1, control_reg_temp2;

	//interrupt generated by SB event
	//SB flag is only in Master mode
	status_reg  = pI2C_handle->pI2C_base_addr->SR1 & ( 1<< I2C_SR1_SB);
	control_reg_temp1	= pI2C_handle->pI2C_base_addr->CR2 & ( 1<< I2C_CR2_ITEVTEN);
	control_reg_temp2	= pI2C_handle->pI2C_base_addr->CR2 & ( 1<< I2C_CR2_ITBUFEN);
	if (status_reg && control_reg_temp1)
	{
		// SB flag set
		if (pI2C_handle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_address_phase(pI2C_handle->pI2C_base_addr, pI2C_handle->DevAddr, I2C_MASTER_WRITE);
		}
		else if (pI2C_handle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_address_phase(pI2C_handle->pI2C_base_addr, pI2C_handle->DevAddr, I2C_MASTER_READ);
		}
	}

	//interrupt generated by ADDR event
	//When master mode : Address is sent
	//When Slave mode  : Address matched with own address
	status_reg  = pI2C_handle->pI2C_base_addr->SR1 & ( 1<< I2C_SR1_ADDR);
	if (status_reg && control_reg_temp1)
	{
		// ADDR flag set
		I2C_clear_addr_flag(pI2C_handle);
	}

	//interrupt generated by BTF(Byte Transfer Finished) event
	status_reg  = pI2C_handle->pI2C_base_addr->SR1 & ( 1<< I2C_SR1_BTF);
	if (status_reg && control_reg_temp1)
	{
		// BTF flag set
		if (pI2C_handle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2C_handle->pI2C_base_addr->SR1 & (1 << I2C_SR1_TXE))
			{
				if(pI2C_handle->TxLen == 0)
				{
					// both txe and btf set. which means tx is complete

					// generate stop
					if (pI2C_handle->Sr == I2C_NO_REPEATED_START)
					{
						I2C_generate_stop(pI2C_handle->pI2C_base_addr);
					}
					// close tx
					I2C_close_tx(pI2C_handle);

					// notify application
					I2C_application_callback(pI2C_handle, I2C_EV_TX_CMPLT);
				}
			}

		}
		else if (pI2C_handle->TxRxState == I2C_BUSY_IN_RX)
		{
			// Do nothing
		}
	}

	//interrupt generated by STOPF event
	//Stop detection flag only slave mode. For master this flag will never be set
	status_reg  = pI2C_handle->pI2C_base_addr->SR1 & ( 1<< I2C_SR1_STOPF);
	if (status_reg && control_reg_temp1)
	{
		// STOPF flag set // need to clear flag by read SR1 and write CR1
		// dummy write to avoid CR reg changes.
		pI2C_handle->pI2C_base_addr->CR1 |= 0x0000;

		// notify application
		I2C_application_callback(pI2C_handle, I2C_EV_STOP);

	}

	//interrupt generated by TXE event
	status_reg  = pI2C_handle->pI2C_base_addr->SR1 & ( 1<< I2C_SR1_TXE);
	if (status_reg && control_reg_temp1 && control_reg_temp2)
	{
		// TXE flag set. transmit data only if master mode
		if (pI2C_handle->pI2C_base_addr->SR2 & (1 << I2C_SR2_MSL))
		{
			if (pI2C_handle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2C_handle->TxLen > 0)
				{
					// load into data reg
					pI2C_handle->pI2C_base_addr->DR = *(pI2C_handle->pTxBuffer);

					// decrement tx len
					pI2C_handle->TxLen--;

					// Increment buffer address
					pI2C_handle->pTxBuffer++;
				}
			}
		}
		else
		{
			//slave mode
			// check if transmitter mode
			if (pI2C_handle->pI2C_base_addr->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_application_callback(pI2C_handle, I2C_EV_DATA_REQ);
			}
		}
	}

	//interrupt generated by RXNE event
	status_reg  = pI2C_handle->pI2C_base_addr->SR1 & ( 1<< I2C_SR1_RXNE);
	if (status_reg && control_reg_temp1 && control_reg_temp2)
	{
		if (pI2C_handle->pI2C_base_addr->SR2 & (1 << I2C_SR2_MSL))
		{
			// RXNE flag set
			if (pI2C_handle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2C_handle->RxSize == 1)
				{
					*pI2C_handle->pRxBuffer = pI2C_handle->pI2C_base_addr->DR;
					pI2C_handle->RxLen--;
				}
				if(pI2C_handle->RxSize > 1)
				{
					if (pI2C_handle->RxLen == 2)
					{
						// disable ack
						I2C_manage_ack(pI2C_handle->pI2C_base_addr, I2C_ACK_DISABLE);
					}

					// read
					*pI2C_handle->pRxBuffer = pI2C_handle->pI2C_base_addr->DR;
					pI2C_handle->pRxBuffer++;
					pI2C_handle->RxLen--;
				}
				if(pI2C_handle->RxLen == 0)
				{
					// close reception and motify application

					// generate stop
					if(pI2C_handle->Sr == I2C_NO_REPEATED_START)
					{
						I2C_generate_stop(pI2C_handle->pI2C_base_addr);
					}
					// close rx
					I2C_close_rx(pI2C_handle);

					// notify
					I2C_application_callback(pI2C_handle, I2C_EV_RX_CMPLT);
				}

			}
		}
		else
		{
			//slave
			// check if slave in rx mode
			if (pI2C_handle->pI2C_base_addr->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_application_callback(pI2C_handle, I2C_EV_DATA_RCV);
			}
		}
	}

}
void I2C_error_irq_handling(I2C_handle_t *pI2C_handle)
{
	uint32_t status_reg,control_reg;

    //status of ITERREN bit in the CR2
	control_reg = (pI2C_handle->pI2C_base_addr->CR2) & ( 1 << I2C_CR2_ITERREN);

	//Bus error
	status_reg = (pI2C_handle->pI2C_base_addr->SR1) & ( 1<< I2C_SR1_BERR);
	if(status_reg  && control_reg )
	{

		//clear the buss error flag
		pI2C_handle->pI2C_base_addr->SR1 &= ~( 1 << I2C_SR1_BERR);

		//notify the application
	   I2C_application_callback(pI2C_handle,I2C_ERROR_BERR);
	}

	//arbitration lost error
	status_reg = (pI2C_handle->pI2C_base_addr->SR1) & ( 1 << I2C_SR1_ARLO );
	if(status_reg  && control_reg)
	{
		//clear the arbitration lost error flag
		pI2C_handle->pI2C_base_addr->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//notify the application about the error
		I2C_application_callback(pI2C_handle,I2C_ERROR_ARLO);
	}

	//ACK failure  error
	status_reg = (pI2C_handle->pI2C_base_addr->SR1) & ( 1 << I2C_SR1_AF);
	if(status_reg  && control_reg)
	{
	    //clear the ACK failure error flag
		pI2C_handle->pI2C_base_addr->SR1 &= ~( 1 << I2C_SR1_AF);

		//notify the application about the error
		I2C_application_callback(pI2C_handle,I2C_ERROR_AF);
	}

	//Overrun/underrun error
	status_reg = (pI2C_handle->pI2C_base_addr->SR1) & ( 1 << I2C_SR1_OVR);
	if(status_reg  && control_reg)
	{
	    //clear the Overrun/underrun error flag
		pI2C_handle->pI2C_base_addr->SR1 &= ~( 1 << I2C_SR1_OVR);

		//notify the application about the error
		I2C_application_callback(pI2C_handle,I2C_ERROR_OVR);
	}

	//Time out error
	status_reg = (pI2C_handle->pI2C_base_addr->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(status_reg  && control_reg)
	{
	    //clear the Time out error flag
		pI2C_handle->pI2C_base_addr->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//notify the application about the error
		I2C_application_callback(pI2C_handle,I2C_ERROR_TIMEOUT);
	}
}

// control APIs
uint8_t get_I2C_status(I2C_RegDef_t *pI2C_base_addr, uint32_t flag_name)
{
	if (pI2C_base_addr->SR1 & flag_name)
	{
		return SET;
	}
	return RESET;
}

void I2C_peripheral_control(I2C_RegDef_t *pI2C_base_addr, uint8_t en_or_dis)
{
	if (en_or_dis == ENABLE)
	{
		pI2C_base_addr->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2C_base_addr->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

void I2C_manage_ack(I2C_RegDef_t *pI2C_base_addr, uint8_t en_or_dis)
{
	if(en_or_dis == I2C_ACK_ENABLE)
	{
		pI2C_base_addr->CR1 |= (1<<I2C_CR1_ACK);
	}
	else
	{
		pI2C_base_addr->CR1 &= ~(1<<I2C_CR1_ACK);
	}
}

void I2C_close_tx(I2C_handle_t *pI2C_handle)
{
	pI2C_handle->pI2C_base_addr->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2C_handle->pI2C_base_addr->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2C_handle->TxRxState = I2C_READY;
	pI2C_handle->pTxBuffer = NULL;
	pI2C_handle->TxLen = 0;

}
void I2C_close_rx(I2C_handle_t *pI2C_handle)
{
	//disable interrupts
	pI2C_handle->pI2C_base_addr->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2C_handle->pI2C_base_addr->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//reset struct elemetns
	pI2C_handle->TxRxState = I2C_READY;
	pI2C_handle->pRxBuffer = NULL;
	pI2C_handle->RxLen = 0;
	pI2C_handle->RxSize = 0;
	if(pI2C_handle->I2C_config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_manage_ack(pI2C_handle->pI2C_base_addr, ENABLE);
	}


}
// helher functions
static void I2C_generate_start(I2C_RegDef_t *pI2C_base_addr)
{
	pI2C_base_addr->CR1 |= (1<<I2C_CR1_START);
}

static void I2C_address_phase(I2C_RegDef_t *pI2C_base_addr, uint8_t slave_addr, uint8_t read_or_write)
{
	slave_addr = slave_addr << 1;
	if ( read_or_write == I2C_MASTER_WRITE) // write mode
	{
		slave_addr &= ~(1); // R/W bit is 0
	}
	else // read mode
	{
		slave_addr |= 1; // R/W bit is 1
	}
	pI2C_base_addr->DR = slave_addr;
}

static void I2C_clear_addr_flag(I2C_handle_t *pI2C_handle)
{
	uint32_t dummy;
	// check if device is in master mode and rx state
	if (pI2C_handle->pI2C_base_addr->SR2 & (1 << I2C_SR2_MSL))
	{
		if(pI2C_handle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2C_handle->RxSize == 1)
			{
				// disable ack
				I2C_manage_ack(pI2C_handle->pI2C_base_addr, DISABLE);

				// clear addr flag
				dummy = pI2C_handle->pI2C_base_addr->SR1;
				dummy = pI2C_handle->pI2C_base_addr->SR2;
				(void)dummy;
			}
		}
		else
		{
			// clear addr flag
			dummy = pI2C_handle->pI2C_base_addr->SR1;
			dummy = pI2C_handle->pI2C_base_addr->SR2;
			(void)dummy;
		}
	}
	else
	{
		// clear addr flag
		dummy = pI2C_handle->pI2C_base_addr->SR1;
		dummy = pI2C_handle->pI2C_base_addr->SR2;
		(void)dummy;
	}
}

void I2C_generate_stop(I2C_RegDef_t *pI2C_base_addr)
{
	pI2C_base_addr->CR1 |= (1<<I2C_CR1_STOP);
}

void I2C_slave_en_di_callback_ev(I2C_RegDef_t *pI2C_base_addr, uint8_t en_or_dis)
{
	if (en_or_dis == ENABLE)
	{
		// enable ITBUFEN Control Bit
		pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// enable ITEVFEN Control Bit
		pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//enable ITERREN Control Bit
		pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else
	{
		// siable ITBUFEN Control Bit
		pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// disable ITEVFEN Control Bit
		pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//disable ITERREN Control Bit
		pI2C_base_addr->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}
}
