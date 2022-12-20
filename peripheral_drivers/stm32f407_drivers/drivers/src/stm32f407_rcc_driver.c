/*
 * stm32f407_rcc_driver.c
 *
 *  Created on: Oct 15, 2022
 *      Author: kunal.chauhan
 */

#include "stm32f407_rcc_driver.h"

uint16_t AHB_pre_scaler[] = {2,4,8,16,64,128,256,512};
uint8_t APB_pre_scaler[] = {2,4,8,16};

uint32_t get_rcc_pclk1_val(void)
{
	uint32_t pclk1, system_clk;
	uint8_t clk_src, temp, ahb_ps, apb1_ps; // ps for pre scaler

	//bit 2 and 3 define the sytsem clk used as HSI, HSe or PLL
	clk_src = (RCC->CFGR >> 2) & 0x3;

	// get system clk
	if (clk_src==0)
	{
		system_clk = 16000000;
	}
	else if (clk_src==1)
	{
		system_clk = 8000000;
	}
	else if (clk_src==2)
	{
		//PLL TODO
	}

	// get AHB pre scaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8)
	{
		ahb_ps = 1;
	}
	else
	{
		ahb_ps = AHB_pre_scaler[temp-8];
	}

	// get APB pre scaler
	temp = ((RCC->CFGR >> 10) & 0x7);
	if (temp < 4)
	{
		apb1_ps = 1;
	}
	else
	{
		apb1_ps = APB_pre_scaler[temp-4];
	}

	pclk1 = (system_clk / ahb_ps)/apb1_ps;

	return pclk1;
}
uint32_t get_rcc_pclk2_val(void)
{
	uint32_t pclk2, system_clk, temp;
	uint8_t clk_src, ahb_ps, apb2_ps; // ps for pre scaler

	clk_src = ( RCC->CFGR >> 2) & 0X3;

	if(clk_src == 0)
	{
		system_clk = 16000000;
	}else
	{
		system_clk = 8000000;
	}
	temp = (RCC->CFGR >> 4 ) & 0xF;

	if(temp < 0x08)
	{
		ahb_ps = 1;
	}else
	{
		ahb_ps = AHB_pre_scaler[temp-8];
	}

	temp = (RCC->CFGR >> 13 ) & 0x7;
	if(temp < 0x04)
	{
		apb2_ps = 1;
	}else
	{
		apb2_ps = APB_pre_scaler[temp-4];
	}

	pclk2 = (system_clk / ahb_ps )/ apb2_ps;

	return pclk2;
}
