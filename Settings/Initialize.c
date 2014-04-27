/*
 * initialize.c
 *
 * This code is called from startup.s or startup.c
 * Sets clock selection and PLL for STM32F100C8
 *  Created on: Mar 29, 2014
 *      Author: Liam
 */


#include "Headers/nRF51822.h"
void SystemInit(void);

void SystemInit (void)
{
	volatile uint32_t	regState;

	  //Set the clock and other stuff

}
