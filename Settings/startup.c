/*
 * startup.c
 *
 * This code is adapted from Johan Simonsson's Mini Example project at http://fun-tech.se/stm32/OlimexBlinky/mini.php
 * Demonstrates startup using C instead of the usual assembly
 * Adjusted to work with my Linker file and Makefile targeting the STM32F100C8
 * Did not bother to set up NVIC
 *  Created on: Mar 29, 2014
 *      Author: Liam
 */

extern int main(void);
extern void SystemInit(void);
extern unsigned int _end_stack;

void Reset_Handler(void);
void nmi_handler(void);
void hardfault_handler(void);


// Define the vector table
unsigned int * myvectors[4]
__attribute__ ((section(".isr_vector")))= {
    (unsigned int *)	&_end_stack,         // stack pointer
    (unsigned int *) 	Reset_Handler,     // code entry point
    (unsigned int *)	nmi_handler,       // NMI handler (not really)
    (unsigned int *)	hardfault_handler  // hard fault handler (let's hope not)
};

void Reset_Handler(void)
{
	SystemInit(); // Set up the clocks and low level initialization like BSS
    main(); // Application entry point
}


void nmi_handler(void)
{
    return ;
}

void hardfault_handler(void)
{
    return ;
}
