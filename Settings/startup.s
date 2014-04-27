 /******************************************************************************
 *	Startup for nRF51822
 *  Targets Eclipse Kepler.
 * Did not bother to set up NVIC
 *
 *            This module performs:
 *                - Set the initial SP
 *                - Set the initial PC == Reset_Handler,
 *                - Set the vector table entries with the exceptions ISR address
 *				  - Does zero initialization of variables in memory
 *                - Branches to whatever low level init code that you have (e.g. clock setting)
 *                - Branches to main in the C library.
 *
 *******************************************************************************/
    
  .syntax unified
  .cpu cortex-m0
  .arch armv6-m
  .fpu softvfp
  .thumb

.global  g_pfnVectors
.global  Default_Handler

/***** Memory map addresses defined in the linker script */
.word  _start_data_flash		/* start address for the initialization values of the .data section.*/
.word  _start_data				/* start address for the .data section.*/
.word  _end_data				/* end address for the .data section.*/
.word  _start_bss				/* start address for the .bss section.*/
.word  _end_bss					/* end address for the .bss section.*/


/* Handler for Reset exception. Initializes BSS */
  .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:  

    .equ    NRF_POWER_RAMON_ADDRESS,            0x40000524
    .equ    NRF_POWER_RAMON_RAMxON_ONMODE_Msk,  0xF

    /* Make sure ALL RAM banks are powered on */
    LDR     R0, =NRF_POWER_RAMON_ADDRESS
    LDR     R2, [R0]
    MOVS    R1, #NRF_POWER_RAMON_RAMxON_ONMODE_Msk
    ORRS    R2, R1
    STR     R2, [R0]

/***** Copy the data segment initializers from flash to SRAM. */
  movs  r1, #0
  b     LoopCopyDataInit

CopyDataInit:
  ldr   r3, =_start_data_flash
  ldr   r3, [r3, r1]
  str   r3, [r0, r1]
  adds  r1, r1, #4
    
LoopCopyDataInit:
  ldr   r0, =_start_data
  ldr   r3, =_end_data
  adds  r2, r0, r1
  cmp   r2, r3
  bcc   CopyDataInit
  ldr   r2, =_start_bss
  b     LoopFillZerobss
/* Zero fill the bss segment. */  
FillZerobss:
  movs  r3, #0
  str   r3, [r2]
  adds r2, r2, #4
    
LoopFillZerobss:
  ldr   r3, = _end_bss
  cmp   r2, r3
  bcc   FillZerobss

  bl  SystemInit   	/* Call the clock system intitialization function.*/

  bl    main 		/* Call application entry point */
  bx    lr

.size   Reset_Handler, .-Reset_Handler



/***** Default handler for exceptions where specific handler has not been written */

  .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop /* Loops here forever to trap the exception */
  .size  Default_Handler, .-Default_Handler



/******************************************************************************
* Complete vector table for nRF51822.
* Must be placed at physical address 0x0000.0000.
* 
******************************************************************************/    
  .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
  .size  g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
.word _end_stack
.word Reset_Handler
.word NMI_Handler
.word HardFault_Handler
.word 0
.word 0
.word 0
.word 0
.word 0
.word 0
.word 0
.word SVC_Handler
.word 0
.word 0
.word PendSV_Handler
.word SysTick_Handler

  /* External Nordic nRF51822 specific Interrupts */
.word POWER_CLOCK_IRQHandler		/*POWER_CLOCK */
.word RADIO_IRQHandler		 		/*RADIO */
.word UART0_IRQHandler		 		/*UART0 */
.word SPI0_TWI0_IRQHandler		 	/*SPI0_TWI0 */
.word SPI1_TWI1_IRQHandler		 	/*SPI1_TWI1 */
.word 0		 						/*Reserved */
.word GPIOTE_IRQHandler		 		/*GPIOTE */
.word ADC_IRQHandler		 		/*ADC */
.word TIMER0_IRQHandler		 		/*TIMER0 */
.word TIMER1_IRQHandler		 		/*TIMER1 */
.word TIMER2_IRQHandler		 		/*TIMER2 */
.word RTC0_IRQHandler		 		/*RTC0 */
.word TEMP_IRQHandler		 		/*TEMP */
.word RNG_IRQHandler		 		/*RNG */
.word ECB_IRQHandler		 		/*ECB */
.word CCM_AAR_IRQHandler		 	/*CCM_AAR */
.word WDT_IRQHandler		 		/*WDT */
.word RTC1_IRQHandler		 		/*RTC1 */
.word QDEC_IRQHandler		 		/*QDEC */
.word LPCOMP_COMP_IRQHandler		/*LPCOMP_COMP */
.word SWI0_IRQHandler		 		/*SWI0 */
.word SWI1_IRQHandler		 		/*SWI1 */
.word SWI2_IRQHandler		 		/*SWI2 */
.word SWI3_IRQHandler		 		/*SWI3 */
.word SWI4_IRQHandler		 		/*SWI4 */
.word SWI5_IRQHandler		 		/*SWI5 */
.word 0		 						/*Reserved */
.word 0		 						/*Reserved */
.word 0		 						/*Reserved */
.word 0		 						/*Reserved */
.word 0		 						/*Reserved */
.word 0		 						/*Reserved */



/*******************************************************************************
* Provide weak aliases for each Exception handler to the Default_Handler. 
* As they are weak aliases, any function with the same name will override 
* this definition.
*
* This is the full set of handlers for the full set of STM32F100C8 exception vectors (LG)
*******************************************************************************/
    
  .weak  NMI_Handler
  .thumb_set NMI_Handler,Default_Handler
  
  .weak  HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler
  
  .weak  SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak  PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak  SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler




  .weak  POWER_CLOCK_IRQHandler
  .thumb_set POWER_CLOCK_IRQHandler,Default_Handler

  .weak  RADIO_IRQHandler
  .thumb_set RADIO_IRQHandler,Default_Handler

  .weak  UART0_IRQHandler
  .thumb_set UART0_IRQHandler,Default_Handler

  .weak  SPI0_TWI0_IRQHandler
  .thumb_set SPI0_TWI0_IRQHandler,Default_Handler

  .weak  SPI1_TWI1_IRQHandler
  .thumb_set SPI1_TWI1_IRQHandler,Default_Handler

  .weak  GPIOTE_IRQHandler
  .thumb_set GPIOTE_IRQHandler,Default_Handler

  .weak  ADC_IRQHandler
  .thumb_set ADC_IRQHandler,Default_Handler

  .weak  TIMER0_IRQHandler
  .thumb_set TIMER0_IRQHandler,Default_Handler

  .weak  TIMER1_IRQHandler
  .thumb_set TIMER1_IRQHandler,Default_Handler

  .weak  TIMER2_IRQHandler
  .thumb_set TIMER2_IRQHandler,Default_Handler

  .weak  RTC0_IRQHandler
  .thumb_set RTC0_IRQHandler,Default_Handler

  .weak  TEMP_IRQHandler
  .thumb_set TEMP_IRQHandler,Default_Handler

  .weak  RNG_IRQHandler
  .thumb_set RNG_IRQHandler,Default_Handler

  .weak  ECB_IRQHandler
  .thumb_set ECB_IRQHandler,Default_Handler

  .weak  CCM_AAR_IRQHandler
  .thumb_set CCM_AAR_IRQHandler,Default_Handler

  .weak  WDT_IRQHandler
  .thumb_set WDT_IRQHandler,Default_Handler

  .weak  RTC1_IRQHandler
  .thumb_set RTC1_IRQHandler,Default_Handler

  .weak  QDEC_IRQHandler
  .thumb_set QDEC_IRQHandler,Default_Handler

  .weak  LPCOMP_COMP_IRQHandler
  .thumb_set LPCOMP_COMP_IRQHandler,Default_Handler

  .weak  SWI0_IRQHandler
  .thumb_set SWI0_IRQHandler,Default_Handler

  .weak  SWI1_IRQHandler
  .thumb_set SWI1_IRQHandler,Default_Handler

  .weak  SWI2_IRQHandler
  .thumb_set SWI2_IRQHandler,Default_Handler

  .weak  SWI3_IRQHandler
  .thumb_set SWI3_IRQHandler,Default_Handler

  .weak  SWI4_IRQHandler
  .thumb_set SWI4_IRQHandler,Default_Handler

  .weak  SWI5_IRQHandler
  .thumb_set SWI5_IRQHandler,Default_Handler

  
/***********************END OF FILE****/
