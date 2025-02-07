.syntax unified

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
  .section .text.Default_Handler
  .type Default_Handler, %function
Default_Handler:
  b Default_Handler
/******************************************************************************
*
* The STM32L031F4Px vector table.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word	0
  .word	0
  .word	0
  .word	0
  .word	0
  .word	0
  .word	0
  .word	SVC_Handler
  .word	0
  .word	0
  .word	PendSV_Handler
  .word	SysTick_Handler
  .word	WWDG_IRQHandler           			/* Window Watchdog interrupt                                                      */
  .word	PVD_IRQHandler            			/* PVD through EXTI line detection                                                */
  .word	RTC_IRQHandler            			/* RTC global interrupt                                                           */
  .word	FLASH_IRQHandler          			/* Flash global interrupt                                                         */
  .word	RCC_IRQHandler            			/* RCC global interrupt                                                           */
  .word	EXTI0_1_IRQHandler        			/* EXTI Line[1:0] interrupts                                                      */
  .word	EXTI2_3_IRQHandler        			/* EXTI Line[3:2] interrupts                                                      */
  .word	EXTI4_15_IRQHandler       			/* EXTI Line15 and EXTI4 interrupts                                               */
  .word	0                         			/* Reserved                                                                       */
  .word	DMA1_Channel1_IRQHandler  			/* DMA1 Channel1 global interrupt                                                 */
  .word	DMA1_Channel2_3_IRQHandler			/* DMA1 Channel2 and 3 interrupts                                                 */
  .word	DMA1_Channel4_7_IRQHandler			/* DMA1 Channel4 to 7 interrupts                                                  */
  .word	ADC_COMP_IRQHandler       			/* ADC and comparator 1 and 2                                                     */
  .word	LPTIM1_IRQHandler         			/* LPTIMER1 interrupt through EXTI29                                              */
  .word	USART4_USART5_IRQHandler  			/* USART4/USART5 global interrupt                                                 */
  .word	TIM2_IRQHandler           			/* TIM2 global interrupt                                                          */
  .word	TIM3_IRQHandler           			/* TIM3 global interrupt                                                          */
  .word	TIM6_IRQHandler           			/* TIM6 global interrupt and DAC                                                  */
  .word	TIM7_IRQHandler           			/* TIM7 global interrupt and DAC                                                  */
  .word	0                         			/* Reserved                                                                       */
  .word	TIM21_IRQHandler          			/* TIMER21 global interrupt                                                       */
  .word	I2C3_IRQHandler           			/* I2C3 global interrupt                                                          */
  .word	TIM22_IRQHandler          			/* TIMER22 global interrupt                                                       */
  .word	I2C1_IRQHandler           			/* I2C1 global interrupt                                                          */
  .word	I2C2_IRQHandler           			/* I2C2 global interrupt                                                          */
  .word	SPI1_IRQHandler           			/* SPI1_global_interrupt                                                          */
  .word	SPI2_IRQHandler           			/* SPI2 global interrupt                                                          */
  .word	USART1_IRQHandler         			/* USART1 global interrupt                                                        */
  .word	USART2_IRQHandler         			/* USART2 global interrupt                                                        */
  .word	AES_RNG_LPUART1_IRQHandler			/* AES global interrupt RNG global interrupt and LPUART1 global interrupt through */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  	.weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler

  	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler

	.weak	WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler
	
	.weak	PVD_IRQHandler
	.thumb_set PVD_IRQHandler,Default_Handler
	
	.weak	RTC_IRQHandler
	.thumb_set RTC_IRQHandler,Default_Handler
	
	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler
	
	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler
	
	.weak	EXTI0_1_IRQHandler
	.thumb_set EXTI0_1_IRQHandler,Default_Handler
	
	.weak	EXTI2_3_IRQHandler
	.thumb_set EXTI2_3_IRQHandler,Default_Handler
	
	.weak	EXTI4_15_IRQHandler
	.thumb_set EXTI4_15_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel2_3_IRQHandler
	.thumb_set DMA1_Channel2_3_IRQHandler,Default_Handler
	
	.weak	DMA1_Channel4_7_IRQHandler
	.thumb_set DMA1_Channel4_7_IRQHandler,Default_Handler
	
	.weak	ADC_COMP_IRQHandler
	.thumb_set ADC_COMP_IRQHandler,Default_Handler
	
	.weak	LPTIM1_IRQHandler
	.thumb_set LPTIM1_IRQHandler,Default_Handler
	
	.weak	USART4_USART5_IRQHandler
	.thumb_set USART4_USART5_IRQHandler,Default_Handler
	
	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler
	
	.weak	TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,Default_Handler
	
	.weak	TIM6_IRQHandler
	.thumb_set TIM6_IRQHandler,Default_Handler
	
	.weak	TIM7_IRQHandler
	.thumb_set TIM7_IRQHandler,Default_Handler
	
	.weak	TIM21_IRQHandler
	.thumb_set TIM21_IRQHandler,Default_Handler
	
	.weak	I2C3_IRQHandler
	.thumb_set I2C3_IRQHandler,Default_Handler
	
	.weak	TIM22_IRQHandler
	.thumb_set TIM22_IRQHandler,Default_Handler
	
	.weak	I2C1_IRQHandler
	.thumb_set I2C1_IRQHandler,Default_Handler
	
	.weak	I2C2_IRQHandler
	.thumb_set I2C2_IRQHandler,Default_Handler
	
	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler
	
	.weak	SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler
	
	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler
	
	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler
	
	.weak	AES_RNG_LPUART1_IRQHandler
	.thumb_set AES_RNG_LPUART1_IRQHandler,Default_Handler

/************************ (C) COPYRIGHT Ac6 *****END OF FILE****/
