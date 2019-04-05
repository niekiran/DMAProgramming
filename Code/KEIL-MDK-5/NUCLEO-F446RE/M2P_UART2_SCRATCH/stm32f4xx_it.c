
#include "stm32f446xx.h"

extern void HT_Complete_callback(void);
extern void FT_Complete_callback(void);
extern void TE_error_callback(void);
extern void DME_error_callback(void);
extern void FE_error_callback(void);

#define is_it_HT() 	DMA1->HISR & ( 1 << 20)
#define is_it_FT() 	DMA1->HISR & ( 1 << 21)
#define is_it_TE() 	DMA1->HISR & ( 1 << 19)
#define is_it_FE() 	DMA1->HISR & ( 1 << 16)
#define is_it_DME() DMA1->HISR & ( 1 << 18)


void clear_exti_pending_bit(void)
{
	EXTI_TypeDef *pEXTI;
	pEXTI = EXTI;
	
	if( (EXTI->PR & (1 << 13) )  )
	{
		//Clears the Pending bit 
		EXTI->PR |= (1 << 13);
	}

	
}






/************************************IRQ handlers*********************************/

//IRQ handler for the button interrupt 
void EXTI15_10_IRQHandler(void)
{
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	//here we will send UART2_TX DMA request to DMA1 controller
	pUART2->CR3 |= ( 1 << 7);
	
	clear_exti_pending_bit();
	
}

//IRQ handler for DMA1 stream6 global interrupt 

void DMA1_Stream6_IRQHandler(void)
{
	//Half-transfer
	if( is_it_HT() )
	{
		DMA1->HIFCR |= ( 1 << 20);
		HT_Complete_callback();
	}else if(is_it_FT() )
	{
		DMA1->HIFCR |= ( 1 << 21);
		FT_Complete_callback();
	}else if ( is_it_TE() )
	{
		DMA1->HIFCR |= ( 1 << 19);
		TE_error_callback();
		
	}else if (is_it_FE() )
	{
		DMA1->HIFCR |= ( 1 << 16);
		FE_error_callback();
	}else if( is_it_DME() )
	{
		DMA1->HIFCR |= ( 1 << 18);
		DME_error_callback();
	}else{
		  ;
	}
	 

}