

#include <stdint.h>

#include "stm32f446xx.h"

//function prototypes of the init & configuration functions
void button_init(void);
void uart2_init(void);
void dma1_init(void);
void send_some_data(void);
void enable_dma1_stream6(void);
void  disable_dma1_stream6(void);
void dma1_interrupt_configuration(void);

//function prototypes of the callbacks
void HT_Complete_callback(void);
void FT_Complete_callback(void);
void TE_error_callback(void);
void DME_error_callback(void);
void FE_error_callback(void);

#define BASE_ADDR_OF_GPIOC_PERI  GPIOC

char data_stream[] ="Hello World\r\n";

int main(void)
{
	button_init();
	uart2_init();
	dma1_init();
	dma1_interrupt_configuration();
	enable_dma1_stream6();
	
	while(1);
	
	return 0;
}


void send_some_data(void)
{
	char somedata[] ="Hello World\r\n";
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	//make sure that in the status register TXE is set (TDR is empty )
	//if TXE is 1, put the byte 
	
	uint32_t len = sizeof(somedata);

	for ( uint32_t i =0 ; i < len ; i++)
	{
		//wer are waiting for TXE to become 1
		while( ! ( pUART2->SR & ( 1 << 7) ) ) ;
		
		pUART2->DR = somedata[i];
	}
	
	
}

void button_init(void)
{
	//button is connected to PC13 . GPIOC pin number 13. 
	
	GPIO_TypeDef *pGPIOC;
	pGPIOC = BASE_ADDR_OF_GPIOC_PERI;
	
	RCC_TypeDef *pRCC;
	pRCC = RCC;
	
	EXTI_TypeDef *pEXTI;
	pEXTI = EXTI;
	
	SYSCFG_TypeDef *pSYSCFG;
	pSYSCFG = SYSCFG;
	
	
	//1. enable the peripheral clock for the GPIOC peripheral 
	pRCC->AHB1ENR |= ( 1 << 2);
	
	//2. Keep the gpio pin in input mode
	pGPIOC->MODER &= ~( 0x3 << 26) ;
	
	//3. Enable the interrupt over that gpio pin
	pEXTI->IMR |= (1 << 13);
	
	//4. Enable the clock for SYSCFG 
	pRCC->APB2ENR |= ( 1 << 14);
	
	//5. Configuring the SYSCFG CR4 register 
	pSYSCFG->EXTICR[3] &= ~(0xF << 4); //Clearing 
	pSYSCFG->EXTICR[3] |=  (0x2 << 4); //set
	
	//6. Configure the edge detection for the EXTI 13 line. 
	pEXTI->FTSR |= ( 1 << 13);

	//7. Enable the IRQ related to that gpio pin in NVIC register of the processor . 
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	
	
}


void uart2_init(void)
{
	RCC_TypeDef *pRCC;
	pRCC = RCC;
	
	GPIO_TypeDef *pGPIOA;
	pGPIOA = GPIOA;
	
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	//1. enable the peripheral clock for the uart2 peripheral 
	pRCC->APB1ENR |= ( 1 << 17);
	
	//2. Configure the gpio pins for uart_tx and uart_rx functionality 
	     //PA2 as TX , PA3 as RX
	
	// lets configure PA2 as UART2 TX
	
  //2.1 Enable the clock for the GPIOA peripheral 
	pRCC->AHB1ENR |= ( 1 << 0);

  //2.2 Change the mode of the PA2 to alternate function mode
	pGPIOA->MODER &= ~(0x3 << 4); 
	pGPIOA->MODER |= (0x2 << 4);
	pGPIOA->AFR[0] &= ~( 0xF << 8);
	pGPIOA->AFR[0] |= ( 0x7 << 8);
  
  //2.3 Enable or disable Pull-up resistor of the gpio pin if required. 
  pGPIOA->PUPDR |= (0x1 << 4);//enables the pull up resistor on the gpio A pin number 2

	// lets configure PA3 as UART2 RX

  //2.4 Change the mode of the PA3 to alternate function mode
	pGPIOA->MODER  &= ~(0x3 << 6); 
	pGPIOA->MODER  |=  (0x2 << 6);
	pGPIOA->AFR[0] &= ~( 0xF << 12);
	pGPIOA->AFR[0] |=  ( 0x7 << 12);
  
  //2.5 Enable or disable Pull-up resistor for PA3	
	pGPIOA->PUPDR |= (0x1 << 6);
	
	
	
	//3 . Configure the baudrate 
	pUART2->BRR = 0x8B;
	
	//4 . configure the data width, no of stop bits , etc 
	// <no configuration reqd here , we will use default values 
	
	//5.  Enable the TX engine of the uart peripheral 
	pUART2->CR1 |= ( 1 << 3);
	
	//6.  Enable the uart peripheral 
	pUART2->CR1 |= ( 1 << 13);
	
	
}


void dma1_init(void)
{
	RCC_TypeDef *pRCC;
	pRCC = RCC;
	
	DMA_TypeDef *pDMA;
	pDMA=DMA1;
	
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	
	//1. enable the peripheral clock for the dma1
	pRCC->AHB1ENR |= ( 1 << 21);
	
	//2. identify the stream which is suitable for your peripheral
	     //channel 4 , stream 6
	
	//3. Identify the channel number on which uart2 peripheral sends dma request
	     //channel 4
	pSTREAM6->CR &= ~( 0x7 << 25);
	pSTREAM6->CR |=  ( 0x4 << 25);
	
	//4. Program the source address  (memory)
	pSTREAM6->M0AR = (uint32_t) data_stream;
	
	//5 . Program the destination address (peripheral )
	pSTREAM6->PAR = (uint32_t) &pUART2->DR;
	
	//6. Program number of data items to send 
	uint32_t len = sizeof(data_stream);
	pSTREAM6->NDTR = len;
	
	//7. The direction of data transfer . m2p, p2m , m2m
	pSTREAM6->CR |= (0x1 << 6);
	
	//8. Program the source and destination data width . 
	pSTREAM6->CR &= ~(0x3 << 13); 
	pSTREAM6->CR &= ~(0x3 << 11);
	
	//8a . enable memory auto increment 
	pSTREAM6->CR |= ( 1 << 10);
	
	//9. direct mode or fifo mode 
	pSTREAM6->FCR |= ( 1 << 2);
	
	//10. Select the fifo threshold 
	pSTREAM6->FCR &= ~(0x3 << 0); //clearing 
	pSTREAM6->FCR |=  (0x3 << 0); //setting
	
	//11. enable the circular mode if required 
	
	//12. single transfer or burst transfer . 
	
	//13. Configure the stream priority 
	

	
}

void  disable_dma1_stream6(void)
{
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	//Enable the stream
	pSTREAM6->CR &= ~( 1 << 0);
}

void enable_dma1_stream6(void)
{
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	//Enable the stream
	pSTREAM6->CR |= ( 1 << 0);
}


void dma1_interrupt_configuration(void)
{
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	
	//1. lets do Half-transfer IE (HTIE)
	pSTREAM6->CR |= (1 << 3);
	
	//2. Transfer complete IE (TCIE)
	pSTREAM6->CR |= (1 << 4);
	
	//3. Transfer error IE (TEIE)
	pSTREAM6->CR |= (1 << 2);
	
	//4 . FIFO overrun/underrun IE (FEIE)
	pSTREAM6->FCR |= (1 << 7);
	
	//5. Direct mode error (DMEIE)
	pSTREAM6->CR |= (1 << 1);
	
	//6. Enable the IRQ for DMA1 stream6 global interrupt in NVIC
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	
}

void HT_Complete_callback(void)
{

}

void FT_Complete_callback(void)
{
	USART_TypeDef *pUART2;
	pUART2 = USART2;
	
	DMA_Stream_TypeDef *pSTREAM6;
	pSTREAM6 = DMA1_Stream6;
	
	//Program number of data items to send 
	uint32_t len = sizeof(data_stream);
	pSTREAM6->NDTR = len;
	
	pUART2->CR3 &= ~( 1 << 7);
	
	enable_dma1_stream6();
		
}


void TE_error_callback(void)
{
	while(1);
}

void FE_error_callback(void)
{
	
	while(1);
}

void DME_error_callback(void)
{
	while(1);
}



