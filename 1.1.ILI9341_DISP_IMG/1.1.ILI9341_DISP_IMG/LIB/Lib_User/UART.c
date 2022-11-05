#include "UART.h"
#include "stdio.h"

struct __FILE {
    int dummy;
};
FILE __stdout;
 
int fputc(int ch, FILE *f) {
 
    UART_SendChar(USART1, ch);
  
    return ch;
}

GPIO_InitTypeDef			GPIO_InitStructure;
USART_InitTypeDef			UART_InitStructure;
NVIC_InitTypeDef			NVIC_InitStructure;

char Str_Interrupt[1000];
int Cout_Interrupt = 0;

void UART_Config(USART_TypeDef* UARTx,unsigned int vrui_BaudRate){
	if(UARTx == USART1){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin 								= GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode								= GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed								= GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin 								= GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode 								= GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	}
	if(UARTx == USART2){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin 								= GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode								= GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed								= GPIO_Speed_50MHz;
		GPIO_Init(GPIOA , &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin 								= GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode 								= GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	}
	if(UARTx == USART3){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin 								= GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode								= GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed								= GPIO_Speed_50MHz;
		GPIO_Init(GPIOB , &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin 								= GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode 								= GPIO_Mode_IN_FLOATING;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	}
	
	UART_InitStructure.USART_BaudRate 						= vrui_BaudRate;
	UART_InitStructure.USART_WordLength 					= USART_WordLength_8b;
	UART_InitStructure.USART_StopBits 						= USART_StopBits_1;
	UART_InitStructure.USART_Parity 							= USART_Parity_No;
	UART_InitStructure.USART_HardwareFlowControl 	= USART_HardwareFlowControl_None;
	UART_InitStructure.USART_Mode 								= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UARTx, &UART_InitStructure);
		
	USART_Cmd(UARTx , ENABLE);
}

void UART_Interrupt(USART_TypeDef* UARTx){
	
	/* Cau hinh vector ngat va muc uu tien */
	
	if(UARTx == USART1) NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	if(UARTx == USART2) NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	if(UARTx == USART3) NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd 								= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* xoa co ngat nhan cho lan dau su dung*/
	USART_ClearFlag(UARTx, USART_IT_RXNE);
	
	USART_ITConfig(UARTx , USART_IT_RXNE , ENABLE);

}

void UART_Receive_DMA(USART_TypeDef* USARTx,char *pData,uint16_t len){

		DMA_InitTypeDef DMA_InitStructure;
		/* USART1_RX DMA1 Channel 6 (See RM0008) */
	
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

		DMA_DeInit(DMA1_Channel3);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USARTx->DR;
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pData;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
		DMA_InitStructure.DMA_BufferSize = len - 1;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(DMA1_Channel3, &DMA_InitStructure);
		USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
		/* Enable DMA Stream Transfer Complete interrupt */
		DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
		
		DMA_Cmd(DMA1_Channel3, ENABLE);
		
}

void UART_SendChar (USART_TypeDef* UARTx,char vrsc_Char){
	USART_SendData(UARTx, vrsc_Char);
	while (USART_GetFlagStatus(UARTx, USART_FLAG_TXE) == RESET); //Cho den khi gui di xong
}

void UART_PutStr (USART_TypeDef* UARTx, char * vrsc_String){
	while(* vrsc_String){
		UART_SendChar(UARTx ,* vrsc_String);
		vrsc_String++;
	}
}

char UART_Recieve (USART_TypeDef* UARTx){
	while (USART_GetFlagStatus(UARTx, USART_IT_RXNE) == RESET) // Cho co nhan Reset
    return (USART_ReceiveData(UARTx));
	return 0;
}

void UART_RecieveStr(USART_TypeDef* UARTx){
	
	int data,i;
	
	if(Cout_Interrupt == 0){ for(i=0;i<100;i++) Str_Interrupt[i]=0;}

	data = USART_ReceiveData(UARTx);
	
	Str_Interrupt[Cout_Interrupt++] = data;
				
}


void Delay_ms(uint16_t time){
	unsigned int i,j;
	for( i=0;i<time;i++){
		for( j=0;j<=0x2aff;j++);
	}
}


void USART1_IRQHandler(void){
	uint16_t data;
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		
		data = USART_ReceiveData(USART1);
		
		if(USART_GetITStatus(USART1, USART_IT_TXE) == RESET){
			USART_SendData(USART1,data);
		}
  }
}

void USART2_IRQHandler(void){
	uint16_t data;
  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {	
		data = USART_ReceiveData(USART3);

		if(USART_GetITStatus(USART2, USART_IT_TXE) == RESET){
			USART_SendData(USART2,data);
		}
  }
}

void USART3_IRQHandler(void){
	uint16_t data;
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
		
		data = USART_ReceiveData(USART3);
		Str_Interrupt[Cout_Interrupt++] = data;

		if(USART_GetITStatus(USART3, USART_IT_TXE) == RESET){
			
		}
  }
}
