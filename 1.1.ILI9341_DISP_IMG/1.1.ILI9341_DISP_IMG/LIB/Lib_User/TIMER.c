#include "TIMER.h"

#define Led1 GPIO_Pin_0



TIM_TimeBaseInitTypeDef TIM2_InitStruct;
GPIO_InitTypeDef 				GPIO_InitStruct;
NVIC_InitTypeDef 				NVIC_InitStruct;



void Timer_Init(void){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM2_InitStruct.TIM_CounterMode 			= TIM_CounterMode_Up;
	TIM2_InitStruct.TIM_Period 						= 1000 - 1;
	TIM2_InitStruct.TIM_Prescaler 				= 72 - 1;
	TIM2_InitStruct.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2, &TIM2_InitStruct);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );
	
	TIM_Cmd(TIM2, ENABLE);
}


void Timer_Gpio_Config(void){
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,ENABLE);
	 
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Speed= GPIO_Speed_50MHz;
	 
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	
}

void Timer_Interrupt_Config(void){

	NVIC_PriorityGroupConfig ( NVIC_PriorityGroup_0 );
	NVIC_InitStruct.NVIC_IRQChannel 							= TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority 		=0;
	NVIC_InitStruct.NVIC_IRQChannelCmd 						= ENABLE ;
	NVIC_Init (&NVIC_InitStruct);
}
