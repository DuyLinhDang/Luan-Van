/****************************************************************************************************
//=================================================================================//
//     	LCD              		STM32
//      VCC             		DC5V/3.3V     
//      GND              		GND        
//=================================================================================//
//
//     	LCD              		STM32   
//   	 	SDI(MOSI)        		PB15         
//    	SDO(MISO)         	PB14         
//=================================================================================//
//     	LCD				      		STM32
//      LED               	3.3          
//      SCK                 PB13         
//      DC/RS               PB8          
//      RST                 PB12       
//      CS                  PB9          
//==================================================================================//
//
//	   	LCD             		STM32
//      T_IRQ               PC10       
//      T_DO                PC2          
//      T_DIN               PC3         
//      T_CS                PC13         
//      T_CLK               PC0         
**************************************************************************************************/	
#include "stm32f10x.h"
#include "led.h"

/*****************************************************************************
 * @name       :void LED_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialize LED GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
void LED_Init(void)
{ 
 GPIO_InitTypeDef  GPIO_InitStructure;	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能GPIOA时钟 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				  //PA8
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
 GPIO_Init(GPIOA, &GPIO_InitStructure);
 GPIO_SetBits(GPIOA,GPIO_Pin_8);						 
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);   //使能GPIOD时钟 
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	    		 //PD2
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		  //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
 GPIO_Init(GPIOD, &GPIO_InitStructure);	
 GPIO_SetBits(GPIOD,GPIO_Pin_2); 						 
}
 
