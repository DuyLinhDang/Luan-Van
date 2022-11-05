#ifndef __UART_H
#define __UART_H

#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern char Str_Interrupt[1000];
extern int Cout_Interrupt;


void UART_Config		(USART_TypeDef* UARTx, unsigned int vrui_BaudRate);
void UART_SendChar 	(USART_TypeDef* UARTx, char vrsc_Char);
void UART_PutStr 		(USART_TypeDef* UARTx, char str[]);
char UART_Recieve 	(USART_TypeDef* UARTx);
void UART_RecieveStr(USART_TypeDef* UARTx);
void UART_Interrupt	(USART_TypeDef* UARTx);
void UART_Receive_DMA(USART_TypeDef* USARTx,char *pData,uint16_t len);

void USART1_IRQHandler	(void);
void USART2_IRQHandler	(void);
void USART3_IRQHandler	(void);

void Delay_ms(uint16_t time);

#endif 

