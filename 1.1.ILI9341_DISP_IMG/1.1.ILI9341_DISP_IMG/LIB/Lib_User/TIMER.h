#ifndef __TIMER_H
#define __TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
//#include "Smart_lock_RFID.h"


void Timer_Init(void);
void Timer_Gpio_Config(void);
void Timer_Interrupt_Config(void);

void TIM2_IRQHandler(void);
void Delay_ms(uint16_t time);



#endif
