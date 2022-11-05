#ifndef __SYSTEM_H
#define __SYSTEM_H


#include "GPIO.h" 
#include "spi.h"
#include "Timer.h"
#include "uart.h"
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"
#include "lcd.h"
#include "touch.h"
#include "gui.h"
#include "string.h"
#include "font7s.h"

void System_Init(void);
void System_Run(void);
	
void Text_Center(u16 x1, u16 y1, u16 x2, u16 y2, u16 fc, u16 bc, u8 *str, u8 size,u8 mode);
void main_test(void);
void Box_text(void);
void Index(void);
void Show_Info(const unsigned char *imagex);

#endif 
