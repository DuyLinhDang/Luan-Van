
#ifndef __TEST_H__
#define __TEST_H__
#include "lcd.h"
#include "delay.h"
#include "gui.h"
#include "touch.h"
#include "key.h" 
#include "led.h"
//#include "so6.h"

#include "string.h"
//#include "Nga2.h"
//#include "dautich.h"

void Test_Color(void);
void Test_FillRec(void);
void Test_Circle(void);
void Test_Triangle(void);
void English_Font_test(void);
void Chinese_Font_test(void);
void Pic_test(void);
void Load_Drow_Dialog(void);
void Touch_Test(void);
void main_test(void);
void Rotate_Test(void);

void Interface_90(void);

void Text_Center(u16 x1, u16 y1, u16 x2, u16 y2, u16 fc, u16 bc, u8 *str, u8 size,u8 mode);
void clear_text(u16 x1, u16 y1, u16 x2, u16 y2, u8 *str, u8 size,u8 mode);
void interface(void);
void Interface_Waiting(void);

void LCD_Clear_TIME(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd,u16 Color);
#endif
