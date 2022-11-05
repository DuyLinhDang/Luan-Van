#include "system.h"




/*
ham Text_Center: su dung de viet can le giua trong pham vi chi dinh.
*/
void Text_Center(u16 x1, u16 y1, u16 x2, u16 y2, u16 fc, u16 bc, u8 *str, u8 size,u8 mode){
	int Width_x = x2-x1;
	int length_text = strlen((const char *)str);
	int Width_text = length_text*8;
	int X_text = (Width_x - Width_text)/2;
	int Y_text = ((y2-y1)-13)/2;
	Show_Str(X_text+x1,Y_text+y1,fc, bc, str, size, mode);
}



void Show_Info(const unsigned char *imagex){
	LCD_DrawRectangle(9,64,80,165);// boder for image size: 70x100
	Gui_DrawbmpUser(10,65,70,100,imagex);
	Show_Str(35,170,BLUE,BLUE,"MSV",16,1);
//	Show_Str(5,188,RED,BLUE,"x",16,1);
	Show_Str(150,60,RED,BLUE,"STUDENT CARD",16,1);
	Show_Str(100,85,BLUE,BLUE, "Full Name: ",16,1);
	Show_Str(100,105,BLUE,BLUE,"Gender   : ",16,1);
	Show_Str(100,125,BLUE,BLUE,"DBirth   : ",16,1);
	Show_Str(100,145,BLUE,BLUE,"ID Card  : ",16,1);
	Show_Str(100,165,BLUE,BLUE,"Class    : ",16,1);
}

void main_test(void){
//	LCD_direction(USE_HORIZONTAL);
	Gui_StrCenter(0,30+15,RED,BLUE,"Phu dung som no",16,1);//居中显示
	Gui_StrCenter(0,60+15,RED,BLUE,"toi lai tan,",16,1);//居中显示	
	Gui_StrCenter(0,90+15,BLUE,BLUE,"Long nguoi de doi ",16,1);//居中显示
	Gui_StrCenter(0,120+15,BLUE,BLUE,"cung nhu hoa,",16,1);//居中显示
	Gui_StrCenter(0,150+15,RED,BLUE,"Nguyen the suot kiep,",16,1);
	Gui_StrCenter(0,180+15,RED,BLUE,"luon chung thuy,",16,1);
	Gui_StrCenter(0,210+15,BLUE,BLUE,"May ai giu duoc",16,1);//居中显示
	Gui_StrCenter(0,240+15,BLUE,BLUE,"den cuoi doi,",16,1);//居中显示
//	delay_ms(10000);
}

