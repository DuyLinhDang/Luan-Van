#include "system.h"

extern unsigned char gImage_hinh2[];

int main(){
	
	SystemInit();
	delay_init(72);	 
	LCD_Init();	
	while(1){
		Gui_DrawbmpUser(10,10,200,160,gImage_hinh2);
	}
}
