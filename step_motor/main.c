#include "stm32f10x.h"
#include "io.h"
#include "delay.h"
#include "pwm.h"
#define STEP0 PAout(0) // define pin step out
#define DIR0 	PAout(1) // define pin DIR out
void step(int step_number, int dir);
int main(void){
	delay_init();
	io_Set(GPIOA,GPIO_Pin_0 | GPIO_Pin_1,GPIO_Mode_Out_PP,GPIO_Speed_50MHz); // init pin PA0 and PA1 are output
  while (1)
  {
		//delay_ms(500);
		step(50,1);
		step(50,0);
		//step(400,1);
		//step(400,0);
		
	}
}
void step(int step_number, int dir){
	int i = 0;
	DIR0 = dir;
	for(i = 0; i<step_number;i++){
		STEP0 = 0;
		delay_us(8000);
		
		STEP0 = 1;
		delay_us(10);
	}
}
