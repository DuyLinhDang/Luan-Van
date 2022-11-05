#include "GPIO.h" 
/*
GPIO_Pin: GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|
					GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|
					GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
*/

typedef struct {
	GPIO_TypeDef * 			GPIOx;
	uint16_t 						GPIO_Pin_x;
	uint32_t 						RCC_APBXPeriph_x;
}Pin_Confi;
/*******************************************************************************************************/
const Pin_Confi Pin_val[]={
	{GPIOA, GPIO_Pin_0,  RCC_APB2Periph_GPIOA}, //  0
	{GPIOA, GPIO_Pin_1,  RCC_APB2Periph_GPIOA}, //  1
	{GPIOA, GPIO_Pin_2,  RCC_APB2Periph_GPIOA}, //  2
	{GPIOA, GPIO_Pin_3,  RCC_APB2Periph_GPIOA}, //  3
	{GPIOA, GPIO_Pin_4,  RCC_APB2Periph_GPIOA}, //  4
	{GPIOA, GPIO_Pin_5,  RCC_APB2Periph_GPIOA}, //  5
	{GPIOA, GPIO_Pin_6,  RCC_APB2Periph_GPIOA}, //  6
	{GPIOA, GPIO_Pin_7,  RCC_APB2Periph_GPIOA}, //  7 
	{GPIOA, GPIO_Pin_8,  RCC_APB2Periph_GPIOA}, //  8
	{GPIOA, GPIO_Pin_9,  RCC_APB2Periph_GPIOA}, //  9 
	{GPIOA, GPIO_Pin_10, RCC_APB2Periph_GPIOA}, //  10 
	{GPIOA, GPIO_Pin_11, RCC_APB2Periph_GPIOA}, //  11
	{GPIOA, GPIO_Pin_12, RCC_APB2Periph_GPIOA}, //  12
	{GPIOA, GPIO_Pin_13, RCC_APB2Periph_GPIOA}, //  13
	{GPIOA, GPIO_Pin_14, RCC_APB2Periph_GPIOA}, //  14
	{GPIOA, GPIO_Pin_15, RCC_APB2Periph_GPIOA}, //  15
	
	{GPIOB, GPIO_Pin_0,  RCC_APB2Periph_GPIOB}, //  16     
  {GPIOB, GPIO_Pin_1,  RCC_APB2Periph_GPIOB}, //  17         
  {GPIOB, GPIO_Pin_2,  RCC_APB2Periph_GPIOB}, //  18
	{GPIOB, GPIO_Pin_3,  RCC_APB2Periph_GPIOB}, //  19        
  {GPIOB, GPIO_Pin_4,  RCC_APB2Periph_GPIOB}, //  20        
  {GPIOB, GPIO_Pin_5,  RCC_APB2Periph_GPIOB}, //  21
	{GPIOB, GPIO_Pin_6,  RCC_APB2Periph_GPIOB}, //  22        
  {GPIOB, GPIO_Pin_7,  RCC_APB2Periph_GPIOB}, //  23        
  {GPIOB, GPIO_Pin_8,  RCC_APB2Periph_GPIOB}, //  24
	{GPIOB, GPIO_Pin_9,  RCC_APB2Periph_GPIOB}, //  25       
  {GPIOB, GPIO_Pin_10, RCC_APB2Periph_GPIOB}, //  26         
  {GPIOB, GPIO_Pin_11, RCC_APB2Periph_GPIOB}, //  27
	{GPIOB, GPIO_Pin_12, RCC_APB2Periph_GPIOB}, //  28        
  {GPIOB, GPIO_Pin_13, RCC_APB2Periph_GPIOB}, //  29         
  {GPIOB, GPIO_Pin_14, RCC_APB2Periph_GPIOB}, //  30
	{GPIOB, GPIO_Pin_15, RCC_APB2Periph_GPIOB}, //  31
	
	{GPIOC, GPIO_Pin_0,  RCC_APB2Periph_GPIOC}, //  32    
  {GPIOC, GPIO_Pin_1,  RCC_APB2Periph_GPIOC}, //  33         
  {GPIOC, GPIO_Pin_2,  RCC_APB2Periph_GPIOC}, //  34
	{GPIOC, GPIO_Pin_3,  RCC_APB2Periph_GPIOC}, //  35        
  {GPIOC, GPIO_Pin_4,  RCC_APB2Periph_GPIOC}, //  36        
  {GPIOC, GPIO_Pin_5,  RCC_APB2Periph_GPIOC}, //  37
	{GPIOC, GPIO_Pin_6,  RCC_APB2Periph_GPIOC}, //  38        
  {GPIOC, GPIO_Pin_7,  RCC_APB2Periph_GPIOC}, //  39        
  {GPIOC, GPIO_Pin_8,  RCC_APB2Periph_GPIOC}, //  40
	{GPIOC, GPIO_Pin_9,  RCC_APB2Periph_GPIOC}, //  41       
  {GPIOC, GPIO_Pin_10, RCC_APB2Periph_GPIOC}, //  42         
  {GPIOC, GPIO_Pin_11, RCC_APB2Periph_GPIOC}, //  43
	{GPIOC, GPIO_Pin_12, RCC_APB2Periph_GPIOC}, //  44        
  {GPIOC, GPIO_Pin_13, RCC_APB2Periph_GPIOC}, //  45         
  {GPIOC, GPIO_Pin_14, RCC_APB2Periph_GPIOC}, //  46
	{GPIOC, GPIO_Pin_15, RCC_APB2Periph_GPIOC}, //  47
};
/*******************************************************************************************************/
void pinMode(uint16_t PINx, uint16_t MODE){
	GPIO_InitTypeDef gpio_typedef;
/*-------------------enable clock-------------------*/
	RCC_APB2PeriphClockCmd(Pin_val[PINx].RCC_APBXPeriph_x, ENABLE);
	gpio_typedef.GPIO_Pin = Pin_val[PINx].GPIO_Pin_x;
/*--------------------------------------------------*/		
	if (MODE == INPUT) { 
		gpio_typedef.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
} 
/*--------------------------------------------------*/		
	else if (MODE == INPUT_PULLUP) {
		gpio_typedef.GPIO_Mode = GPIO_Mode_IPU;
		gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	} 
/*--------------------------------------------------*/		
	else if (MODE == INPUT_PULLDOWN) {
		gpio_typedef.GPIO_Mode = GPIO_Mode_IPD;
		gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	}
/*--------------------------------------------------*/		
	else if (MODE == OUTPUT) {
		gpio_typedef.GPIO_Mode = GPIO_Mode_Out_PP;
		gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	}
/*--------------------------------------------------*/		
	else if (MODE == OUTPUT_OD) {
		gpio_typedef.GPIO_Mode = GPIO_Mode_Out_OD;
		gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	} 
/*--------------------------------------------------*/			
	else if (MODE == AF_OD) {
		gpio_typedef.GPIO_Mode = GPIO_Mode_AF_OD;
		gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	}
/*--------------------------------------------------*/			
	else if (MODE == AF_PP) {
		 gpio_typedef.GPIO_Mode = GPIO_Mode_AF_PP;
		 gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	}
/*--------------------------------------------------*/		
	else if (MODE == AN_INPUT) {
		gpio_typedef.GPIO_Mode = GPIO_Mode_AIN;
		gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	}
/*--------------------------------------------------*/		
	GPIO_Init(Pin_val[PINx].GPIOx, &gpio_typedef);        
} 
/*******************************************************************************************************/
void digitalWrite(uint16_t PINx, uint16_t value){
	if(value == HIGH){
		Pin_val[PINx].GPIOx -> BSRR = Pin_val[PINx].GPIO_Pin_x;
	}
	else{
		Pin_val[PINx].GPIOx -> BRR = Pin_val[PINx].GPIO_Pin_x;
	}
}
/*******************************************************************************************************/
int digitalRead(uint16_t PINx){
	return GPIO_ReadInputDataBit(Pin_val[PINx].GPIOx, Pin_val[PINx].GPIO_Pin_x);
}
/*******************************************************************************************************/
/*LED pin configure*/
void GPIOx_Set(GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin_x, GPIOMode_TypeDef GPIO_Mode_x, GPIOSpeed_TypeDef GPIO_Speed_x){
	GPIO_InitTypeDef gpio_typedef;
	if(GPIOx==GPIOA){
		// enable clock for GPIOA
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	}
/*--------------------------------------------------*/
	else if(GPIOx==GPIOB){
		// enable clock for GPIOB
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	}
/*--------------------------------------------------*/
	else if(GPIOx==GPIOC){
		// enable clock for GPIOC
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	}
/*--------------------------------------------------*/
	gpio_typedef.GPIO_Pin = GPIO_Pin_x;
	gpio_typedef.GPIO_Mode = GPIO_Mode_x;
	gpio_typedef.GPIO_Speed = GPIO_Speed_x;
	GPIO_Init(GPIOx,&gpio_typedef);
}
/*******************************************************************************************************/
void GPIOx_ConfiLed(void){
	GPIO_InitTypeDef gpio_typedef;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	gpio_typedef.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_typedef.GPIO_Pin = GPIO_Pin_0;
	gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_typedef);
}
/*******************************************************************************************************/
void  GPIOx_ConfiBut(void){
	GPIO_InitTypeDef gpio_typedef;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	gpio_typedef.GPIO_Mode = GPIO_Mode_IPU;
	gpio_typedef.GPIO_Pin = GPIO_Pin_0;
	gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_typedef);
}
/*******************************************************************************************************/
void  GPIOx_Confi7Seg(void){
	GPIO_InitTypeDef gpio_typedef;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	gpio_typedef.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_typedef.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|
									GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;				
	gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_typedef);
}
/*******************************************************************************************************/
void  GPIOx_ConfisMaTrix(void){
	GPIO_InitTypeDef gpio_typedef;
	/*cap xung clock cho ngoai vi*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	
	/*cau hinh chan dieu khien cot PA0->PA7*/
	gpio_typedef.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_typedef.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4|
								  GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
	gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_typedef);
	
	/*cau hinh chan dieu khien hang PB8-PB15*/
	gpio_typedef.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_typedef.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9| GPIO_Pin_10| GPIO_Pin_11| GPIO_Pin_12|
								  GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	gpio_typedef.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio_typedef);	
}



