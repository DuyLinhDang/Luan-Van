//***************************************Contact angle system******************************************
//********************Started 14/10/2022**********************************
//********************Author Dang Duy Linh B1806354***********************

//add lib
#include "stm32f10x.h"
#include "io.h"
#include "delay.h"
#include "exit.h"
#include "usart.h"
#include "cJSON.h"
#include "stm32f10x_flash.h"
#include "myFlash.h"
#include <String.h>
#include <stdlib.h>

//#define STEP0 PAout(3) // define pin step out
//#define DIR0 	PAout(2) // define pin DIR out

//***************************define macro*************************************
//define gpio output for step X-axis
#define DirX 		GPIO_Pin_0
#define StepX 	GPIO_Pin_1
//define gpio output for step Y-axis
#define DirY 		GPIO_Pin_2
#define StepY 	GPIO_Pin_3
//define gpio output for step Z-axis
#define DirZ		GPIO_Pin_4
#define StepZ 	GPIO_Pin_5
//define gpio output for step B-axis
#define DirB 		GPIO_Pin_6
#define StepB 	GPIO_Pin_7
//define gpio out MS1 for Z axis
#define MS1_Z 	GPIO_Pin_8

//define gpio exit for GPIO B
#define Exit_X	GPIO_Pin_0
#define Exit_Y	GPIO_Pin_1
#define Exit_Z	GPIO_Pin_3
#define Exit_B	GPIO_Pin_4
//define gpio output reverse for GPIO B


//define chieu quay dong co step
#define CW  	0  //xoay thuan
#define CCW 	1	 //xoay nghich

//define speed for step
#define Slow_X		1000
#define Fast_X		350
#define Slow_Y		1000
#define Fast_Y		370
#define Slow_Z		20000
#define Fast_Z		370
#define Midium_Z	1000
#define Slow_B		4000
#define Fast_B		2000

#define Distance_X_home		1600*3 //0.8cm
#define Distance_Y_home		400
#define Distance_Z_home		800

#define RxBufferSize 	100  
#define TxBufferSize 	100
//***************************init variable*************************************
//define distance X-axis, Y-axis, Z-axis, B-axis
const uint32_t distance_X 			= 1600*2; //khoang cach truc X, tinh theo so vong
const uint32_t distance_Y				= 1600*8; //khoang cach truc Y, tinh theo so vong
const uint32_t distance_Z1			= 1600*7; //khoang cach truc Z1, part 1, tinh theo so vong/lay nuoc
const uint32_t distance_Z2			= 1600*4; //khoang cach truc Z2, part 2, tinh theo so vong/lay nuoc
const uint32_t distance_B				= 95*14; //khoang cach truc B, tinh theo so vong
const uint32_t distance_drop		= 1000; //khoang cach Z1 slow, tinh theo so vong,0.4cm
uint16_t distance_slow_to_drop_water = 0;
// define avariabe to remerber number step of X-axis
uint32_t stepX_number 	= 1600;
// define avariabe to remerber number step of Y-axis
uint32_t stepY_number 	= 1600;
// define avariabe to remerber number step of Z-axis
uint32_t stepZ_number 	= 1600;
// define avariabe to remerber number step of B-axis
uint32_t stepB_number 	= 0;

//define flat for interrupt X-axis
uint8_t interrupt_X 	= 0;

//define flat for interrupt Y-axis
uint8_t interrupt_Y 	= 0;

//define flat for interrupt Z-axis
uint8_t interrupt_Z 	= 0;

//define flat for interrupt B-axis
uint8_t interrupt_B 	= 0;

//define flat for interrupt function home
uint8_t interrupt_home 	= 0;

//define variable to stop Z -axis of pipet
// get value to usart
uint8_t stop_Z_pipet 	= 0;

//define variable number water need to drop
// get value to usart
uint16_t water_drop_number 	= 1;
// define value for usart

char TxBuffer[TxBufferSize] = "USART1 Interrupt";

char RxBuffer[RxBufferSize] = "";
char RxBuffer_cpy[RxBufferSize] = "";
char data_RX = NULL;
__IO uint8_t RxCounter = 0x00;
uint8_t flat_excute_json;
//define json for each data
char JSON[200];
cJSON* DataJson;
cJSON* StartJson;
cJSON* StopJson;
cJSON* HomeJson;
cJSON* CycleJson;
cJSON* AddXJson;
cJSON* SubXJson;
cJSON* TakeWaterJson;
cJSON* DropJson;
cJSON* ImgJson;
cJSON* NumberDropJson;
cJSON* CheckErrorJson;
cJSON* DistanceJson;
//define variable for Json
char str_homeJson[10];
char str_imgJson[10];
char str_numberDropJson[10];
char str_checkErrorJson[10];
char str_distanceJson[10];
char str_StartJson[10];
char str_StopJson[10];
char str_CycleJson[10];
char str_AddXJson[10];
char str_SubXJson[10];
char str_TakeWaterJson[10];
char str_DropJson[10];

char homeJson[10];
char imgJson[10];
uint8_t numberDropJson = 0;
char checkErrorJson[50];
uint8_t distanceJson = 0;

uint8_t startJson = 0;
uint8_t stopJson = 0;
uint16_t cycleJson = 0;
uint16_t addXJson = 0;
uint16_t subXJson = 0;
uint16_t takeWaterJson = 0;
uint16_t dropJson = 0;
//***************************function*************************************
//function set home for system
void home(void);
//function set distance pipet and flatform (ban vat lieu)
void distance_pipet(void);
//function take water to drop
void take_water(void);
//function to drop water
void drop_water(void);
void drop_water_nofinish(void);
//function triger for PC to image excution
void triger_image_excution(void);
//function select GPIO PB3 is input
void select_pin_special_PB3();
//function to handle cjon
extern void cJSON_handler(char *data);
//function to clear data RX
void Clear_Data_RX();
//void step(int step_number, int dir);
void sendData();
uint16_t b = 0;
int main(void){
	//init delay
	delay_init();
	//int usart
	USARTx_Init(USART1,Pins_PA9PA10,9600);
	io_Set(GPIOC,GPIO_Pin_13,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	io_Set(GPIOB,GPIO_Pin_10,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	//init port output for step motor (pin A0-A8) 
	io_Set(GPIOA, StepX | DirX | StepY | DirY | StepZ | DirZ |StepB | DirB | MS1_Z,GPIO_Mode_Out_PP,GPIO_Speed_50MHz);
	//set MS1_Z is low
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	//init port input interrupt (pin B0 B1 B3)
	exit_Init(GPIOB, GPIO_Pin_0, Trigger_Rising);
	exit_Init(GPIOB, GPIO_Pin_1, Trigger_Rising);
	select_pin_special_PB3();
	exit_Init(GPIOB, GPIO_Pin_3, Trigger_Rising);
	//init port input interrupt to call function home
	exit_Init(GPIOB, GPIO_Pin_4, Trigger_Rising);
	//interrupt reverse
	exit_Init(GPIOB, GPIO_Pin_5, Trigger_Falling);
	exit_Init(GPIOB, GPIO_Pin_6, Trigger_Falling);
	home();
	//GPIO_SetBits(GPIOA,StepX);
	//step_motor(GPIOA,StepX,DirX,CCW,550,1600*100);
	//step_motor(GPIOA,StepY,DirY,CCW,500,1600*100);
	//step_motor(GPIOA,StepZ,DirZ,CCW,500,1600*100);
	//step_motor(GPIOA,StepB,DirB,CCW,500,1600*100);
	//home();
	//GPIO_ResetBits(GPIOA,MS1_Z);
//	USART_Puts(USART1,"hello");
//	sendData();
//	home();
//	delay_ms(500);
//	take_water();
//	delay_ms(500);
//	drop_water();
//	step_motor(GPIOA,StepB,DirB,CCW,5000,100*14);
//	//step_motor(GPIOA,StepB,DirB,CCW,60000,5);
//	delay_ms(1000);
//	step_motor(GPIOA,StepB,DirB,CW,5000,100*14);
	while(1)
	{
		if(takeWaterJson == 1){
			//call function take water
			take_water();
			//clear flat
			takeWaterJson = 0;
		}
		if(dropJson == 1){
			//call function drop water
			drop_water();
			//clear flat
			dropJson = 0;
		}
		if(addXJson == 1){
			//call function run X
			step_motor(GPIOA,StepX,DirX,CW,1000,400); // distance with 0.2cm
			//clear flat
			addXJson = 0;
		}
		if(subXJson == 1){
			//call function run X
			step_motor(GPIOA,StepX,DirX,CCW,1000,400); // distance with 0.2cm
			//clear flat
			subXJson = 0;
		}
		if(cycleJson == 1){
			//call function run X
			take_water();
			delay(100);
			drop_water();
			//clear flat
			cycleJson = 0;
		}
		if(startJson==1){
			// take water
			take_water();
			// drop
			drop_water();
			water_drop_number = water_drop_number - 1;
			step_motor(GPIOA,StepX,DirX,CW,1000,400*4);
			if(water_drop_number == 0){
				// chem
				GPIO_SetBits(GPIOB,GPIO_Pin_10);
				delay_ms(1000);
				GPIO_ResetBits(GPIOB,GPIO_Pin_10);
				// reset
				NVIC_SystemReset();	
		}
			
		}
		
	}
}
//***************************************************************function***********************************************************
//**************************************************function home***********************************************
void home(void)
{
	// xoay cac dong co ve vi tri dat cong tat hanh trinh, neu dang o do thi khong can xoay dong co
	// call function step and waiting interrupt Z-axis and pin of interrupt ==0
	while(!interrupt_Z && !GPIO_ReadInputDataBit(GPIOB,Exit_Z))
		{
		step_motor(GPIOA,StepZ,DirZ,CW,Fast_Z,1);
		}
	// clear flat interrupt Z-axis
		interrupt_Z = 0;
		
	// call function step and waiting interrupt X-axis and pin of interrupt ==0
	while(!interrupt_X && !GPIO_ReadInputDataBit(GPIOB,Exit_X))
		{
		step_motor(GPIOA,StepX,DirX,CCW,Fast_X,1);
		}
	// clear flat interrupt X-axis
		interrupt_X = 0;
		
	// call function step and waiting interrupt Y-axis and pin of interrupt ==0
	while(!interrupt_Y && !GPIO_ReadInputDataBit(GPIOB,Exit_Y))
		{
		step_motor(GPIOA,StepY,DirY,CCW,Fast_Y,1);
		}

	// clear flat interrupt Y-axis
		interrupt_Y = 0;
		
	// call function step and waiting interrupt B-axis and pin of interrupt ==0
	while(!interrupt_B && !GPIO_ReadInputDataBit(GPIOB,Exit_B))
		{
		step_motor(GPIOA,StepB,DirB,CW,Fast_B,1);
		}
	// clear flat interrupt B-axis
		interrupt_B = 0;
		delay_ms(500);
		step_motor(GPIOA,StepX,DirX,CW,Fast_X,Distance_X_home);
}
//**************************************function interrupt************************
// exti for line0, pin B0, X-axis
void EXTI0_IRQHandler(void)
{
	//set flat interrupt_X to 1
	interrupt_X = 1;
	EXTI_ClearFlag(EXTI_Line0);
} 
// exti for line1, pin B1, Y-axis
void EXTI1_IRQHandler(void)
{
	//set flat interrupt_Y to 1
	interrupt_Y = 1;
	EXTI_ClearFlag(EXTI_Line1);
}
// exti for line3, pin B3, Z-axis
void EXTI3_IRQHandler(void)
{
	//set flat interrupt_Z to 1
	interrupt_Z = 1;
	EXTI_ClearFlag(EXTI_Line3);
}
// exti for line4, pin B4, B-axis
void EXTI4_IRQHandler(void)
{
	//set flat interrupt_B to 1
	interrupt_B = 1;
	EXTI_ClearFlag(EXTI_Line4);
}
//******************************************function calculate distance pipet to flatform***********************************************
void distance_pipet()
{
	while(!stop_Z_pipet)
	{
		stepZ_number++;
		step_motor(GPIOA,StepZ,DirZ,CW ,1000,1);
	}
		
}
//**************************************************function take water***********************************************
void take_water(void)
{
	//run Y-axis, number_step = distance_Y,goto right
	step_motor(GPIOA, StepY, DirY, CW, Fast_Y, distance_Y);
	//add value to stepY_number
	//stepY_number = stepY_number + distance_Y;
	
	//run Z-axis, number_step=distance_Z1 goto part 1, run dowwn
	step_motor(GPIOA, StepZ, DirZ, CCW, Fast_Z, distance_Z1);
	//add value to stepZ_number
	//stepZ_number += distance_Z1;
	
	// press pipet
	step_motor(GPIOA, StepB, DirB, CCW, Fast_B, distance_B);
	//add value to stepB_number
	stepB_number += distance_B;
	
	//run Z-axis, number_step=distance_Z2 goto part 2, run down
	step_motor(GPIOA, StepZ, DirZ, CCW, Fast_Z, distance_Z2);
	//add value to stepZ_number
	stepZ_number += distance_Z2;
	
	interrupt_B = 0;
	//release pipet
	while(!interrupt_B && !GPIO_ReadInputDataBit(GPIOB,Exit_B))
	{
		step_motor(GPIOA,StepB,DirB,CW,Slow_B,1);
	}
	interrupt_B = 0;
	//sub value to stepB_number
//	stepB_number -= distance_B;
	interrupt_Z = 0;
	// run Z-axis, number_step = distance_Z2+distance_Z1, run up
	while(!interrupt_Z && !GPIO_ReadInputDataBit(GPIOB,Exit_Z))
	{
		step_motor(GPIOA,StepZ,DirZ,CW,Fast_Z,1);
	}
	interrupt_Z = 0;
	//sub value to stepZ_number
	//stepZ_number -= (distance_Z2 + distance_Z1);
	interrupt_Y = 0;
	//run Y-axis, number_step = distance_Y, goto left
	while(!interrupt_Y && !GPIO_ReadInputDataBit(GPIOB,Exit_Y))
	{
		step_motor(GPIOA,StepY,DirY,CCW,Fast_Y,1);
	}
	// clear flat interrupt Y-axis
	interrupt_Y = 0;
	//sub value to stepY_number
	//stepY_number = stepY_number - distance_Y;
}
//****************************************************function drop***********************************************
//******************************************************importance***********************************************
void drop_water(void)
{
	distance_slow_to_drop_water = 450;
	
	//check if distance_drop lower or equals distance_slow_to_drop_water then break function
	if(distance_drop <= distance_slow_to_drop_water)
	{
		return ;
	}
	
	//run down part 1, go fast, distance = distance_Z1 - distance_drop (2 cycles), 0.4cm
	step_motor(GPIOA, StepZ, DirZ, CCW, Midium_Z, (distance_drop));
	//add distance to stepZ_number
	stepZ_number += (distance_drop);
	
	//press pipet
	step_motor(GPIOA, StepB, DirB, CCW, Slow_B, distance_B + 14*8);
	//add value to stepB_number
	stepB_number += distance_B;
	
//	//set MS1_Z to high
//	GPIO_SetBits(GPIOA,MS1_Z);
	// run down part 2, go slow, distance_go_down_slow_to_drop
	step_motor(GPIOA, StepZ, DirZ, CCW, Slow_Z, distance_slow_to_drop_water);
	//add distance to stepZ_number
	distance_slow_to_drop_water++;
	//time delay when drop water
	delay_ms(100);
//	//set MS1_Z to low
//	GPIO_ResetBits(GPIOA,MS1_Z);
	// run up part 1, go slow, distance_go_up_slow_to_out
	step_motor(GPIOA, StepZ, DirZ, CW, Midium_Z, distance_slow_to_drop_water);
	//sub value to stepZ_number
	//stepZ_number -= distance_slow_to_drop_water;
	interrupt_B = 0;
	//release pipet
	while(!interrupt_B && !GPIO_ReadInputDataBit(GPIOB,Exit_B))
	{
		step_motor(GPIOA,StepB,DirB,CW,Slow_B,1);
	}
	interrupt_B = 0;
	//sub value to stepB_number
	stepB_number -= distance_B;
	
	// run up part 2, go fast
	while(!interrupt_Z && !GPIO_ReadInputDataBit(GPIOB,Exit_Z))
	{
		step_motor(GPIOA,StepZ,DirZ,CW,Midium_Z,1);
	}
	interrupt_Z = 0;
	//sub value to stepZ_number
}

void drop_water_without_distance(void)
{

	
	//check if distance_drop lower or equals distance_slow_to_drop_water then break function
	if(distance_drop <= distance_slow_to_drop_water)
	{
		return ;
	}
	
	//run down part 1, go fast, distance = distance_Z1 - distance_drop (2 cycles), 0.4cm
	step_motor(GPIOA, StepZ, DirZ, CCW, Midium_Z, (distance_drop));
	//add distance to stepZ_number
	stepZ_number += (distance_drop);
	
	//press pipet
	step_motor(GPIOA, StepB, DirB, CCW, Slow_B, distance_B);
	//add value to stepB_number
	stepB_number += distance_B;
	
//	//set MS1_Z to high
//	GPIO_SetBits(GPIOA,MS1_Z);
	while(!distanceJson){
		// run down part 2, go slow, distance_go_down_slow_to_drop
		step_motor(GPIOA, StepZ, DirZ, CCW, Slow_Z, 1);
		//add distance to stepZ_number
		distance_slow_to_drop_water++;
		if(distance_slow_to_drop_water>700)
			// reset when no receive signal
			NVIC_SystemReset();
	}
	//time delay when drop water
	delay_ms(600);
	// run up part 1, go slow, distance_go_up_slow_to_out
	step_motor(GPIOA, StepZ, DirZ, CW, Midium_Z, distance_slow_to_drop_water);
	//sub value to stepZ_number
	stepZ_number -= distance_slow_to_drop_water;
	
	//trigger for handler img
	USART_Puts(USART1,"xuly1anh");
	//release pipet
	step_motor(GPIOA, StepB, DirB, CW, Fast_B, distance_B);
	//sub value to stepB_number
	stepB_number -= distance_B;
	
	// run up part 2, go fast
	step_motor(GPIOA, StepZ, DirZ, CW ,Midium_Z, (distance_drop));
	//sub value to stepZ_number
	stepZ_number -= (distance_drop);
}
void drop_water_with_distance(void)
{

	
	//check if distance_drop lower or equals distance_slow_to_drop_water then break function
	if(distance_drop <= distance_slow_to_drop_water)
	{
		return ;
	}
	//run down part 1, go fast, distance = distance_Z1 - distance_drop (2 cycles), 0.4cm
	step_motor(GPIOA, StepZ, DirZ, CCW, Midium_Z, (distance_drop + distance_slow_to_drop_water - 100));
	//add distance to stepZ_number
	stepZ_number += (distance_drop);
	
	//press pipet
	step_motor(GPIOA, StepB, DirB, CCW, Slow_B, distance_B);
	//add value to stepB_number
	stepB_number += distance_B;
	
	// run down part 2, go slow, distance_go_down_slow_to_drop
	step_motor(GPIOA, StepZ, DirZ, CCW, Slow_Z, 100);

	//time delay when drop water
	delay_ms(600);
	// run up part 1, go slow, distance_go_up_slow_to_out
	step_motor(GPIOA, StepZ, DirZ, CW, Midium_Z, 100);
	//sub value to stepZ_number
	stepZ_number -= distance_slow_to_drop_water;
	
	//trigger for handler img
	USART_Puts(USART1,"xuly1anh");
	//release pipet
	step_motor(GPIOA, StepB, DirB, CW, Fast_B, distance_B);
	//sub value to stepB_number
	stepB_number -= distance_B;
	
	// run up part 2, go fast
	step_motor(GPIOA, StepZ, DirZ, CW ,Midium_Z, (distance_drop + distance_slow_to_drop_water - 100));
	//sub value to stepZ_number
	stepZ_number -= (distance_drop);
}
//******************************************function triger for PC to image excution***********************************************
void triger_image_excution(void)
{
	//do something
}
//*************************************function interrupt to call function home***********************************************
void EXTI9_5_IRQHandler(void)
{
	/* Make sure that interrupt flag is set for line 5 */
	if (EXTI_GetITStatus(EXTI_Line5) != RESET)
		{
		//set flat to interrupt for function home
			
		//call function home
		EXTI_ClearFlag(EXTI_Line5);
		}
		/* Make sure that interrupt flag is set for line 6 */
		if (EXTI_GetITStatus(EXTI_Line6) != RESET)
		{
			
		EXTI_ClearFlag(EXTI_Line6);
		}
}
//*************************************function select pin to gpio***********************************************
void select_pin_special_PB3()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//Enable porta, porte clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);    //Enable multiplex function clock
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE );

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//*************************************function usart interrupt***********************************************
void USART1_IRQHandler(void)
{
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
     data_RX = USART_ReceiveData(USART1);
		if(data_RX != '\n') RxBuffer[RxCounter++] = data_RX;
		else{
		// set flat to call function handle json
		//flat_excute_json = 1;
		for(b=0;b<100;b++){
			if(RxBuffer[b] == 0x27){
				RxBuffer_cpy[b] = 0x22;
				}
			else{
				RxBuffer_cpy[b] = RxBuffer[b];
				}
				}
		cJSON_handler(RxBuffer_cpy);
		}
	}
}
//************************************function handle json when it has interrupt********************************
void cJSON_handler(char *data){
	//add dato to format json
	DataJson = cJSON_Parse(data);
	// get object json
	HomeJson 				= cJSON_GetObjectItem(DataJson,"Home");
	ImgJson 				= cJSON_GetObjectItem(DataJson,"Img");
	NumberDropJson 	= cJSON_GetObjectItem(DataJson,"NumberDrop");
	DistanceJson 		= cJSON_GetObjectItem(DataJson,"Distance");
	CheckErrorJson 	= cJSON_GetObjectItem(DataJson,"CheckError");
	StartJson 			= cJSON_GetObjectItem(DataJson,"Start");
	StopJson 				= cJSON_GetObjectItem(DataJson,"Stop");
	CycleJson 			= cJSON_GetObjectItem(DataJson,"Cycle");
	AddXJson 				= cJSON_GetObjectItem(DataJson,"Add_x");
	SubXJson 				= cJSON_GetObjectItem(DataJson,"Sub_x");
	TakeWaterJson 	= cJSON_GetObjectItem(DataJson,"Take_water");
	DropJson 				= cJSON_GetObjectItem(DataJson,"Drop_water");
	// handler json
	//home
	if(HomeJson==NULL){
		//do something
		//a = 1;
	}
	else{
	if(strstr(HomeJson->valuestring,"ON")){
		NVIC_SystemReset();
		//home();
		}
		else{ // "OFF"
			//do something
			//a = 1;
		}
	}
	//Img
	if(ImgJson==NULL){
	//do something
		//a = 1;
	}
	else{
		if(strstr(ImgJson->valuestring,"ON")){
			//do something
			//a = 1;
			
		}
		else{ // "OFF"
			//do something
			//a = 1;
		}
	}
	//CheckError
	if(CheckErrorJson==NULL){
		//do something
		//a = 1;
	}
	else{
		if(strstr(CheckErrorJson->valuestring,"da_truyen_thanh_cong")){
			//do something
			//a = 1;
		}
		else{ // truyen error
			//do something
			//a = 1;
		}
	}
	//Number drop
//	if(NumberDropJson==NULL){
//		//do something
//		//a = 1;
//	}
//	else{
//		//numberDropJson = NumberDropJson->valuedouble;
//	}
	//Number drop
	if(DistanceJson==NULL){
		//do something
		//a = 1;
	}
	else{
		if(strstr(DistanceJson->valuestring,"stop")){
		// run
			distanceJson = 1;
		}
		else{ // "OFF"
			//do something
		}
	}
	
	//Start
	if(StartJson==NULL){
		//do something
	}
	else{
	if(strstr(StartJson->valuestring,"start")){
		// run
		startJson = 1;
		stopJson = 0;
		if(water_drop_number == 0){
			water_drop_number = 10;
			}
		}
		else{ // "OFF"
			//do something
		}
	}
	//Stop
	if(StopJson==NULL){
		//do something
	}
	else{
	if(strstr(StopJson->valuestring,"stop")){
		// run
		stopJson = 1;
		startJson = 0;
		}
		else{ // "OFF"
			//do something
		}
	}
	
	//cycle
	if(CycleJson==NULL){
		//do something
	}
	else{
	if(strstr(CycleJson->valuestring,"run")){
		// run
		cycleJson = 1;
		}
		else{ // "OFF"
			//do something
		}
	}
	//add x
	if(AddXJson==NULL){
		//do something
	}
	else{
	if(strstr(AddXJson->valuestring,"add")){
		// run
		addXJson = 1;
		}
		else{ // "OFF"
			//do something
		}
	}
	//sub x
	if(SubXJson==NULL){
		//do something
	}
	else{
	if(strstr(SubXJson->valuestring,"sub")){
		// run
		subXJson = 1;
		}
	else{ // "OFF"
			//do something
		}
	}

	//take water
	if(TakeWaterJson==NULL){
		//do something
	}
	else{
	if(strstr(TakeWaterJson->valuestring,"take_water")){
		// run
		takeWaterJson = 1;
		}
		else{ // "OFF"
			//do something
		}
	}
	//drop
	if(DropJson==NULL){
		//do something
	}
	else{
	if(strstr(DropJson->valuestring,"drop_water")){
		// run
		dropJson = 1;
		}
		else{ // "OFF"
			//do something
		}
	}
	flat_excute_json = 0;
	//clear data json
	cJSON_Delete(DataJson);
	//clear data RX
	Clear_Data_RX();
}
//***************************function to clear data RX
void Clear_Data_RX(){
	u16 j=0;
	for(j=0;j<RxBufferSize;j++){
		RxBuffer[j] = NULL;
	}
	RxCounter = 0;
}

void sendData(){
//	clear data in char
	int i = 0;
	for(i=0;i<10;i++){
		str_homeJson[i] = 0;
		str_imgJson[i] = 0;
		str_numberDropJson[i] = 0;
		str_distanceJson[i] = 0;
	}
	for(i=0;i<50;i++){
		str_checkErrorJson[i] = 0;
	}
	//save data to aray
	sprintf(str_homeJson,"%s",homeJson);
	sprintf(str_imgJson,"%s",imgJson);
	sprintf(str_numberDropJson,"%d",numberDropJson);
	sprintf(str_distanceJson,"%d",distanceJson);
	sprintf(str_checkErrorJson,"%s",checkErrorJson);
	//mix string 
	//home
	strcat(JSON,"{\"Home\":\"");
	strcat(JSON,str_homeJson);
	strcat(JSON,"\",");
	//Img
	strcat(JSON,"\"Img\":\"");
	strcat(JSON,str_imgJson);
	strcat(JSON,"\",");
	//number drop
	strcat(JSON,"\"NumberDrop\":");
	strcat(JSON,str_numberDropJson);
	strcat(JSON,",");
	//Distance
	strcat(JSON,"\"Distance\":");
	strcat(JSON,str_distanceJson);
	strcat(JSON,",");
	//check error
	strcat(JSON,"\"CheckError\":\"");
	strcat(JSON,str_checkErrorJson);
	strcat(JSON,"\"}");
	printf("%s\n",JSON);
}