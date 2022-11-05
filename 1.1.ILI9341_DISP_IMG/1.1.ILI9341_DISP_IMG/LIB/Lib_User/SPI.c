/**************************************************************************************************/	
#include "spi.h"

/*****************************************************************************
 * @name       :u8 SPI_WriteByte(SPI_TypeDef* SPIx,u8 Byte)
 * @date       :2018-08-09 
 * @function   :Write a byte of data using STM32's hardware SPI
 * @parameters :SPIx: SPI type,x for 1,2,3
                Byte:Data to be written
 * @retvalue   :Data received by the bus
******************************************************************************/
u8 SPI_WriteByte(SPI_TypeDef* SPIx,u8 Byte)
{
	while((SPIx->SR&SPI_I2S_FLAG_TXE)==RESET);		
	SPIx->DR=Byte;	 	
	while((SPIx->SR&SPI_I2S_FLAG_RXNE)==RESET);
	return SPIx->DR;          	     	
} 

/*****************************************************************************
 * @name       :void SPI2_Init(void)	
 * @date       :2018-08-09 
 * @function   :Initialize the STM32 hardware SPI2
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void SPI2_Init(void)	
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	//SPI2_Setup_GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin 	=  GPIO_Pin_14;    
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	//SPI2_SetUp
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 ,ENABLE);
	   
	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode 					= SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize 			= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 					= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 					= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 					= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit 			= SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	//Enable SPI2
	SPI_Cmd(SPI2, ENABLE);   
}

/*****************************************************************************
 * @name       :void SPI1_Init(void)	
 * @date       :2018-08-09 
 * @function   :Initialize the STM32 hardware SPI1
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void SPI1_Init(void)	
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	//SPI1_Setup_GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
  //SPI1_SCK -> PA5 , SPI1_MISO -> PA6 , SPI1_MOSI ->	PA7	
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	//SD_CS -> PB7 	
  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
	GPIO_SetBits(GPIOB,GPIO_Pin_7); // Card Disable  
	
	
	//SPI1_SetUp
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	   
	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode 					= SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize 			= SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL 					= SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA 					= SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS 					= SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit 			= SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	//Enable SPI1
	SPI_Cmd(SPI1, ENABLE);   
}
