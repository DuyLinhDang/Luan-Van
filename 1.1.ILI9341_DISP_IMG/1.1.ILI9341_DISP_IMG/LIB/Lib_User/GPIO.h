
/*
  *****************************************************************************
  * @file			mcu-gpio.h                                                              *
  * @author			Thai Salem                                               
  * @date			00/00/0000                                                         
  ******************************************************************************
*/

#ifndef GPIO_H
#define GPIO_H	

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

//extern GPIO_InitTypeDef gpio_typedef;
/*******************************************************************************************************/
#define HIGH              0x1
#define LOW               0x0
/*******************************************************************************************************/
#define OUTPUT 						GPIO_Mode_Out_PP
#define INPUT  						GPIO_Mode_IN_FLOATING
#define INPUT_PULLUP 			GPIO_Mode_IPU
#define INPUT_PULLDOWN 		GPIO_Mode_IPD
#define OUTPUT_OD					GPIO_Mode_Out_OD
#define AF_OD             GPIO_Mode_AF_OD
#define AF_PP             GPIO_Mode_AF_PP
#define AN_INPUT          GPIO_Mode_AIN
/*******************************************************************************************************/
#define PA0           (0u)
#define PA1           (1u)
#define PA2           (2u)
#define PA3           (3u)
#define PA4           (4u)
#define PA5           (5u)
#define PA6           (6u)
#define PA7           (7u)
#define PA8           (8u)
#define PA9           (9u)
#define PA10          (10u)
#define PA11          (11u)
#define PA12          (12u)
#define PA13          (13u)
#define PA14          (14u)
#define PA15          (15u)

#define PB0           (16u)
#define PB1           (17u)
#define PB2           (18u)
#define PB3           (19u)
#define PB4           (20u)
#define PB5           (21u)
#define PB6           (22u)
#define PB7           (23u)
#define PB8           (24u)
#define PB9           (25u)
#define PB10          (26u)
#define PB11          (27u)
#define PB12          (28u)
#define PB13          (29u)
#define PB14          (30u)
#define PB15          (31u)

#define PC0           (32u)
#define PC1           (33u)
#define PC2           (34u)
#define PC3           (35u)
#define PC4           (36u)
#define PC5           (37u)
#define PC6           (38u)
#define PC7           (39u)
#define PC8           (40u)
#define PC9           (41u)
#define PC10          (42u)
#define PC11          (43u)
#define PC12          (44u)
#define PC13          (45u)
#define PC14          (46u)
#define PC15          (47u)
/*******************************************************************************************************/
/*GPIO configure*/
void pinMode(uint16_t PINx,uint16_t MODE);
/*----------------------------------------------------------------------------------------*/

/*write the value for Pinx of GPIOx*/
void digitalWrite(uint16_t PINx, uint16_t value);
/*----------------------------------------------------------------------------------------*/

/*Read the value from Pinx of GPIOx */
int digitalRead(uint16_t PINx);
/*----------------------------------------------------------------------------------------*/

/*GPIO configure*/
void GPIOx_Set(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin_x, GPIOMode_TypeDef GPIO_Mode_x, GPIOSpeed_TypeDef GPIO_Speed_x);
/*----------------------------------------------------------------------------------------*/
void GPIOx_ConfiLed(void);
/*----------------------------------------------------------------------------------------*/
void GPIOx_ConfiBut(void);
/*----------------------------------------------------------------------------------------*/
void GPIOx_Confi7Seg(void);
/*----------------------------------------------------------------------------------------*/
void GPIOx_ConfisMaTrix(void);

#ifdef __cplusplus
}
#endif

#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* THAI E&C *****END OF FILE****/



