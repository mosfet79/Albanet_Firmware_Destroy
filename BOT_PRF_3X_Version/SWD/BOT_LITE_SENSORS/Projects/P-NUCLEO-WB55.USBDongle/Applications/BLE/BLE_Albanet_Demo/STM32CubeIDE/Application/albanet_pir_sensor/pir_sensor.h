/*
 * PIR Sensor Library
 *
 *  Created on: November, 2021
 *      Author: Ernesto Alonso Molina I.
 */

#ifndef APPLICATION_ALBANET_PIR_SENSOR_PIR_SENSOR_H_
#define APPLICATION_ALBANET_PIR_SENSOR_PIR_SENSOR_H_

#define PIR_PIN		1

typedef enum
{
	PIR_INPUT = 0
}Pin_TypeDef;

typedef enum
{
	PIN_MODE_GPIO = 0,
	PIN_MODE_EXTI = 1
}PinMode_TypeDef;


#define PIR_INPUT_PIN					GPIO_PIN_2
#define PIR_INPUT_PORT					GPIOA
#define PIR_INPUT_CLK_ENABLE()			__HAL_RCC_GPIOA_CLK_ENABLE()
#define PIR_INPUT_CLK_DISABLE()			__HAL_RCC_GPIOA_CLK_DISABLE()
#define PIR_INPUT_EXT_LINE				GPIO_PIN_2
#define PIR_INPUT_EXTI_IRQn				EXTI2_IRQn


#define Error_Code_No_Error_PIR 	0x00


void BSP_PIR_Initialization(Pin_TypeDef Pin, PinMode_TypeDef Pin_Mode);
void BSP_PIR_DeInit(Pin_TypeDef Pin);
uint32_t BSP_PIR_GetState(Pin_TypeDef Pin);



#endif /* APPLICATION_ALBANET_PIR_SENSOR_PIR_SENSOR_H_ */
