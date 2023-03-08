/*
 * Enable PIR Sensor, considering a possible aggressive movement.
 *
 *  Created on: November, 2021
 *      Author: Ernesto Alonso Molina I.
 *
 * The PIR sensor has included a digital output, open collector
 * considering the PORT A Pin 2 to receive sense about the presence.
 *
 */


#include "ble_common.h"
#include "app_conf.h"
#include "hal_common.h"
#include "ble_mesh.h"
#include "appli_mesh.h"
#include "types.h"
#include "ble_hal_aci.h"
#include "ble_hci_le.h"
#include <string.h>

#include "models_if.h"
#include "mesh_cfg.h"
#include "generic.h"
#include "light.h"
#include "light_lc.h"
#include "sensors.h"
#include "common.h"
#include "serial_if.h"
#include "appli_nvm.h"
#include "pal_nvm.h"
#include "appli_config_client.h"
#include "appli_generic_client.h"
#include "appli_light_client.h"
#include "appli_sensor.h"
#include "appli_sensors_client.h"
#include "stm32_seq.h"

#include "stm32wbxx_hal_rcc.h"
#include "stm32wbxx_hal.h"
#include "app_common.h"
#include "main.h"
#include "app_entry.h"

#include "stm32wbxx_usb_dongle.h"

#include "mesh_cfg_usr.h"

#define BOUNCE_THRESHOLD                20U
#define LONG_PRESS_THRESHOLD            1000U
#define MANUAL_UNPROVISION_TIMER        3000U


#include "pir_sensor.h"

GPIO_TypeDef* PIR_PORT[PIR_PIN] = {PIR_INPUT_PORT};

const uint16_t PIR_PINS[PIR_PIN] = {PIR_INPUT_PIN};

const uint8_t PIR_IRQn[PIR_PIN] = {PIR_INPUT_EXTI_IRQn};


void BSP_PIR_Initialization(Pin_TypeDef Pin, PinMode_TypeDef PinMode)
{
	GPIO_InitTypeDef gpioinitstruct = {0};

	/*Enable the PIR PIN Clock*/
	PIR_INPUT_CLK_ENABLE();

	if(PinMode == PIN_MODE_GPIO)
	{
		gpioinitstruct.Pin	= GPIO_PIN_2;
		gpioinitstruct.Mode	= GPIO_MODE_INPUT;
		gpioinitstruct.Pull = GPIO_NOPULL;
		gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(PIR_PORT[Pin], &gpioinitstruct);

	}
	if(PinMode == PIN_MODE_EXTI)
	{
		gpioinitstruct.Pin = PIR_PINS[Pin];
		gpioinitstruct.Mode = GPIO_MODE_IT_RISING_FALLING;
		gpioinitstruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PIR_PORT[Pin], &gpioinitstruct);

		HAL_NVIC_SetPriority(EXTI2_IRQn,0x00, 0x00);
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	}


}

void BSP_PIR_DeInit(Pin_TypeDef Pin){
	GPIO_InitTypeDef gpio_init_structure;

	gpio_init_structure.Pin	= PIR_PINS[Pin];
	HAL_NVIC_DisableIRQ((IRQn_Type)(PIR_IRQn[Pin]));
	HAL_GPIO_DeInit(PIR_PORT[Pin], gpio_init_structure.Pin);
}

uint32_t BSP_PIR_GetState(Pin_TypeDef Pin)
{
	return HAL_GPIO_ReadPin(PIR_PORT[Pin], PIR_PINS[Pin]);
}

void PIR_Enable(void){
	option01 = 0;
	option02 = 0;
	option03 = 0;
	option04 = 1;
	BSP_LED_On(LED_RED);
	Clock_Wait(200);
	BSP_LED_Off(LED_RED);
	Clock_Wait(200);
	BSP_LED_On(LED_RED);
	Clock_Wait(200);
	BSP_LED_Off(LED_RED);
	Clock_Wait(200);
	BSP_LED_On(LED_RED);
	Clock_Wait(200);
}

void PIR_Disable(void){
	option01 = 0;
	option02 = 0;
	option03 = 0;
	option04 = 0;
	BSP_LED_On(LED_BLACK);
	BSP_LED_On(LED_BLUE);
	Clock_Wait(200);
	BSP_LED_Off(LED_BLUE);
	Clock_Wait(200);
	BSP_LED_On(LED_BLUE);
	Clock_Wait(200);
	BSP_LED_Off(LED_BLUE);
	Clock_Wait(200);
	BSP_LED_On(LED_BLUE);
	Clock_Wait(200);
}


