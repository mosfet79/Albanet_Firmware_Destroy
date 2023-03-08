/*
 * Library to Control Light Sensor.
 *
 *  Created on: October, 2021
 *      Author: Ernesto Alonso Molina I.
 *
 * The sensor VEML6040 is exclusive for light sensor
 * that includes and I2C, considering four outputs:
 *
 *      1) White
 *      2) Blue
 *      3) Green
 *      4) Red
 *
 */

#include "app_common.h"
#include "main.h"
#include "app_entry.h"


#include "ble_clock.h"

#include <math.h>
#include <stdio.h>
#include "light_sensor.h"


/* Identity for I2C Bus considering Light Sensor */
extern I2C_HandleTypeDef hi2c1;

/* Buffer Declaration to integrate values */
uint8_t buf[12] = {};
extern uint8_t lastConfiguration;

uint8_t VEML6040_Setup(void)
{

	/* ------------------------------------------------------------------
	 * THE VEML6040 requires (00h) registers to configure (First Byte)
	 *         Default Value = MSB(0,0,0,1,0,1,0,0)LSB = x14
	 * Bit 0 (Enable Color Sensor) = 0
	 * Bit 1 (Auto or Manual Force Mode) = 0
	 * Bit 2 (Trigger effect to light measurement) = 1
	 * Bit 3 (Not Used) = 0
	 * Bit 4 (IT0, Integration Time Setting) = 80 mSec
	 * Bit 5 (IT1, Integration Time Setting)
	 * Bit 6 (IT2, Integration Time Setting)
	 * Bit 7 (Bit Not used)
	 *
	 *  Pending to check the number of bytes integrate on system.
	 * -----------------------------------------------------------------*/


	/*Entity, DevAddress, MemAddress*/
	if (HAL_I2C_Mem_Write(&hi2c1, VEML6040_I2C_ADDRESS, 0x0000, 2, 0x0014, 2, 10) != HAL_OK )
	{
		BSP_LED_Toggle(LED_RED);
	}

	BSP_LED_Toggle(LED_GREEN);

	return Error_Code_No_Error;
}


/*Read Data from Sensor*/
uint8_t VEML6040_readSensor(uint8_t CommandCode)
{
	//Buffer for Sensor Data
	uint8_t ReadData[2];
	uint8_t light_sensor_data[2];

	//Color Data, its output converted on system.
	uint16_t Color_Data;

	/*Start Reading  Sensor, considering 2 Bytes*/
	if(HAL_I2C_Mem_Read(&hi2c1, VEML6040_I2C_ADDRESS, CommandCode, 1, light_sensor_data, sizeof(light_sensor_data), 10) != HAL_OK)
	{
		BSP_LED_Toggle(LED_RED);
		return Error_Code_Error;
	}else{
		ReadData[0] = light_sensor_data[0];
		ReadData[1] = light_sensor_data[1];

		Color_Data = (ReadData[1] << 8) + ReadData[0];

		/*Process to convert value*/
		float ambient_light_lux;

		/*Sensor Value*/

		ambient_light_lux = Color_Data * 0x10;

		//*ReturnVal = Color_Data;
		return ambient_light_lux;
		//return Error_Code_No_Error;
	}
}


void LIGHT_Disable(){
	option01 = 0;
	option02 = 0;
	option03 = 0;
	option04 = 0;
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
	BSP_LED_On(LED_BLACK);
}

void LIGHT_Enable(){

	/*Conditions to change variables*/
	option01 = 0;
	option02 = 0;
	option03 = 1;
	option04 = 0;


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
