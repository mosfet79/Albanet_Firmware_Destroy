/*
 * Control Light Sensor, considering I2C Bus.
 *
 *  Created on: October, 2021
 *      Author: Ernesto Alonso Molina I.
 */

#ifndef APPLICATION_ALBANET_LIGHT_SENSOR_LIGHT_SENSOR_H_
#define APPLICATION_ALBANET_LIGHT_SENSOR_LIGHT_SENSOR_H_


/*Device Physical Address*/
#define VEML6040_I2C_ADDRESS 0x10 << 1


/*Registers to catch data from Light Sensor*/
extern uint8_t Light_Sensor_LSB[1];
extern uint8_t Light_Sensor_MSB[1];


/*Registers to indicate parameters about Light Sensors*/
#define VEML6040_IT_40MS		0x00
#define VEML6040_IT_80MS		0x10
#define VEML6040_IT_160MS		0x20
#define VEML6040_IT_320MS		0x30
#define VEML6040_IT_640MS		0x40
#define VEML6040_IT_1280MS		0x50


/*TRIGGER MEASUREMENT WHEN IN FORCE MODE*/
#define VEML6040_TRIG_DISABLE 0x00
#define VEML6040_TRIG_ENABLE  0x04


/*AUTO_FORCE (AUTO:SENSOR MEASURES AUTOMATICALLY)*/
#define VEML6040_AF_AUTO		0x00
#define VEML6040_AF_FORCE		0x02

/*ENABLE/DISABLE SENSOR*/
#define VEML6040_SD_ENABLE		0x00
#define VEML6040_SD_DISABLE		0x01

/*COMMAND CODES*/
#define COMMAND_CODE_CONF		0x00
#define COMMAND_CODE_RED		0x08
#define COMMAND_CODE_GREEN		0x09
#define COMMAND_CODE_BLUE		0x0A
#define COMMAND_CODE_WHITE		0x0B

/*G SENSITIVITY, GETTING AMBIENT LIGHT*/
#define VEML6040_GSENS_40MS		0.25168
#define VEML6040_GSENS_80MS		0.12584
#define VEML6040_GSENS_160MS	0.06292
#define VEML6040_GSENS_320MS	0.03146
#define VEML6040_GSENS_640MS	0.01573
#define VEML6040_GSENS_1280MS	0.007865


/*I2C ACTIONS FOR READ AND WRITE*/
#define I2C_VEML6040_WRITE		0x00
#define I2C_VEML6040_READ		0x01


/* ERROR CODES FOR SENSOR*/
#define Error_Code_No_Error 	0x00
#define Error_Code_Error		0x01


float VEML_6040_getAmbientLight();
void VEML6040_startMeasurement(void);


#endif /* APPLICATION_ALBANET_LIGHT_SENSOR_LIGHT_SENSOR_H_ */
