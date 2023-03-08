#ifndef ALBANET_CONFIGURATION_H_
#define ALBANET_CONFIGURATION_H_


void function_sensor(uint8_t state, uint8_t sensor);
void albanet_WEB_dimmer(uint8_t power);
void PIR_Counter(uint16_t cycle);

/*Functions to Manipulate PIR Sensor*/
void PIR_Enable(void);
void PIR_Disable(void);

void LIGHT_Enable(void);
void LIGHT_Disable(void);




#define DEMO_VERSION	1







#endif
