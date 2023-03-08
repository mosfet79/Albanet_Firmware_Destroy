/**
 ******************************************************************************
 * @file    app_common.h
 * @author  MCD Application Team
 * @brief   Common
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_COMMON_H
#define __APP_COMMON_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "app_conf.h"

#define ALBANET_DEBUG_STATE				1
#define ALBANET_DEBUG_DIMMER			1
#define ALBANET_DEBUG_ACTIVITIES		0


#define ALBANET_PWM_DEBUG_ACTIVITIES	1
#define USB_BOARD_DEBUG_LEVEL_1			0
#define ALBANET_DEBUG_STRATEGY			1

/*DEFINITION OF I2C BUS*/

#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) /sizeof(*(__BUFFER__)))


#define TXBUFFERSIZE    (COUNTOF(aTxBuffer_i2c) - 1)
#define RXBUFFERSIZE	TXBUFFERSIZE

static const uint8_t  LIGHT_SENSOR_ADDR	= 0x10<<1;
static const uint8_t  HTS221_ADDR_RD	= 0XBF<<1;
static const uint8_t  HTS221_ADDR_WR	= 0XBE<<1;
static const uint8_t  PMIC_ADDR			= 0x18<<1;
static const uint8_t  ESP32_ADDR		= 0x23<<1;
static const uint8_t  STM32WB55			= 0x24<<1;


/*MAC ADDRESS EXCLUSIVE FOR HUB COMMUNICATION LINK*/



/*Registers to catch data from Temperature Sensor*/
uint8_t humedity_out_lsb[1];
uint8_t humedity_out_msb[1];

/*Registers to catch data from Light Sensor*/
uint8_t light_sensor_lsb[1];
uint8_t light_sensor_msb[1];



  /* -------------------------------- *
   *  Basic definitions               *
   * -------------------------------- */

#undef NULL
#define NULL                    0

#undef FALSE
#define FALSE                   0

#undef TRUE
#define TRUE                    (!0)

  /* -------------------------------- *
   *  Critical Section definition     *
   * -------------------------------- */
#define BACKUP_PRIMASK()    uint32_t primask_bit= __get_PRIMASK()
#define DISABLE_IRQ()       __disable_irq()
#define RESTORE_PRIMASK()   __set_PRIMASK(primask_bit)

  /* -------------------------------- *
   *  Macro delimiters                *
   * -------------------------------- */

#define M_BEGIN     do {

#define M_END       } while(0)

  /* -------------------------------- *
   *  Some useful macro definitions   *
   * -------------------------------- */

#define MAX( x, y )          (((x)>(y))?(x):(y))

#define MIN( x, y )          (((x)<(y))?(x):(y))

#define MODINC( a, m )       M_BEGIN  (a)++;  if ((a)>=(m)) (a)=0;  M_END

#define MODDEC( a, m )       M_BEGIN  if ((a)==0) (a)=(m);  (a)--;  M_END

#define MODADD( a, b, m )    M_BEGIN  (a)+=(b);  if ((a)>=(m)) (a)-=(m);  M_END

#define MODSUB( a, b, m )    MODADD( a, (m)-(b), m )


#define PAUSE( t )           M_BEGIN \
                               volatile int _i; \
                               for ( _i = t; _i > 0; _i -- ); \
                             M_END

#define DIVF( x, y )         ((x)/(y))

#define DIVC( x, y )         (((x)+(y)-1)/(y))

#define DIVR( x, y )         (((x)+((y)/2))/(y))

#define SHRR( x, n )         ((((x)>>((n)-1))+1)>>1)

#define BITN( w, n )         (((w)[(n)/32] >> ((n)%32)) & 1)

#define BITNSET( w, n, b )   M_BEGIN (w)[(n)/32] |= ((U32)(b))<<((n)%32); M_END

  /* -------------------------------- *
   *  Compiler                         *
   * -------------------------------- */
#define PLACE_IN_SECTION( __x__ )  __attribute__((section (__x__)))

#ifdef WIN32
#define ALIGN(n)
#else
#define ALIGN(n)             __attribute__((aligned(n)))
#endif


#ifdef __cplusplus
}
#endif

#endif /*__APP_COMMON_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
