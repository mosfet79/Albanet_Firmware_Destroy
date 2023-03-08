/**
******************************************************************************
* @file    appli_light_client.c
* @author  BLE Mesh Team
* @brief   Application interface for Generic Mesh Models 
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 20120 STMicroelectronics</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "hal_common.h"
#include "types.h"
#include "appli_generic.h"
#include "appli_light.h"
#include "common.h"
#include "mesh_cfg_usr.h"
#include "appli_nvm.h"
#include "appli_mesh.h"
#include "generic_client.h"
#include "appli_light_client.h"
#include "light_client.h"

#include "app_common.h"




/** @addtogroup ST_BLE_Mesh
*  @{
*/

/** @addtogroup Application_Mesh_Models
*  @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern MOBLEUINT8 Tid_Client;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @brief  Appli_Light_Lightness_Set: This function is callback for Application
*          when Lightness message is called
* @param  void
* @retval MOBLE_RESULT
*/ 
MOBLE_RESULT Appli_LightClient_Lightness_Set(void) 
{ 
  MOBLE_ADDRESS elementAddr = 0; 
  MOBLEUINT8 pLightnessParam[3];

  
#if(ALBANET_DEBUG_DIMMER == 1)
  TRACE_M(TF_GENERIC,"Executing Function Lightness Set \r\n");
#endif


  Appli_IntensityControlPublishing(pLightnessParam);
#if(ALBANET_DEBUG_DIMMER == 1)

  TRACE_M(TF_GENERIC,"Publishing Intensity\r\n");
  TRACE_M(TF_GENERIC, "Element Address: %.02x \r\n", elementAddr);
  TRACE_M(TF_GENERIC, "Brightness to Execute: %.02X \r\n", pLightnessParam);

#endif


  TRACE_M(TF_GENERIC,"Executing the Lightness Value on Element %.02x \r\n", elementAddr);

  LightClient_Lightness_Set_Unack(elementAddr,
                          (_Light_LightnessParam*) pLightnessParam, 
                          sizeof(pLightnessParam) ); 

  TRACE_M(TF_GENERIC,"Specific Client to Set Lightness\r\n");

  return MOBLE_RESULT_SUCCESS;
} 


/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

