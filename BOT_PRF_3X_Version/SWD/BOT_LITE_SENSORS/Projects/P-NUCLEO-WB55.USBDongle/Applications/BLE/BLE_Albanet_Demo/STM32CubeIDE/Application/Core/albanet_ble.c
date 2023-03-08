
//#include "../../.settings/albanet_ble.h"

#include "main.h"
//#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "stm32_seq.h"


typedef struct{
	uint8_t 	Device_Led_Selection;
	uint8_t		Led1;
}P2P_LedCharValue_t;

typedef struct{
	uint8_t		Device_Button_Selection;
	uint8_t		ButtonStatus;
}P2P_ButtonCharValue_t;

typedef struct
{
	uint8_t					Notification_Status;
	P2P_LedCharValue_t		LedControl;
	P2P_ButtonCharValue_t	ButtonControl;
	uint16_t				ConnectionHandle;

}P2P_Server_App_Context_t;

PLACE_IN_SECTION("BLE_APP_CONTEXT") static P2P_Server_App_Context_t P2P_Server_App_Context;

static void P2PS_Send_Notification(void);
static void P2PS_APP_LED_BUTTON_context_Init(void);

void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
	switch(pNotification->P2P_Evt_Opcode)
	{
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
	case P2PS_STM_BOOT_REQUEST_EVT:
		APP_DBG_MSG("-- P2P APPLICATION SERVER : BOOT REQUESTED\n");
		APP_DBG_MSG("\n\r");
		*(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
		NVIC_SystemReset();
		break;
#endif

	case P2PS_STM__NOTIFY_ENABLED_EVT:
		/*User Code begin P2PS_STM__NOTIFY ENABLED_EVT*/
		P2P_Server_App_Context.Notification_Status = 1;
		APP_DBG_MSG("-- P2P APPLICATION SERVER:NOTIFICATION ENABLED");
		APP_DBG_MSG("\n\r");
		break;
		/*User code end P2PS*/

	case P2PS_STM_NOTIFY_DISABLED_EVT:
		P2P_Server_App_Context.Notification_Status = 0;
		APP_DBG_MSG("-- P2P APPLICATION SERVER: NOTIFICATION DISABLED");
		APP_DBG_MSG("\n\r");
		break;

	case P2PS_STM_WRITE_EVT:
		if(pNotification->DataTransfered.pPayload[0] == 0x00){
			if(pNotification->DataTransfered.pPayload[1] == 0x01)
			{
				BSP_LED_On(LED_BLUE);
				P2P_Server_App_Context.LedControl.Led1=0x01;
			}if(pNotification->DataTransfered.pPayload[1] == 0x00)
			{
				BSP_LED_Off(LED_BLUE);
				P2P_Server_App_Context.LedControl.Led1=0x00;
			}
		}

#if(P2P_SERVER1 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x01){ /* end device 1 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif

#if(P2P_SERVER2 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x02){ /* end device 2 selected */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
           APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER3 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x03){ /* end device 3 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 3 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 3 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER4 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x04){ /* end device 4 selected */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
           APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 2 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER5 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x05){ /* end device 5 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 5 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 5 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
#if(P2P_SERVER6 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x06){ /* end device 6 selected */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
           APP_DBG_MSG("-- P2P APPLICATION SERVER 6 : LED1 ON\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 6 : LED1 OFF\n");
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif

      break;

	default:

	  break;
	}

	return;

}

//void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
//{
///* USER CODE BEGIN P2PS_APP_Notification_1 */
//
///* USER CODE END P2PS_APP_Notification_1 */
//  switch(pNotification->P2P_Evt_Opcode)
//  {
///* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */
//
///* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
//  case PEER_CONN_HANDLE_EVT :
///* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
//
///* USER CODE END PEER_CONN_HANDLE_EVT */
//    break;
//
//    case PEER_DISCON_HANDLE_EVT :
///* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
//       P2PS_APP_LED_BUTTON_context_Init();
///* USER CODE END PEER_DISCON_HANDLE_EVT */
//    break;
//
//    default:
///* USER CODE BEGIN P2PS_APP_Notification_default */
//
///* USER CODE END P2PS_APP_Notification_default */
//      break;
//  }
///* USER CODE BEGIN P2PS_APP_Notification_2 */
//
///* USER CODE END P2PS_APP_Notification_2 */
//  return;
//}


void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, P2PS_Send_Notification );

  /**
   * Initialize LedButton Service
   */
  P2P_Server_App_Context.Notification_Status=0;
  P2PS_APP_LED_BUTTON_context_Init();
/* USER CODE END P2PS_APP_Init */
  return;
}

void P2PS_APP_SW1_Button_Action(void)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}

void P2PS_APP_LED_BUTTON_context_Init(void){

  BSP_LED_Off(LED_BLUE);

  #if(P2P_SERVER1 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x01; /* Device1 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x01;  /* Device1 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER2 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x02;        /* Device2 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x02;  /* Device2 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER3 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x03; 		 /* Device3 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x03; /* Device3 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER4 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x04; /* Device4 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x04; /* Device4 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
 #if(P2P_SERVER5 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x05; /* Device5 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x05; /* Device5 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
#if(P2P_SERVER6 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x06; /* device6 */
  P2P_Server_App_Context.LedControl.Led1=0x00; /* led OFF */
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x06; /* Device6 */
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
}

void P2PS_Send_Notification(void)
{

  if(P2P_Server_App_Context.ButtonControl.ButtonStatus == 0x00){
    P2P_Server_App_Context.ButtonControl.ButtonStatus=0x01;
  } else {
    P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
  }

   if(P2P_Server_App_Context.Notification_Status){
    APP_DBG_MSG("-- P2P APPLICATION SERVER :INFORM CLIENT BUTTON 1 PUSHED  --\n ");
    APP_DBG_MSG(" \n\r");
   // P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&P2P_Server_App_Context.ButtonControl);
   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
   }

  return;
}




