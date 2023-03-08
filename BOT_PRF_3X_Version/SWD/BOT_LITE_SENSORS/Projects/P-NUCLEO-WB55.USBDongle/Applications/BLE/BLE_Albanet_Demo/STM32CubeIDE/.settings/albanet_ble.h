#ifndef SETTINGS_ALBANET_BLE_H_
#define SETTINGS_ALBANET_BLE_H_

#ifdef __cpluscplus
extern "C"
#endif


typedef enum
{
	PEER_CONN_HANDLE_EVT,
	PEER_DISCON_HANDLE_EVT,

} P2PS_APP__Opcode_Notification_evt_t;


typedef struct
{
	P2PS_APP__Opcode_Notification_evt_t P2P_Evt_Opcode;
} P2PS_APP_ConnHandle_Not_evt_t;

//void P2PS_APP_Init(void);
//void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t * pNotification);
//void P2PS_APP_SW1_Button_Action(void);

#ifdef __cplusplus
}
#endif


#endif /* SETTINGS_ALBANET_BLE_H_ */
