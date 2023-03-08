/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_entry.c
 * @author  MCD Application Team
 * @brief   Entry point of the Application
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "main.h"
#include "app_entry.h"
#include "app_ble.h"

#include "ble.h"
#include "ble_clock.h"
#include "ble_common.h"
#include "ble_const.h"

#include "tl.h"
#include "stm32_seq.h"
#include "shci_tl.h"
#include "stm32_lpm.h"
#include "app_debug.h"



#include <errno.h>
#include "hal_common.h"
#include "serial_if.h"
#include "serial_ctrl.h"
#include "light.h"
#include "light_lc.h"
#include "vendor.h"

#include "appli_config_client.h"
#include "appli_generic_client.h"
#include "appli_light_client.h"



#include "models_if.h"



#include "appli_mesh.h"
#include "appli_nvm.h"
#include "pal_nvm.h"
#include "lp_timer.h"
#include "mesh_cfg.h"


#include "PWM_config.h"
#include "PWM_handlers.h"


#include "serial_ctrl.h"


//#include "light_client.h"
//#include "light.h"


/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
I2C_HandleTypeDef hi2c1;

/* The HUB MAC Address */

const MODEL_OpcodeTableParam_t *Light_OpcodeTable;
const MODEL_OpcodeTableParam_t *Generic_OpcodeTable;
const MODEL_OpcodeTableParam_t *LightLC_OpcodeTable;
const MODEL_OpcodeTableParam_t *Sensor_OpcodeTable;
MOBLEUINT16 Light_OpcodeTableLength;
MOBLEUINT16 Generic_OpcodeTableLength;
MOBLEUINT16 LightLC_OpcodeTableLength;
MOBLEUINT16 Sensor_OpcodeTableLength;
extern MOBLEUINT16 Vendor_Opcodes_Table[] ;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
#ifdef SAVE_MODEL_STATE_POWER_FAILURE_DETECTION    
extern MOBLEUINT8 PowerOnOff_flag;
#endif
#ifdef ENABLE_OCCUPANCY_SENSOR           
extern MOBLEUINT8 Occupancy_Flag;
#endif
extern const void *mobleNvmBase; 
extern const void *appNvmBase;
extern const void *prvsnr_data;
#if (LOW_POWER_FEATURE == 1)
extern __IO uint32_t uwTick;
extern HAL_TickFreqTypeDef uwTickFreq;
#if ( CFG_LPM_SUPPORTED == 1)
static uint32_t BleMesh_sleepTime;
#endif
extern volatile uint8_t BleProcessInit;
#endif

/* USER CODE BEGIN PTD */
/*Functions added to integrate on sequencer declarations*/
#if(ALBANET_DEBUG_ACTIVITIES == 1)
static void Extract_x29(void);
#endif



static void Send_x28(void);
static void BLELinker(void);
static void Albanet_I2C_Buffer(void);


#if(ALBANET_DEBUG_SENSORS_ACTIONS == 1)
static void Sensor_Monitoring(void);
#endif


/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define POOL_SIZE (CFG_TLBLE_EVT_QUEUE_LENGTH*4U*DIVC(( sizeof(TL_PacketHeader_t) + TL_BLE_EVENT_FRAME_SIZE ), 4U))

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t EvtPool[POOL_SIZE];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t SystemCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t SystemSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t	BleSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private functions prototypes-----------------------------------------------*/
static void SystemPower_Config( void );
static void appe_Tl_Init( void );
static void APPE_SysStatusNot( SHCI_TL_CmdStatus_t status );




extern MOBLEUINT8 bdaddr[];



static void APPE_SysUserEvtRx( void * pPayload );
#if (CFG_HW_LPUART1_ENABLED == 1)
extern void MX_LPUART1_UART_Init(void);
#endif
#if (CFG_HW_USART1_ENABLED == 1)
extern void MX_USART1_UART_Init(void);
#endif

/* USER CODE BEGIN PFP */
static void Led_Init( void );
static void Button_Init( void );
/* USER CODE END PFP */

uint8_t Mesh_Stop_Mode;

char albanet_instruction[5];

/* Functions Definition ------------------------------------------------------*/
void APPE_Init( void )
{
  MOBLEUINT32 last_user_flash_address = ((READ_BIT(FLASH->SFR, FLASH_SFR_SFSA) >> FLASH_SFR_SFSA_Pos) << 12) + FLASH_BASE;
  
  SystemPower_Config(); /**< Configure the system Power Mode */
  
  HW_TS_Init(hw_ts_InitMode_Full, &hrtc); /**< Initialize the TimerServer */

/* USER CODE BEGIN APPE_Init_1 */
  APPD_Init( );

  /**
   * The Standby mode should not be entered before the initialization is over
   * The default state of the Low Power Manager is to allow the Standby Mode so an request is needed here
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);

  /*Initialization of LED embedded on Board*/
  Led_Init();
  /*Initialization embedded on Board*/
  Button_Init();
  /*Setup PWM embedded on system, considering PWM1*/
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  //PWM_Init();
  /*Activate the PWM considering last value º*/
  //Modify_PWM(COOL_LED, 0);


  
  mobleNvmBase = (const void *)(last_user_flash_address - NVM_SIZE);
  appNvmBase   = (const void *)(last_user_flash_address - NVM_SIZE - APP_NVM_SIZE);
  prvsnr_data  = (const void *)(last_user_flash_address - NVM_SIZE - APP_NVM_SIZE - PRVN_NVM_PAGE_SIZE);    
  
#if (LOW_POWER_FEATURE == 1)
  /**
   * Initialize the lp timer to be used when the systick is stopped in low power mode
   */
  LpTimerInit();
#endif
  
/* USER CODE END APPE_Init_1 */
  appe_Tl_Init();	/*  Initialize all transport layers */

  /**
   * From now, the application is waiting for the ready event ( VS_HCI_C2_Ready )
   * received on the system channel before starting the Stack
   * This system event is received with APPE_SysUserEvtRx()
   */
/* USER CODE BEGIN APPE_Init_2 */

/* USER CODE END APPE_Init_2 */
  return;
}
/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/**
 * @brief  Configure the system for power optimization
 *
 * @note  This API configures the system to be ready for low power mode
 *
 * @param  None
 * @retval None
 */
static void SystemPower_Config(void)
{

  /**
   * Select HSI as system clock source after Wake Up from Stop mode
   */
  LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);

  /* Initialize low power manager */
  UTIL_LPM_Init();
  /* Initialize the CPU2 reset value before starting CPU2 with C2BOOT */
  LL_C2_PWR_SetPowerMode(LL_PWR_MODE_SHUTDOWN);

#if (CFG_USB_INTERFACE_ENABLE != 0)
  /**
   *  Enable USB power
   */
  HAL_PWREx_EnableVddUSB();
#endif

  return;
}

static void appe_Tl_Init( void )
{
  TL_MM_Config_t tl_mm_config;
  SHCI_TL_HciInitConf_t SHci_Tl_Init_Conf;
  /**< Reference table initialization */
  TL_Init();

  /**< System channel initialization */
  UTIL_SEQ_RegTask( 1<< CFG_TASK_SYSTEM_HCI_ASYNCH_EVT_ID, UTIL_SEQ_RFU, shci_user_evt_proc );
  SHci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&SystemCmdBuffer;
  SHci_Tl_Init_Conf.StatusNotCallBack = APPE_SysStatusNot;
  shci_init(APPE_SysUserEvtRx, (void*) &SHci_Tl_Init_Conf);

  /*Activities allocated for Albanet System*/

  /*Register Initialization for different services*/
#if(ALBANET_DEBUG_ACTIVITIES == 1)
   /*Functions to integrate I2C and BLE communications*/
   //UTIL_SEQ_RegTask( 1 << CFG_TASK_I2CBUS_TX_REQ_ID, UTIL_SEQ_RFU, Extract_x29);
#endif
   /*Function to integrate the I2C receiving messages from CLOUD*/
   //UTIL_SEQ_RegTask( 1 << CFG_TASK_I2CBUS_RX_REQ_ID, UTIL_SEQ_RFU, Send_x28);

   /*This is the function to integrate message in the BLE builder*/
   //UTIL_SEQ_RegTask( 1 << CFG_TASK_BLE_LINK_REQ_ID, UTIL_SEQ_RFU, BLELinker);

   /*This is the function to integrate new ESP32 Control*/
   UTIL_SEQ_RegTask( 1 << CFG_TASK_BUS_TESTER_REQ_ID, UTIL_SEQ_RFU, Albanet_I2C_Buffer);


#if(ALBANET_DEBUG_SENSORS_ACTIONS == 1)
   UTIL_SEQ_RegTask( 1 << CFG_TASK_I2C_SENSORS_REQ_ID, UTIL_SEQ_RFU, Sensor_Monitoring);
#endif



  /**< Memory Manager channel initialization */
  tl_mm_config.p_BleSpareEvtBuffer = BleSpareEvtBuffer;
  tl_mm_config.p_SystemSpareEvtBuffer = SystemSpareEvtBuffer;
  tl_mm_config.p_AsynchEvtPool = EvtPool;
  tl_mm_config.AsynchEvtPoolSize = POOL_SIZE;
  TL_MM_Init( &tl_mm_config );

  TL_Enable();

  return;
}



static void APPE_SysStatusNot( SHCI_TL_CmdStatus_t status )
{
  UNUSED(status);
  return;
}

/**
 * The type of the payload for a system user event is tSHCI_UserEvtRxParam
 * When the system event is both :
 *    - a ready event (subevtcode = SHCI_SUB_EVT_CODE_READY)
 *    - reported by the FUS (sysevt_ready_rsp == RSS_FW_RUNNING)
 * The buffer shall not be released
 * ( eg ((tSHCI_UserEvtRxParam*)pPayload)->status shall be set to SHCI_TL_UserEventFlow_Disable )
 * When the status is not filled, the buffer is released by default
 */
static void APPE_SysUserEvtRx( void * pPayload )
{
  UNUSED(pPayload);
  /* Traces channel initialization */
  /* Enable debug on CPU2 */
  APPD_EnableCPU2( );

  APP_BLE_Init( );
  UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */
static void Led_Init( void )
{
#if (CFG_LED_SUPPORTED == 1)
  /**
   * Leds Initialization
   */

  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  BSP_LED_On(LED_BLUE);
  BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_RED);

#endif

    return;
}

static void Button_Init( void )
{
#if (CFG_BUTTON_SUPPORTED == 1)
  /**
   * Button Initialization
   */

  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW3, BUTTON_MODE_EXTI);

#endif

  return;
}
/* USER CODE END FD_LOCAL_FUNCTIONS */

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

void UTIL_SEQ_Idle( void )
{
#if ( CFG_LPM_SUPPORTED == 1)
#if (LOW_POWER_FEATURE == 1)
  if(BleProcessInit != 0)
  {
    BleMesh_sleepTime = (uint32_t)BLEMesh_GetSleepDuration();

    if (BleMesh_sleepTime > 0)
    {
      LpTimerStart(BleMesh_sleepTime);

      UTIL_LPM_EnterLowPower( );

      uwTick += (uwTickFreq*LpGetElapsedTime());
    }
		return;
  }
#else
  UTIL_LPM_EnterLowPower( );
#endif
#else
#if (LOW_POWER_FEATURE == 1)
  if(BleProcessInit != 0)
  {
	UTIL_SEQ_SetTask( 1<<CFG_TASK_MESH_REQ_ID, CFG_SCH_PRIO_0);

	TRACE_M(TF_MISC, "Only simulating start BLE !!!\r\n");

  }
#endif
#endif
  return;
}

/**
  * @brief  This function is called by the scheduler each time an event
  *         is pending.
  *
  * @param  evt_waited_bm : Event pending.
  * @retval None
  */
void UTIL_SEQ_EvtIdle( UTIL_SEQ_bm_t task_id_bm, UTIL_SEQ_bm_t evt_waited_bm )
{
#if (LOW_POWER_FEATURE == 1)
  UTIL_SEQ_Run( 0 );
#else
  UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );

  //UTIL_SEQ_SetTask( 1<< CFG_TASK_I2CBUS_RX_REQ_ID, CFG_SCH_PRIO_0);


  //UTIL_SEQ_SetTask( 1<< CFG_TASK_I2CBUS_TX_REQ_ID, CFG_SCH_PRIO_0);
  //UTIL_SEQ_SetTask( 1 << CFG_TASK_BLE_LINK_REQ_ID, CFG_SCH_PRIO_0);
  UTIL_SEQ_SetTask( 1 << CFG_TASK_BUS_TESTER_REQ_ID, CFG_SCH_PRIO_0);
  //UTIL_SEQ_SetTask( 1 << CFG_TASK_I2C_SENSORS_REQ_ID, CFG_SCH_PRIO_0);


#endif
}

void shci_notify_asynch_evt(void* pdata)
{
  UTIL_SEQ_SetTask( 1<<CFG_TASK_SYSTEM_HCI_ASYNCH_EVT_ID, CFG_SCH_PRIO_0);
  return;
}

void shci_cmd_resp_release(uint32_t flag)
{
  UTIL_SEQ_SetEvt( 1<< CFG_IDLEEVT_SYSTEM_HCI_CMD_EVT_RSP_ID );
  return;
}

void shci_cmd_resp_wait(uint32_t timeout)
{
  UTIL_SEQ_WaitEvt( 1<< CFG_IDLEEVT_SYSTEM_HCI_CMD_EVT_RSP_ID );
  return;
}

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  switch (GPIO_Pin)
  {
#ifdef SAVE_MODEL_STATE_POWER_FAILURE_DETECTION    
    case POWEROFF_PIN:
      {
        PowerOnOff_flag = 1;
      }
      break;
#endif

    case BUTTON_SW1_PIN:
      {
        UTIL_SEQ_SetTask( 1<<CFG_TASK_MESH_SW1_REQ_ID, CFG_SCH_PRIO_0);
      }
      break;

#ifdef ENABLE_OCCUPANCY_SENSOR       
    case BUTTON_SW2_PIN:
      {
        Occupancy_Flag = 1;
      }
      break;
#endif

#if ( CFG_LPM_SUPPORTED == 1)
    case BUTTON_SW3_PIN:
      {
        if(Mesh_Stop_Mode == 0)
        {
          Mesh_Stop_Mode = 1;
          /**
           * Do allow stop mode in the application
           */
          UTIL_LPM_SetStopMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_ENABLE);
          BSP_LED_Off(LED_GREEN);
        }
        else
        {
          Mesh_Stop_Mode = 0;
          /**
          * Do not allow stop mode in the application
          */
          UTIL_LPM_SetStopMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);
          BSP_LED_On(LED_GREEN);
        }
      }
      break;
#endif
      
  default:
      break;

  }
  return;
}

/*Actions to Integrate Frames from ESP32 that contains instructions*/
static void Albanet_I2C_Buffer(void){

#if(USB_BOARD_DEBUG_LEVEL_1 == 1)
    TRACE_M(TF_MISC, "Executing ESP32 Frame Receiver\r\n");
#endif


    uint8_t albanet_frames[21] = {};

	/*Now receiving frames from ESP32 and review Integrity*/
	if (HAL_I2C_Master_Receive(&hi2c1, 0x23<<1, albanet_frames, sizeof(albanet_frames),10) != HAL_OK){
 	    TRACE_M(TF_MISC, "FAILED READ ESP32 DEVICE W/I2C PROCESS\r\n");
 	    albanet_frames [21] = 0;
	}else{
		Clock_Wait(5);
 	    TRACE_M(TF_MISC, "------------ Processing Data from Cloud -----------\r\n");

  	    	if( albanet_frames[2] == 0xC0 && albanet_frames[3] ==0xE1\
				&& albanet_frames[4] == 0x26 && albanet_frames[5] == 0x93\
				&& albanet_frames[6] == 0x24 && albanet_frames[7] == 0xD9){



				/* --------------- OPTION FOR DIMMING EFFECT --------------- */
			    if(albanet_frames[14] == 0x17){
			    	TRACE_M(TF_MISC, "------------ DIMMING LIGHT ACTION -----------\r\n");
			    	TRACE_M(TF_MISC, "------------ CURRENT VALUE DIMMING: %d----------\r\n",albanet_frames[16]);
			    	sprintf(albanet_instruction, "%.02x", albanet_frames[16]);
			    	TRACE_M(TF_MISC, "--Frame Created: alba C001 8202 %s 01 --\r\n",albanet_instruction);
			    	if((albanet_frames[16]>10) && (albanet_frames[16]<=20)){
			    		SerialCtrl_Process("alba C001 8202 06 01",20);
			    	}else if((albanet_frames[16]>20) && (albanet_frames[16]<=30)){
			    		SerialCtrl_Process("alba C001 8202 07 01",20);
			    	}else if((albanet_frames[16]>30) && (albanet_frames[16]<=40)){
			    		SerialCtrl_Process("alba C001 8202 08 01",20);
			    	}else if((albanet_frames[16]>40) && (albanet_frames[16]<=50)){
			    		SerialCtrl_Process("alba C001 8202 09 01",20);
			    	}else if((albanet_frames[16]>50) && (albanet_frames[16]<=60)){
			    		SerialCtrl_Process("alba C001 8202 0A 01",20);
			    	}else if((albanet_frames[16]>60) && (albanet_frames[16]<=70)){
			    		SerialCtrl_Process("alba C001 8202 0B 01",20);
			    	}else if((albanet_frames[16]>70) && (albanet_frames[16]<=80)){
			    		SerialCtrl_Process("alba C001 8202 0C 01",20);
			    	}else if((albanet_frames[16]>80) && (albanet_frames[16]<=90)){
			    		SerialCtrl_Process("alba C001 8202 0D 01",20);
			    	}else if((albanet_frames[16]>90) && (albanet_frames[16]<=100)){
			    		SerialCtrl_Process("alba C001 8202 0E 01",20);
			    	}
				}


			    /*----------------- OPTION FOR PIR SENSOR -------------------*/
				if(albanet_frames[14] == 0x13){
					TRACE_M(TF_MISC, "***** PIR SENSOR ACTION *****\r\n");
					if(albanet_frames[15] == 0x11){
						/*Enable PIR Sensor*/
						TRACE_M(TF_MISC, "***** ENABLE PIR SENSOR *****\r\n");
						SerialCtrl_Process("alba C001 8202 02 01", 20);
					}else if(albanet_frames[15] == 0x10){
						/*Disable PIR Sensor*/
						TRACE_M(TF_MISC, "***** DISABLE PIR SENSOR *****\r\n");
						SerialCtrl_Process("alba C001 8202 03 01", 20);
					}
				}


				/*----------------- OPTIONS FOR LIGHT SENSOR ----------------*/
				if(albanet_frames[14] == 0x14){
					TRACE_M(TF_MISC, "------------ LIGHT SENSOR ACTION -----------\r\n");
						if(albanet_frames[15] == 0x11){
							/*Enable Light Sensor*/
							TRACE_M(TF_MISC, "***** ENABLE LIGHT SENSOR *****\r\n");
							SerialCtrl_Process("alba C001 8202 04 01", 20);
						}else if (albanet_frames[15] == 0x10){
							/*Disable Light Sensor*/
							TRACE_M(TF_MISC, "***** DISABLE LIGHT SENSOR *****\r\n");
							SerialCtrl_Process("alba C001 8202 05 01", 20);
						}
				}

				if(albanet_frames[14] == 0x18){
					if(albanet_frames[15] == 0x10){
						TRACE_M(TF_MISC, "------------ LIGHT 001 OFF -----------\r\n");
						SerialCtrl_Process("alba C001 8202 00 01",20);
					}
					else if(albanet_frames[15] == 0x11){
						TRACE_M(TF_MISC, "------------ LIGHT LIGHT 001 ON -----------\r\n");
						SerialCtrl_Process("alba C001 8202 01 01", 20);
					}
				}



  	    	}
	}
	return;
}

/*This is the function to send x28 messages from cloud*/
static void Send_x28(void){

	TRACE_M(TF_MISC, "INSERTING DATA FROM SERIAL INTERFACE !!!\r\n");

	/*
	 *  (1) APPLI_GENERIC_ON_OFF_SET (8201)
	 *  (2) LIGHT_LIGHTNESS_STATUS (824B)
	 *  (3) LIGHT_LIGHTNESSLINEAR_STATUS (824F)
	 *  (4) GENERIC_LEVEL_STATUS (8205)
	 *
	 */


	//Appli_LightClient_Lightness_Set();


	SerialCtrl_Process("alba C001 8202 01 01",20);
	//SerialCtrl_Process("00182020101",20);

	TRACE_M(TF_MISC, "Turning ON BOT !!!\r\n");

	Clock_Wait(5000);


	SerialCtrl_Process("alba C001 8202 00 01",20);

	TRACE_M(TF_MISC, "Turning OFF BOT !!!\r\n");

	Clock_Wait(5000);

	return;
}

/*This is the function to send messages over BLE*/
static void BLELinker(void){

	//BSP_LED_Toggle(LED_BLUE);

	TRACE_M(TF_MISC, "ONLY RUNNING AN ADITIONAL TASK TO INTEGRATE SEQUENCER !!!\r\n");



	return;
}

/*--------- Function to read sensors and debug actions on Screen ------------------*/
static void Sensor_Monitoring(void){

	TRACE_M(TF_MISC, "Executing Sensor Measurement\r\n");

	/*---------- Buffer to integrate data from sensors--------------------*/
	uint8_t Sensor_Buffer[16] = {};

	Clock_Wait(1000);


	//BSP_LED_Toggle(LED_BLACK);
	//BSP_LED_Toggle(LED_BLUE);


	/*------------LIGHT SENSOR------------------*/

	/*Reading Data, considering address*/
//	if (HAL_I2C_Master_Receive(&hi2c1, 0x10<<1, Sensor_Buffer, sizeof(Sensor_Buffer),10) != HAL_OK){
//		BSP_LED_Toggle(LED_RED);
//		BSP_LED_Toggle(LED_BLACK);
//		BSP_LED_Toggle(LED_BLUE);
// 	    TRACE_M(TF_MISC, "ISN'T POSSIBLE READ THE LIGHT SENSOR...\r\n");
// 	    Sensor_Buffer[16] = 0;
//	}else{
//		Clock_Wait(5);
//		BSP_LED_Toggle(LED_GREEN);
// 	    TRACE_M(TF_MISC, "LIGHT SENSOR READ READ IT SUCESSFULL...\r\n");
//	}
	return;
}



/* USER CODE END FD_WRAP_FUNCTIONS */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
