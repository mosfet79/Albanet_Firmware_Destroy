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
#include "tl.h"
#include "stm32_seq.h"
#include "shci_tl.h"
#include "stm32_lpm.h"
#include "app_debug.h"

#include "appli_mesh.h"
#include "appli_nvm.h"
#include "pal_nvm.h"
#include "lp_timer.h"
#include "mesh_cfg.h"
#include "light.h"
#include "PWM_handlers.h"

/*Library Integrate PIR Sensor Actions*/
#include "albanet_configuration.h"

#include "common.h"

static void Led_Init(void);
static void Button_Init(void);

extern I2C_HandleTypeDef hi2c1;



uint16_t light_sensor_data[1] = {};

uint16_t counter = 0;
uint8_t PirAction = 0;
uint8_t PIR_OUT = 0;
uint8_t LOCK_PIR = 0x00;
uint8_t clock_cycle = 0;

extern MOBLEUINT8 RestoreFlag;
extern MOBLEUINT16 IntensityValue;
extern MOBLEUINT8 IntensityFlag;
extern MOBLEUINT8 PowerOnOff_flag;


uint8_t option01;
uint8_t option02;
uint8_t option03;
uint8_t option04;
uint8_t option05;
uint8_t option06;


uint8_t VERSION_SELECTION = 0x01;


uint8_t PIR_STATE;





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

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define POOL_SIZE (CFG_TLBLE_EVT_QUEUE_LENGTH*4U*DIVC(( sizeof(TL_PacketHeader_t) + TL_BLE_EVENT_FRAME_SIZE ), 4U))




/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t EvtPool[POOL_SIZE];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t SystemCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t SystemSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t	BleSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255];



/* Private functions prototypes-----------------------------------------------*/
static void SystemPower_Config( void );
static void appe_Tl_Init( void );
static void APPE_SysStatusNot( SHCI_TL_CmdStatus_t status );

static void APPE_SysUserEvtRx( void * pPayload );


static void pir_sensor_management(void);
static void pir_irq_control(void);
static void controller_cycle(void);

#if (CFG_HW_LPUART1_ENABLED == 1)
extern void MX_LPUART1_UART_Init(void);
#endif
#if (CFG_HW_USART1_ENABLED == 1)
extern void MX_USART1_UART_Init(void);
#endif

/* USER CODE BEGIN PFP */
static void Led_Init( void );
static void Button_Init( void );

TIM_HandleTypeDef htim16;


/* USER CODE END PFP */

uint8_t Mesh_Stop_Mode;

/* Functions Definition ------------------------------------------------------*/
void APPE_Init( void )
{
  MOBLEUINT32 last_user_flash_address = ((READ_BIT(FLASH->SFR, FLASH_SFR_SFSA) >> FLASH_SFR_SFSA_Pos) << 12) + FLASH_BASE;
  
  SystemPower_Config(); /**< Configure the system Power Mode */
  
  HW_TS_Init(hw_ts_InitMode_Full, &hrtc); /**< Initialize the TimerServer */

/* USER CODE BEGIN APPE_Init_1 */
  APPD_Init( );

  PIR_STATE = 0;
  /**
   * The Standby mode should not be entered before the initialization is over
   * The default state of the Low Power Manager is to allow the Standby Mode so an request is needed here
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP, UTIL_LPM_DISABLE);


  Led_Init();

  /*Previously the Main Light started using BLACK LED*/
  Button_Init();

  /*Create the Setup to execute Start Up of Light Sensor*/
  VEML6040_Setup();


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


  return;
}

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

  UTIL_SEQ_RegTask( 1<< CFG_LED_TESTER_ID, UTIL_SEQ_RFU, pir_sensor_management);

  UTIL_SEQ_RegTask( 1<< CFG_PIR_SENSOR_REQ_ID, UTIL_SEQ_RFU, pir_irq_control);

  UTIL_SEQ_RegTask( 1<< CFG_LIGHT_SENSOR_REQ_ID, UTIL_SEQ_RFU, controller_cycle);

  SHci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&SystemCmdBuffer;
  SHci_Tl_Init_Conf.StatusNotCallBack = APPE_SysStatusNot;
  shci_init(APPE_SysUserEvtRx, (void*) &SHci_Tl_Init_Conf);

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
 *    - reported by the FUS (sysevt_ready_rsp == FUS_FW_RUNNING)
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

  BSP_LED_Init(LED_BLACK);
  BSP_LED_Init(LED_ORANGE);

  /*Startup the MAIN LIGHT*/
  BSP_LED_On(LED_BLACK);
  BSP_LED_On(LED_ORANGE);

  /*Correct Assignation*/
  BSP_LED_On(LED_RED);

  /*Correct Assignation*/
  BSP_LED_On(LED_BLUE);

  /*Correct Assignation*/
  BSP_LED_On(LED_GREEN);

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
#endif

#if (CFG_PIR_SENSOR == 1)
  /*Initialization of PIR SENSOR*/
  BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
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
    UTIL_SEQ_SetTask( 1<<CFG_TASK_MESH_REQ_ID, CFG_SCH_PRIO_0);

  }
#else
  UTIL_LPM_EnterLowPower( );
#endif
#else
#if (LOW_POWER_FEATURE == 1)
  if(BleProcessInit != 0)
  {
    UTIL_SEQ_SetTask( 1<<CFG_TASK_MESH_REQ_ID, CFG_SCH_PRIO_0);
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

  UTIL_SEQ_SetTask( 1<<CFG_LED_TESTER_ID, CFG_SCH_PRIO_0);


#if (LOW_POWER_FEATURE == 1)
  UTIL_SEQ_Run( 0 );
#else
  UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );

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
        //UTIL_SEQ_SetTask( 1<<CFG_PIR_SENSOR_REQ_ID, CFG_SCH_PRIO_0);

      }
      break;
    case BUTTON_SW2_PIN:
      {
        UTIL_SEQ_SetTask( 1<<CFG_PIR_SENSOR_REQ_ID, CFG_SCH_PRIO_0);
        //UTIL_SEQ_SetTask( 1<<CFG_TASK_MESH_SW1_REQ_ID, CFG_SCH_PRIO_0);

      }
      break;

  default:
      break;

  }
  return;

}

uint8_t flag_light_sensor = 0;

/* USER CODE END FD_WRAP_FUNCTIONS */
void pir_sensor_management(void)
{

	/*OPTIONS TO INTEGRATE SENSORS*/
	/*
	 * 000 --> PIR_DISABLE, LIGHT_DISABLE, CALENDAR_DISABLE, MANUAL_ENABLE; (STATE 1)
	 * 001 --> PIR_DISABLE, LIGHT_DISABLE, CALENDAR_ENABLE, MANUAL_DISABLE; (STATE 2)
	 * 010 --> PIR_DISABLE, LIGHT_ENABLE, CALENDAR_DISABLE, MANUAL_DISABLE; (STATE 3)
	 * 011 --> PIR_ENABLE, LIGHT_DISABLE, CALENDAR_DISABLE, MANUAL_DISABLE; (STATE 4)
	 *
	 */

	 /*Always Light Sensor Reading values */
	//float light_sensor = VEML6040_readSensor(0x20);
	/*Now Reading Color Green is on Register 0x09*/
	float light_sensor = VEML6040_readSensor(0x09);

    TRACE_M(TF_LIGHT,"Routine checking Service\r\n");



	if((option01==0)&&(option02==0)&&(option03==0)&&(option04==0))
	{
		/*Predominates Manual Execution*/
		BSP_LED_Toggle(LED_RED);
		LOCK_PIR = 0x00;
	}else if((option01==0)&&(option02==1)&&(option03==0)&&(option04==0)){
		/*Predominates  Calendar State*/
		BSP_LED_Toggle(LED_BLUE);
		LOCK_PIR = 0x00;
	}else if((option01==0)&&(option02==0)&&(option03==1)&&(option04==0)){
		LOCK_PIR = 0x00;
		/*Condition to evaluate the next state*/
		if(flag_light_sensor == 0){

			/*Predominates Light Sensor LUX setpoint, Hardcoded value on system*/
			/*The Range Value Could Be 0 to 200*/

			if((light_sensor >= 0) && (light_sensor < 20)){
				Modify_PWM(SINGLE_LED, 1000);
			}else if((light_sensor >= 20) && (light_sensor < 40)){
				Modify_PWM(SINGLE_LED, 2000);
			}else if((light_sensor >= 40) && (light_sensor < 60)){
				Modify_PWM(SINGLE_LED, 4000);
			}else if((light_sensor >= 60) && (light_sensor < 80)){
				Modify_PWM(SINGLE_LED, 6000);
			}else if((light_sensor >= 80) && (light_sensor < 100)){
				Modify_PWM(SINGLE_LED, 8000);
			}else if((light_sensor >= 100) && (light_sensor < 120)){
				Modify_PWM(SINGLE_LED, 10000);
			}else if((light_sensor >= 120) && (light_sensor < 140)){
				Modify_PWM(SINGLE_LED, 12000);
			}else if((light_sensor >= 140) && (light_sensor < 160)){
				Modify_PWM(SINGLE_LED, 14000);
			}else if((light_sensor >= 180) && (light_sensor < 200)){
				Modify_PWM(SINGLE_LED, 16000);
			}else{
				//Modify_PWM(SINGLE_LED, 18000);
				BSP_LED_Toggle(LED_BLUE);
			}
		}


	}else if((option01==0)&&(option02==0)&&(option03==0)&&(option04==1)){
			/*Predominates PIR Sensor*/
			if(LOCK_PIR == 0x00){
				BSP_LED_Off(LED_BLACK);
				LOCK_PIR = 0x01;
			}

	}else{
		BSP_LED_Off(LED_BLACK);
	}
}

void function_sensor(uint8_t state, uint8_t sensor){
	/*Options to Execute PIR Sensor*/
	if(sensor == 0x13){
		if(state == 0x00){
			/*Disable PIR Sensor*/
			PIR_Disable();
		}else if(state == 0x01){
			/*Enable PIR Sensor*/
			PIR_Enable();
		}
	/*Options to Execute LIGHT Sensor*/
	}else if(sensor == 0x14){
		if(state == 0x00){
			/*Disable LIGHT sensor*/
			LIGHT_Disable();
		}else if(state == 0x01){
			/*Enable LIGHT sensor*/
			LIGHT_Enable();
		}
	}
}

void pwm_change_effect()
{
	for(int z=0; z<= 10; z++){
		BSP_LED_Toggle(LED_RED);
		BSP_LED_Toggle(LED_BLUE);
		Clock_Wait(100);
	}
}

void albanet_WEB_dimmer(uint8_t power){
	/*Power Execution Dynamically*/
	if(power == 10){
		Modify_PWM(SINGLE_LED, 100);
		pwm_change_effect();
	}else if(power == 20){
		Modify_PWM(SINGLE_LED, 1000);
		pwm_change_effect();
	}else if(power == 30){
		Modify_PWM(SINGLE_LED, 2000);
		pwm_change_effect();
	}else if(power == 40){
		Modify_PWM(SINGLE_LED, 3000);
		pwm_change_effect();
	}else if(power == 50){
		Modify_PWM(SINGLE_LED, 7000);
		pwm_change_effect();
	}else if(power == 60){
		Modify_PWM(SINGLE_LED, 10000);
		pwm_change_effect();
	}else if(power == 70){
		Modify_PWM(SINGLE_LED, 12000);
		pwm_change_effect();
	}else if(power == 80){
		Modify_PWM(SINGLE_LED, 14000);
		pwm_change_effect();
	}else if(power == 90){
		Modify_PWM(SINGLE_LED, 18000);
		pwm_change_effect();
	}

}

void controller_cycle()
{
	if(LOCK_PIR == 0x01 && clock_cycle == 0){
		counter = 20;
		clock_cycle = 1;
		BSP_LED_On(LED_BLACK);


		Clock_Wait(5000);
//		for(int x=0; x<counter; x++)
//		{
//			Clock_Wait(500);
//			BSP_LED_Toggle(LED_BLACK);
//
//		}
		BSP_LED_Off(LED_BLACK);
	}
	clock_cycle=0;
}

/*This function is called whiled the IRQ is activated*/
void pir_irq_control(void){

	//PIR_OUT = BSP_PB_GetState(BUTTON_SW2);
	BSP_LED_Toggle(LED_BLUE);
	if((option01==0)&&(option02==0)&&(option03==0)&&(option04==1)){
		/*This action call to controller Cycle*/
		//UTIL_SEQ_SetTask( 1<<CFG_LIGHT_SENSOR_REQ_ID, CFG_SCH_PRIO_0);
		BSP_LED_On(LED_BLACK);
		Clock_Wait(5000);
		BSP_LED_Off(LED_BLACK);


	}

}
