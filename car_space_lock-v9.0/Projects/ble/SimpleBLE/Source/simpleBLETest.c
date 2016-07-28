/**************************************************************************************************
  Filename:       SimpleBLETest.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "npi.h"
#include "stdio.h"
#include "app_process.h"

#include "Osal_snv.h"

#include "gatt.h"
#include "att.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
#include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLETest.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "hal_mcu.h"
#include "hal_hmc5883.h"
#include "app_motor.h"
#include "app_process.h"
#include "app_hmc5883.h"
#include "app_buzzer.h"
#include "app_uart.h"
#include "app_tx_power_control.h"
#include "LSD_RF_APPrf.h"
#include "LSD_RF_SX1276.h"
#include "app_batter.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
//#define SBP_PERIODIC_EVT_PERIOD                   200        //x ms
#define SBP_PERIODIC_EVT_PERIOD                   100           //x ms

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          480

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     320

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          400//1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         4

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

uint16  PeriodtimeCnt1 = 0;
uint16  PeriodtimeCnt2 = 0;

uint16 SysTick1 = 0;
uint16 SysTick2 = 0;
uint8 WakeAddr[8]={5,6,7,8,9,10,11,7};
uint8 report_count = 0;
uint16 report_countnum=0;
uint16 report_timeout_count = 0;

extern  XYZ_DataN_t    Nxyz;
extern uint8 start_sleep_timerout_flag;
extern uint8 start_timerout_flag;
extern uint8 control_buzzer_flag;
extern uint8 last_report_get_ack_flag;
extern uint8 save_report_tpye;
//the control register for  hmc5883  
uint8 HMC5883_CTROL_REG[3]={0x78,0x20,0x00};    

uint8 ssid[6]={'S', 'i', 'm', 'p', 'l', 'e'};//默认ssid

uint8 start_report_flag = 0;
uint8 immediatly_report_flag = 0;
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */


static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
uint8 scanRspData[] =
{
  // complete name
  0x07,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',   
  'i',   
  'm',   
  'p',   
  'l',   
  'e',   

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0      // 0dBm
};


// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // in this peripheral
  0x1A,   // length of this data 26byte
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,  
  /*Apple Pre-Amble*/
  0x4C,
  0x00,
  0x02,
  0x15,
  /*Device UUID (16 Bytes)*/
  0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48,0xD2, 0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0,
  /*Major Value (2 Bytes)*/
  0x00, 0x01,
  
  /*Minor Value (2 Bytes)*/
  0x00,0x02,
  
  /*Measured Power*/
  0xCD
};

static int8 gMP = 0xCF;

// GAP GATT Attributes
uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple";
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTED,
};
// Application state
 uint8 simpleBLEState = BLE_STATE_IDLE;

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};


// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  uint8 read_tmp[6] = {0};
  uint8 ret8;
   
  simpleBLEPeripheral_TaskID = task_id;
  
  if(osal_snv_read(0xfe,1,&gMP)!=NV_OPER_FAILED){
	  advertData[29]=gMP;
  }
  
  // 如果该段内存未曾写入过数据， 直接读，会返回 NV_OPER_FAILED ,
  // 我们利用这个特点作为第一次烧录后的运行， 从而设置参数的出厂设置
  ret8 = osal_snv_read(admin_flash_start_addr , sizeof(read_tmp), read_tmp);//admin pwd
  if(NV_OPER_FAILED == ret8)
  {
    osal_memset(read_tmp, '0', sizeof(read_tmp));
    ret8 = osal_snv_write(admin_flash_start_addr, sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
    
    ret8 = osal_snv_read(admin_flash_start_addr , sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
  }
  
  ret8 = osal_snv_read(user_flash_start_addr , sizeof(read_tmp), read_tmp);//user pwd
  if(NV_OPER_FAILED == ret8)
  {
    osal_memset(read_tmp, '1', sizeof(read_tmp));
    ret8 = osal_snv_write(user_flash_start_addr, sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
    
    ret8 = osal_snv_read(user_flash_start_addr , sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
  }
  
  ret8 = osal_snv_read(ssid_flash_start_addr , sizeof(read_tmp), read_tmp);//ssid 
  if(NV_OPER_FAILED == ret8)
  {
    osal_memcpy(read_tmp,  ssid,  6);
    ret8 = osal_snv_write(ssid_flash_start_addr, sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
   
    ret8 = osal_snv_read(ssid_flash_start_addr , sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
  }
  else//update ssid from flash
  {
    ret8 = osal_snv_read(ssid_flash_start_addr , sizeof(read_tmp), read_tmp);
    if(ret8 != NV_OPER_FAILED)
    {
      osal_memcpy(scanRspData + 2, read_tmp, 6);
      osal_memcpy(attDeviceName, read_tmp, 6);
      GAP_UpdateAdvertisingData( simpleBLEPeripheral_TaskID, FALSE, sizeof( scanRspData ), scanRspData );
      GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
    }
  }
  
  ret8 = osal_snv_read(dead_date_flash_start_addr , sizeof(read_tmp), read_tmp);//dead date
  if(NV_OPER_FAILED == ret8)
  {
    osal_memset(read_tmp, '0', sizeof(read_tmp));
    ret8 = osal_snv_write(dead_date_flash_start_addr, sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
    
    ret8 = osal_snv_read(dead_date_flash_start_addr , sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
  }
  
  ret8 = osal_snv_read(carlock_id_flash_start_addr , sizeof(read_tmp), read_tmp);//lock id
  if(NV_OPER_FAILED == ret8)
  {
    osal_memset(read_tmp, '0', sizeof(read_tmp));
    ret8 = osal_snv_write(carlock_id_flash_start_addr, sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
    
    ret8 = osal_snv_read(carlock_id_flash_start_addr , sizeof(read_tmp), read_tmp);
    if(ret8 == NV_OPER_FAILED)
      return;
  }
  
  uint8 use_lock_times[2] = {0};
  ret8 = osal_snv_read(use_lock_times_start_addr , sizeof(use_lock_times), use_lock_times);//use times
  if(NV_OPER_FAILED == ret8)
  {
    osal_memset(use_lock_times, 0, sizeof(use_lock_times));
    ret8 = osal_snv_write(use_lock_times_start_addr, sizeof(use_lock_times), use_lock_times);
    if(ret8 == NV_OPER_FAILED)
      return;
    
    ret8 = osal_snv_read(use_lock_times_start_addr , sizeof(use_lock_times), use_lock_times);
    if(ret8 == NV_OPER_FAILED)
      return;
  }
  
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters      
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
	
  }

  // Setup the GAP Bond Manager   //设定绑定的管理员
  {
    uint32 passkey = 0; // passkey "000000"     //配对密码
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    //uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE; //需要将 pairMode 设置为 GAPBOND_PAIRING_MODE_INITIATE，
                                                    //才能在连接的时候产生配对请求 
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;      //只有显示密码的能力
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );//把passkey的地址里面的值拷贝到 gapBond_Passcode变量
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );//同理把pairMode地址里面的值拷贝到gapBond_PairingMode
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );//同理
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );//同理
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding ); //同理
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1[SIMPLEPROFILE_CHAR1_LEN] = {0};
    uint8 charValue2[SIMPLEPROFILE_CHAR2_LEN] = {0};
    uint8 charValue3[SIMPLEPROFILE_CHAR3_LEN] = {0};
    uint8 charValue4[SIMPLEPROFILE_CHAR4_LEN] = {0};
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = {0};
    uint8 charValue6[SIMPLEPROFILE_CHAR6_LEN] = {0};
 
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, charValue6 );
  }

  
// Register callback with SimpleGATTprofile
VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

// Enable clock divide on halt
// This reduces active current while radio is active and CC254x MCU
// is halted
HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )
  
//  P2INP |=  0x03;    // Configure Port 0 ,1，as pull-down 20K
//  P0INP &= ~0x03;    // Configure Port 0 ,1，as pull-down 20K
  P0INP |=  0xC0;    // Configure Port 6 ,7，as 3-state
  P0SEL &= ~0x3C;    // Configure Port 0 as GPIO
  P0DIR |=  0x3C;    // Port 0 pins P0.0 , P0.1 , P0.6 , P0.7 
                     // as  input (AD0, AD1 ,X1，X2) 
                     // all others (P0.4-P0.7) as output
	P1SEL |=  0xE0;    // P1_5, P1_6, and P1_7 are peripherals
	P1SEL &= ~0x10;    // P1_4 is GPIO (SSN)
	P1DIR |=  0x13;    // SPI  
                     // all others (P1.0-P1.5) as output
  P2SEL = 0X00;         // Configure Port 2 as GPIO
  P2DIR = 0x01;      // All port 1 pins (P2.0) as output

  P0 &= ~0x24;       // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 &= ~0x03;       // All pins on port 1 to low
  P2 |=  0x07;       // All pins on port 2 to low 

  //init batter check
  batter_control(1);

  //init Hmc5883
  Hmc5883_Init(HMC5883_ADDR);  
  Hmc5883_WriteReg(0x00, HMC5883_CTROL_REG, 3); 

  Hmc5883InitGetVector();  //init the vector sum for power up
  
  //show battery 
  show_bat_by_leds();
  //rf470m  init
  LSD_RF_Init(frefunction1);

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
  
  // 启动看门狗， 定是1000ms， 1000ms内需要喂狗， 否则就重启了
 // HAL_SYSTEM_WATCH_DOG_1000MS(); 
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
  extern uint8 ssid_modify_flag;
  extern uint8 new_ssid[];
  extern uint8 feedback_pwd_state; 
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )//这里面的函数只执行了一遍
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );//注册回调函数

    // Start Bond Manager
    //VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs ); //注册回调函数

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )//第二次调用该函数才进入里面执行周期性的任务
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      //周期性触发定时器，使simpleBLEPeripheral_TaskID对应的任务周期性地执行
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );                                                                  
    }
    
    // Perform periodic application task
    performPeriodicTask();          //执行周期性的任务
    
    return (events ^ SBP_PERIODIC_EVT);
  }
  
  //pwd state
  if(feedback_pwd_state != 0)
  {
    char2_notify(get_pwd_state());
    feedback_pwd_state = 0;
  }
  
  carlockseatctrol();//car lock control up or down
  
  //Rf470M_sendDelayTimeRXmodeDelayTime();        //  sent data and RXmode                    
  
  if(ssid_modify_flag == 1)                     //Update ssid in ssid changed
  {  
    ssid_modify_flag = 0;
    osal_memcpy(scanRspData + 2, new_ssid, 6);
    osal_memcpy(attDeviceName, new_ssid, 6);

    GAP_UpdateAdvertisingData( simpleBLEPeripheral_TaskID, FALSE, sizeof( scanRspData ), scanRspData );
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
  }
  
  if(immediatly_report_flag == 1)
  {
    immediatly_report_flag = 0;
    report_hub();
  }

  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )
  default:
    // do nothing
    break;
  }
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        simpleBLEState = BLE_STATE_IDLE;
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        simpleBLEState = BLE_STATE_CONNECTED;
      }
      break;

    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

void u16_to_str(uint16 n, uint8 buf[5])
{
  int i = 0;
  uint8 tmp[5] = {0};
  
  if( n == 0 )
  {
    n = 1;
  }
  
  while(n)
  {  
    tmp[i++] = n % 10 + '0';
    n /= 10; 
  }
  
  for(int j=0; j<5; j++)
    buf[j] = tmp[5 - j - 1];
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{ 
  static uint16 hub_buzzer_count = 0;
  extern uint16 buzzer_on_time;
  
//  WD_KICK();   //  喂狗， 避免软件重启  
  
  if(control_buzzer_flag == 1)//蜂鸣器响的时间控制
  {
      hub_buzzer_count++;
      if(hub_buzzer_count > buzzer_on_time)
      {
        hub_buzzer_count = 0;
        control_buzzer_flag = 0;
        buzzer_control(0);
      }
  }
  
  if(last_report_get_ack_flag == REPORT_START)
  {
    report_count++;
    if(report_count == report_countnum)//50*100 = 5s
    {
      report_count = 0;               //重新开始计算时间
      report_countnum = report_countnumN;
      report_timeout_count++;
      if(report_timeout_count < 120) // 5*12*10s = 10分钟
      {
        immediatly_report_flag = 1;
      }
      else
      {
        report_timeout_count = 0;
        last_report_get_ack_flag = REPORT_ACK_TIMEOUT;
        Rf470_setfre1();               //超时设置为频点一，转为接收模式
      }
        
    }
  }
  else
  {
    report_count = 0;
  }
 
  PeriodtimeCnt1++;
  if(PeriodtimeCnt1 == UpdateNotifityTimeN)
  {
    uint8 data = 0;
    
    PeriodtimeCnt1 = 0;
    data = get_notify_data();    //battery and lock state report
    char4_notify(data);
  }
  
  PeriodtimeCnt2++;
  //LED blinks
  if(PeriodtimeCnt2<LedLightTimeN)
  {
     HalLedSet (HAL_LED_3, HAL_LED_MODE_OFF);  
  }
  else if(PeriodtimeCnt2<LedUnlightTimeN)
  {
    HalLedSet (HAL_LED_3, HAL_LED_MODE_OFF);  
  }
  else
  {
    PeriodtimeCnt2=0;
  } 

//  SysTick1++;
//  if(SysTick1 >= 200)   //睡眠2s时间到
//  {
//    SysTick1 =0;
//    LSD_RF_WOR_Execute(1);   //启动CAD采样一次
//  }
//  
//  SysTick2++;
//  if(SysTick2 >= 600)   //睡眠6s时间到
//  {
//    SysTick2 =0;
//    OFF_Timerout();                      //关闭超时定时器
//    LSD_RF_WORInit();                   //WOR初始化
//    LSD_RF_WOR_Execute(0);              //启动执行WOR
//  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{ 
  uint8 Value[SIMPLEPROFILE_CHAR6_LEN] = {0};
  char6_t char6;
  
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      break;
    
    case SIMPLEPROFILE_CHAR2:
      break;
      
    case SIMPLEPROFILE_CHAR3:
      break;
      
    case SIMPLEPROFILE_CHAR5:
      break;
      
    case SIMPLEPROFILE_CHAR6:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, Value );
      osal_memcpy(char6.pwd, Value, 6);
      char6.id = Value[6];
      char6.cmd = Value[7];
      char6.res = Value[8];
      osal_memcpy(char6.num, Value + 9, 6);
      executive_cmd(char6);
      break; 
      
    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)



uint16 connHandle = 0;
extern gattCharCfg_t simpleProfileChar1Config[];
extern gattCharCfg_t simpleProfileChar2Config[];
extern gattCharCfg_t simpleProfileChar3Config[];
extern gattCharCfg_t simpleProfileChar4Config[];
extern gattAttribute_t simpleProfileAttrTbl[];

bStatus_t char1_notify(uint8 *buf, uint8 len)
{
  attHandleValueNoti_t Report;
  
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);
  uint16 value = GATTServApp_ReadCharCfg( connHandle, simpleProfileChar1Config);
  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    Report.handle = simpleProfileAttrTbl[2].handle;
    Report.len = len;                
    osal_memcpy(Report.value, buf, len);
    // Send the notification
    return GATT_Notification( connHandle, &Report, FALSE );
  }
  
  return bleIncorrectMode;
}


bStatus_t char2_notify(uint8 state)
{
  attHandleValueNoti_t Report;
  
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);
  uint16 value = GATTServApp_ReadCharCfg( connHandle, simpleProfileChar2Config);
  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    Report.handle = simpleProfileAttrTbl[6].handle;
    Report.len = 1;                
    osal_memcpy(Report.value, &state, 1);
    // Send the notification
    return GATT_Notification( connHandle, &Report, FALSE );
  }
  
  return bleIncorrectMode;
}

bStatus_t char3_notify(uint8 *buf, uint8 len)
{
  attHandleValueNoti_t Report;
  
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);
  uint16 value = GATTServApp_ReadCharCfg( connHandle, simpleProfileChar3Config);
  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    Report.handle = simpleProfileAttrTbl[10].handle;
    Report.len = len;                
    osal_memcpy(Report.value, buf, len);
    // Send the notification
    return GATT_Notification( connHandle, &Report, FALSE );
  }
  
  return bleIncorrectMode;
}


bStatus_t char4_notify(uint8 state)
{
  attHandleValueNoti_t Report;
  
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connHandle);
  uint16 value = GATTServApp_ReadCharCfg( connHandle, simpleProfileChar4Config);
  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    Report.handle = simpleProfileAttrTbl[14].handle;
    Report.len = 1;                
    osal_memcpy(Report.value, &state, 1);
    // Send the notification
    return GATT_Notification( connHandle, &Report, FALSE );
  }
  
  return bleIncorrectMode;
}

void get_notify3_data_sent(void)
{
  uint8 buf[5] = {0};
  extern uint16  QueueMiddle;
  extern uint16  QueueAverage;

    char3_notify("\r\n", 2);
    u16_to_str(QueueMiddle, buf);
    char3_notify(buf, 5);
}

void lock_init(void)
{
  uint8 read_tmp[6] = {0};
  uint8 use_lock_times[2] = {0};
  uint8 ret8;
   
  osal_memset(read_tmp, '0', sizeof(read_tmp));
  ret8 = osal_snv_write(admin_flash_start_addr, sizeof(read_tmp), read_tmp);
  if(ret8 == NV_OPER_FAILED)
     return;

  osal_memset(read_tmp, '1', sizeof(read_tmp));
  ret8 = osal_snv_write(user_flash_start_addr, sizeof(read_tmp), read_tmp);
  if(ret8 == NV_OPER_FAILED)
     return;
    
  osal_memcpy(read_tmp,  ssid,  6);
  ret8 = osal_snv_write(ssid_flash_start_addr, sizeof(read_tmp), read_tmp);
  if(ret8 == NV_OPER_FAILED)
     return;

  osal_memcpy(scanRspData + 2, read_tmp, 6);
  osal_memcpy(attDeviceName, read_tmp, 6);
  GAP_UpdateAdvertisingData( simpleBLEPeripheral_TaskID, FALSE, sizeof( scanRspData ), scanRspData );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
   
  osal_memset(read_tmp, '0', sizeof(read_tmp));
  ret8 = osal_snv_write(dead_date_flash_start_addr, sizeof(read_tmp), read_tmp);
  if(ret8 == NV_OPER_FAILED)
    return;
    
  osal_memset(read_tmp, '0', sizeof(read_tmp));
  ret8 = osal_snv_write(carlock_id_flash_start_addr, sizeof(read_tmp), read_tmp);
  if(ret8 == NV_OPER_FAILED)
     return;

  osal_memset(use_lock_times, 0, sizeof(use_lock_times));
  ret8 = osal_snv_write(use_lock_times_start_addr, sizeof(use_lock_times), use_lock_times);
  if(ret8 == NV_OPER_FAILED)
     return; 
  
  HAL_SYSTEM_RESET();//reset
}

/*********************************************************************
*********************************************************************/
