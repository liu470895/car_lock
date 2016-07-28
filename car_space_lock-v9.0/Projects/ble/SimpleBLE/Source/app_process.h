#ifndef APP_PROCESS_H
#define APP_PROCESS_H

#include "comdef.h"
#if !defined (longtime)
  #define  longtime 
#endif

#define admin_flash_start_addr  0x80
#define user_flash_start_addr   0x86
#define ssid_flash_start_addr   0x92
#define dead_date_flash_start_addr 0x98
#define carlock_id_flash_start_addr 0xa0
#define use_lock_times_start_addr 0xa6

#define idle                    0
#define SystemStabletimeN       200
#define MoterOnDelayN           30              //300ms 电机延时启动
#define MoterOver90DelayN       50              //3s   电机超程延迟启动

/* carlock mode define  */
#define carlockmode_uncar       0x00
#define carlockmode_cared       0x01
#define carlockmode_carederr    0x02

/* carlock motor  threshold value define  */
#define MotorDownADValueN             80        //  Value=i*0.45R/3300*1023   i=716mA  i*0.1395
#define MotorUpADValueN               100       //  Value=i*0.45R/3300*1023   i=716mA  i*0.1395
#define UncarADerrtimesRage1          1
#define UncarADerrtimesRage2          2
#define UncarADerrtimesRage3          3
#define CaredADerrtimesN              6

/* carlock state downed  times define  */
#define CarLockDownedTimeN            1500+1      //10ms*x  15s
#define CarLockDownedTimeArrive       1           // It is time to uping to 90隆茫 for carlock
#define CarLockupedTimeArrive         1           // It is time to uping to 90隆茫 for carlock
#define CarGoChecktimeArrive          1           // It is time to uping to 90隆茫 for carlock

#if defined (longtime)
  #define CarLockCaredN                 30           // x minite
  #define CarLockCarUpN                 30          // 10ms*x   xS
  #define CarLockCaredTimeN           CarLockCaredN*100           // It is time to waiting parking for carlock
  #define CarLockCarUpTimeN           CarLockCarUpN*100           // It is time to waiting parking for carlock
  #define CarGoChecktimeN             10*100                      // It is time to waiting parking for carlock

  #define Hmc5883CaredUpdatetimeN    (uint32)(2*10)                         //10ms*x   500ms   有车多久检查地磁
  #define XYZDifferSumN                 500          //  Hmc5883 Check Differ for car 
  #define XYZDifferVSumN                300          //  Hmc5883 Check Differ for car 
  #define XYZDifferVOnlyN               -200          //  Hmc5883 Check Differ for car 

#else
  #define CarLockCaredN                 30                          // x s    30s
  #define CarLockCarUpN                 1                           //10ms*x  1s
  #define CarLockCaredTimeN           CarLockCaredN*100           // It is time to waiting parking for carlock
  #define CarLockCarUpTimeN           CarLockCarUpN*100           // It is time to waiting parking for carlock

  #define Hmc5883CaredUpdatetimeN    (uint32)(1*10)                         //10ms*x   5s   有车多久检查地磁
  #define XYZDifferSumN                 200          //  Hmc5883 Check Differ for car 
  #define XYZDifferVSumN                200          //  Hmc5883 Check Differ for car 

#endif

/* beep runing  time define  */
#define BeepRunTimeN                  2                   //x S 
#define BeepRunTimeRage                BeepRunTimeN*100    //10ms*100*x  

/*  Hmc5883update time define  */
#define Hmc5883UpdateTimex         (uint8)60                              //x hour 
#define Hmc5883UpdateTimeN1        (uint32)(Hmc5883UpdateTimex*100*60)    //10ms*100*x  
#define Hmc5883UpdateTimeN2        (uint32)(1000)                         //10ms*1000  

/*  Hmc5883CheckCarState  define  */
#define CheckCarStateEmpty         0x00          //  Car Empty 
#define CheckCarStateCome          0x01          //  Car Come 
#define CheckCarStateGo            0x02          //  Car Go 
#define CheckCarStateDone          0x03          //  Car Go 


/*  CaredModeWait car stop  Time     define  */
#define CaredModeWaitStopCarTime          12000+1   //120s    
#define CaredModeWaitStopCarTimeArrive    1         //1ms    

/*  RfRead Mode Time after the sent date define  */
#define  RfSentRXmodeStartTrigN        3                  //x*10ms   
#define  RfRXmodeDelaytimeN            2                  //x*10ms   
#define  RfSentDelaytimeN              1                  //x*10ms  

/*  Use carlock Times  define  */
#define  UseTimesN         60000                   
/*  Beep on Time  define  */
#define  BeepontimeN       50                           //x*100ms  

/*  Use carlock Times  define  */
#define  report_countnumN         50//100                   //x*100ms       


#ifndef  test
  //#define    test    
#endif

// 判断密码是否正确
#define  upwdright   0x01
#define  upwderr     0x02
#define  mpwdright   0x03
#define  mpwderr     0x04
#define  lockidright   0x01
#define  lockiderr     0x02

// 队列长度和判断阈值
#define  QueueLength                      11
#define  QueueAverageValueOverflowN       30
#define  QueuePeakValueOverflowN          100
#define  Queue_yOverflowN                 100

//#define  QueuePeakValueOverflowN          100
//#define  Queue_yOverflowN                 200

#define REPORT_IDLE                    0
#define REPORT_START                   1
#define REPORT_ACK_TIMEOUT             2

typedef struct Char6
{
	uint8 pwd[6];
	uint8 id;
	uint8 cmd;
	uint8 res;
	uint8 num[6];
}char6_t;

typedef struct 
{
	uint8 pwd[6];
	uint8 id;
	uint8 cmd;
	uint8 res;
	uint8 num[6];
	uint8 lockid[6];
}hub_user_data_t;

typedef enum 
{
	REPORT_STATE_INFO 			    = 0,
	REPORT_MODIFY_ADMIN_PWD 	  = 1,
	REPORT_MODIFY_USER_PWD 		  = 2,
	REPORT_MODIFY_PWD_DEAD_DATE = 3,
	REPORT_MODIFY_SSID 			    = 4,
  REPORT_MODIFY_LOCK_ID 		  = 5,
	REPORT_LOOP_STATE 			    = 6
}Report_Type_t;


void show_bat_by_leds(void);
unsigned char get_pwd_state(void);
unsigned char executive_cmd(char6_t char6);
unsigned char get_adc_battery(void);
unsigned char get_battery_level(void);
unsigned char is_lock(void);
unsigned char get_lock_state(void);
unsigned char get_notify_data(void);
void cargo_init(void);
void carcome_init(void);

uint8 Read_Dio0(void);
uint8 rf470m_irq(void);
void get_use_times(uint8 use_times[2]);
void get_lock_status(uint8 *lock_status);
void get_lock_info(uint8 lock_info[9]);
void lock_init(void);
void ready_report_hub_para(uint8 report_type);
void report_hub(void);
uint8 hub_process(hub_user_data_t hub);
void set_use_times(void);

extern   void beep_on_ntime(void);
extern   void Rf470_setfre1(void);
extern   void Rf470M_sendDelayTimeRXmodeDelayTime(void);
extern   void Hmc5883InitGetVector(void);
extern   void UpdateHmc5883vector(void);
extern   void systemcontrol(void);
extern   void  carlockseatctrol(void);

#endif//APP_PROCESS_H