#include "app_process.h"
#include "simpleGATTprofile.h"
#include "OSAL.h"
#include "Osal_snv.h"
#include "app_uart.h"
#include "npi.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "stdio.h"

#include "simpleBLETest.h"

#include "app_led_control.h"
#include "app_hmc5883.h"
#include "app_buzzer.h"
#include "app_batter.h"
#include "app_uart.h"
#include "app_tx_power_control.h"
#include "app_keyscan.h"
#include "app_motor.h"
#include "LSD_RF_APPrf.h"
#include "LSD_RF_SX1276.h"
#include "hal_defs.h"
#include "gapgattserver.h"
#include "math.h" 

uint8   BeepTimes                  = idle;
uint8   KeyStatus                  = idle;
uint8   MotorTarget                = carlock90degree;
uint8   Motorfuction               = Enable;
uint8   Carlockmode                = carlockmode_uncar;
uint16  MotorADValue_carlock       = idle;             //  90��   AD Value
uint8   UncarADerrtimes            = idle;
uint16   CaredADerrtimes            = idle;
uint32  CarLockDownedTime          = idle;
uint32  BeepRunTime                = idle;
uint32  Hmc5883UpdateTime          = idle;
uint32  CaredModeParkingTime       = idle;
uint8   CaredModeStartFlg          = idle;
uint8   CheckCarState              = CheckCarStateEmpty;
uint8   ToHubReportFlag            = Disable;
uint8   MoterState                 = idle;
uint32  Hmc5883CaredUpdatetime     = idle;
uint32  SystemStabletime           = SystemStabletimeN; 
uint32  MoterOnDelay               = MoterOnDelayN; 
uint32  Caredtime                  = idle; 
uint32  CarGotime                  = idle; 
uint32  CarGoChecktime             = idle; 
uint8   CheckCarChange;
uint8   CheckCarUpdateFlg;
uint32  beepontime                 = idle;;

uint8   save_report_tpye = 0;
uint8   last_report_get_ack_flag = REPORT_IDLE;
uint8   feedback_pwd_state = 0;
uint8   ssid_modify_flag = 0;
uint8   lockid_state 	= 0;
uint16  Hmc5883InitValue_x =0 ;
uint16  Hmc5883InitValue_y =0 ;
uint8   control_buzzer_flag = 0;
uint16  buzzer_on_time = 0;
uint8   lock_status_M = 0;
uint8   lock_status_N = 0;
uint8   lock_status_K = 0;
uint8   lock_status_L = 0;
uint8   lock_status_buf[15] = {0};

unsigned char buf[30]       = {0};
unsigned char pwd[6] 		    = {0};
unsigned char lockid[6] 		= {'0', '0', '0', '0', '0', '0'};

uint8 report_buf[30] = {0};
uint8 hub_buf[30] = {0};
uint8 old_admin_pwd[6]			    = {'0','0','0','0','0','0'};
uint8 new_ssid[6]               = {'0','0','0','0','0','0'};

uint16 conventional_fault_count = 0;
uint8  malignant_fault_count = 0;
uint32 magnetic_intensity = 0;
uint8  RfSentRXmodeEnFlag = Disable;     
uint16 RfSentDelaytime = 0;     

XYZ_DataN_t    Nxyz;
XYZ_DataP_t    Pxyz;
uint16  differ_x;
uint16  differ_y;
uint16  differ_z;
uint16  XYZDifferSum=0;

static uint16  Queue[QueueLength]={0};
static uint16  Queue1[QueueLength]={0};
uint16  QueueMiddle = 1;
uint16  QueueAverage = 1;


void QueueInput(uint16 xyzdiffer_vectorsum_abs ,uint16 xyz_x ,uint16 queueLength)  //���
{  
  queueLength --;
  for(int i=0; i < queueLength ; i++ )
  {
     Queue[i] = Queue[i+1] ;
     Queue1[i] = Queue1[i+1] ;
  }
  Queue[ queueLength ] = xyzdiffer_vectorsum_abs;                    //�������ֵ
  Queue1[ queueLength ] = xyz_x;                                     //�������ֵ
  UpdateHmc5883vector();  
}
uint8 QueueMiddleOverEvent(uint16 queueLength)
{
    uint16 middle = queueLength >> 1;
  
    QueueMiddle = Queue[middle];
    //get_notify3_data_sent();����ʱ��
    if( Queue[middle] >  QueuePeakValueOverflowN )
       return true;
    else
       return false;
}

uint8 QueueeMiddlMaxEvent(uint16 queueLength)
{
  uint16 middle = queueLength>>1;
  
  for(int i=0; i < queueLength ; i++ )
  {
    if( i ==  middle ) { continue; }
    if( Queue[middle] < Queue[i] )
    {
       return false;
    }
  }
  return true;
}
 uint8 QueueAverageOverEvent(uint16 queueLength)
{  
  uint32 sum =0 ;
  
  for(int i=0; i < queueLength ; i++ )
  {
    sum +=  Queue[i];
  }
  QueueAverage = sum/ queueLength;
  sum = 0;
    
  if( QueueAverage > QueueAverageValueOverflowN )
  {
     return true;
  }
  else
  {
     return false;
  }
}
uint8 Hmc5883Change_xy( void )
{
  uint16 differ_x = 0 ;
  uint16 differ_y = 0 ;

  while( Hmc5883_Get_Vector(&Nxyz) != 1);            //  ȷ�ϵش����ݶ�ȡ�ɹ�

  if( Hmc5883InitValue_x > Nxyz.x  ) 
  {
      differ_x = Hmc5883InitValue_x - Nxyz.x;
  }
  else
  {
      differ_x = Nxyz.x - Hmc5883InitValue_x ;
  }
  if( Hmc5883InitValue_y > Nxyz.y  ) 
  {
      differ_y = Hmc5883InitValue_y - Nxyz.y;
  }
  else
  {
      differ_y = Nxyz.y - Hmc5883InitValue_y ;
  }
  
  if( ( differ_x < Queue_yOverflowN ) &&  ( differ_y < Queue_yOverflowN ) )
  {
       return true;
  }
  else
  {
       return false;
  }
}

void QueueClear( uint16 queueLength )  //��������
{  
  for(int i=0; i < queueLength ; i++ )
  {
     Queue[i] = 0;
     Queue1[i] = 0;
  }
}

void Rf470M_sendDelayTimeRXmodeDelayTime(void)
{
/*  if(  RfSentRXmodeEnFlag != Enable )
  {
    return;
  }
  if( RfSentDelaytime == RfRXmodeDelaytimeN) 
  {
      LSD_RF_RXmode();                                     //set RXmode
      RfSentRXmodeEnFlag = Disable; 
      RfSentDelaytime = 0;
  }
  if( RfSentDelaytime != 0) 
  {
      RfSentDelaytime--;    
  }*/
}

void show_bat_by_leds(void)
{
  uint8 level ;
  
  buzzer_control(1);
  level= get_battery_level();

  if(level <4 )
    led_control(HAL_LED_1, 1, 90, 1500, 1);
  else if(level <8 )
    led_control(HAL_LED_1 | HAL_LED_2 , 1, 90, 1500, 1);
  else if(level <12 )
    led_control(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, 1, 90, 1500, 1);
  buzzer_control(0);
}
 
unsigned char get_pwd_state(void)
{
	lock_status_K = 2;
	lock_status_L = feedback_pwd_state;
	return ((lock_status_K << 4) & 0xf0) | (lock_status_L & 0x0f);
}

void cargo_init(void)
{
      MotorTarget=carlock90degree;
      Carlockmode = carlockmode_uncar;
      MoterOnDelay = MoterOnDelayN;
}

void carcome_init(void)
{
  if(  KeyStatus == carlock90degree ) 
  {
    MotorTarget=carlock0degree;
    Carlockmode = carlockmode_cared;
    CheckCarState = CheckCarStateEmpty;
#if defined(test)
    Caredtime = 100;         //����ͣ��ʱ��
#else 
    Caredtime = CarLockCaredTimeN;         //����ͣ��ʱ��
#endif
    MoterOnDelay = MoterOnDelayN;
    CaredModeStartFlg = Enable;  
    QueueClear(QueueLength);               //clear
    while( Hmc5883_Get_Vector(&Nxyz) != 1);            //  ȷ�ϵش����ݶ�ȡ�ɹ�
    UpdateHmc5883vector();                             //  ���µش�����
    Hmc5883InitValue_x = Nxyz.x;
    Hmc5883InitValue_y = Nxyz.y;
  }
}
unsigned char  executive_cmd(char6_t char6)
{
  unsigned char pwd_state 	= 0;
  
  osal_memset(pwd, '0', sizeof(pwd));//change to character
  
	if(char6.id == 'U')
	{ 
		if(NV_OPER_FAILED == osal_snv_read(user_flash_start_addr, 6, pwd))
		{
			return 0;
		}
	  
		if(1 == osal_memcmp(char6.pwd, pwd, sizeof(pwd)))//check pwd
		{
			 pwd_state = upwdright;//pwd is right
		}
		else
		{
			 pwd_state = upwderr;//pwd is err
		}
        
		feedback_pwd_state = pwd_state;
    
    if( feedback_pwd_state == upwderr )
      return 0;
    
    if( 1 == osal_memcmp(char6.num, "200000", 6))//user control up lock eg:888888UA0200000
    {
        cargo_init();
    }
    else if(1 == osal_memcmp(char6.num, "100000", 6))//user control down lock eg:888888UA0100000
    {
        carcome_init();
    }
    else
    {
      return 0;
    }
	}
	else if(char6.id == 'M')
	{
    
		if(NV_OPER_FAILED == osal_snv_read(admin_flash_start_addr, 6, pwd))
		{
			return 0;
		}
		
		if(1 == osal_memcmp(char6.pwd, pwd, sizeof(pwd)))//check admin pwd
		{
			pwd_state = mpwdright;//admin pwd is right
		}
		else
		{
			pwd_state = mpwderr;//admin pwd is err
		}
    
		feedback_pwd_state = pwd_state;
    
    if(  feedback_pwd_state == mpwderr  )
      return 0;
    
		switch(char6.cmd)
		{
			case 'A':
				if(1 == osal_memcmp(char6.num, "200000", 6))//admin control up lock eg:888888MA0200000
				{
           cargo_init();
				}
				else if(1 == osal_memcmp(char6.num, "100000", 6))//admin control down lock eg:888888MA0100000
				{
           carcome_init();
				}
				else if(1 == osal_memcmp(char6.num, "3", 1))//admin control lock buzzer eg:888888MA0300000
				{
          control_buzzer_flag = 1; 
          buzzer_control(1);
          buzzer_on_time = (char6.num[4]-'0')*10;
          buzzer_on_time += (char6.num[5]-'0'); 
          buzzer_on_time *= buzzer_on_time;
          //beepontime=0;
				}
				else
				{
					return 0;
				}
				break;
				
			case 'B':
				if(char6.res == '1')
				{
					if(NV_OPER_FAILED == osal_snv_write(user_flash_start_addr, 6, char6.num))//modify user password eg:888888MB1xxxxxx
					{
						return 0;
					}
          ready_report_hub_para(REPORT_MODIFY_USER_PWD);
				}
				else if(char6.res == '2')
				{
					if(NV_OPER_FAILED == osal_snv_write(dead_date_flash_start_addr, 6, char6.num))//modify user password dead date eg:888888MB2xxxxxx
					{
						return 0;
					}
          ready_report_hub_para(REPORT_MODIFY_PWD_DEAD_DATE);
				}
				else if(char6.res == '3')
				{
          if(NV_OPER_FAILED == osal_snv_read(admin_flash_start_addr, 6, old_admin_pwd))//get old admin password
					{
						return 0;
					}
          
					if(NV_OPER_FAILED == osal_snv_write(admin_flash_start_addr, 6, char6.num))//modify admin password eg:888888MB3xxxxxx
					{
						return 0;
					}
          ready_report_hub_para(REPORT_MODIFY_ADMIN_PWD);
				}
				else if(char6.res == '4')
				{
					if(NV_OPER_FAILED == osal_snv_write(ssid_flash_start_addr, 6, char6.num))//modify lock ssid eg:888888MB4xxxxxx
					{
						return 0;
					}
          ssid_modify_flag = 1;
          osal_memcpy(new_ssid, char6.num, 6);
          ready_report_hub_para(REPORT_MODIFY_SSID);
				}
				else if(char6.res == '5')
				{            
					if(NV_OPER_FAILED == osal_snv_write(carlock_id_flash_start_addr, 6, char6.num))//modify lock id eg:888888MB4xxxxxx
					{
						return 0;
					} 
          ready_report_hub_para(REPORT_MODIFY_LOCK_ID);
          extern  uint8 report_count;
          report_count = 0;
				}
				else
				{
					return 0;
				}
				break;
				
			case 'C':
			    if(char6.res == '1')
				{
					if(1 == osal_memcmp(char6.num, "000000", 6))//control lock init eg:888888MC1000000
					{
						lock_init();
					}
				}
				break;
			case 'D':
			  if(char6.res == '1')
				{
					if(1 == osal_memcmp(char6.num, "000000", 6))//control lock soft reset eg:888888MD1000000
					{
						HAL_SYSTEM_RESET();//systerm reset 
					}
				}
				break;	
			case 'Q':
				if(char6.res == '1')//query eg:888888MQ1000000
				{
            lock_status_buf[0] = get_battery_level();;
            lock_status_buf[1] = get_lock_state();;
            lock_status_buf[2] = 2;
            lock_status_buf[3] = feedback_pwd_state;
            char1_notify(lock_status_buf, sizeof(lock_status_buf));
				}
				break;
			default:
				break;
		}
	}
	
	return 1;
}

unsigned char get_adc_battery(void)
{
  return HalAdcRead(HAL_ADC_CHANNEL_1, HAL_ADC_RESOLUTION_8);
}

unsigned int get_adc_motor(void)
{
  return HalAdcRead(HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_10);
}

unsigned char get_battery_level(void)
{
  unsigned char adc_bat = 0; 
  unsigned char level = 0;
  
//  batter_control(1);
   adc_bat = get_adc_battery()*2;
//   batter_control(0);
  
  if(adc_bat > 0 && adc_bat < 150)	//%0
    level = 1;
  if(adc_bat >= 150 && adc_bat < 155)	//%10
    level = 2;
  if(adc_bat >= 155 && adc_bat < 160)	//%20
    level = 3;
  if(adc_bat >= 160 && adc_bat < 165)	//%30
    level = 4;
  if(adc_bat >= 165 && adc_bat < 170)	//%40
    level = 5;
  if(adc_bat >= 170 && adc_bat < 175)	//%50
    level = 6;
  if(adc_bat >= 175 && adc_bat < 180)	//%60
    level = 7;
  if(adc_bat >= 180 && adc_bat < 185)	//%70
    level = 8;
  if(adc_bat >= 185 && adc_bat < 190)	//%80
    level = 9;
  if(adc_bat >= 190 && adc_bat < 195)	//%90
    level = 10;
  if(adc_bat >= 195 && adc_bat < 255)	//%100
    level = 11;
  
  return level;
}

void CalculateXYZDifferSum( void )                 //��ȡ��״̬
{
   if( Nxyz.x >  Pxyz.x )
   {
       differ_x =  Nxyz.x - Pxyz.x ;
   }
   else
   {
       differ_x =  Pxyz.x - Nxyz.x ;
   }
  
   if( Nxyz.y >  Pxyz.y )
   {
       differ_y =  Nxyz.y - Pxyz.y ;
   }
   else
   {
       differ_y =  Pxyz.y - Nxyz.y ;
   }
  
   if( Nxyz.z >  Pxyz.z )
   {
       differ_z =  Nxyz.z - Pxyz.z ;
   }
   else
   {
       differ_z =  Pxyz.z - Nxyz.z ;
   }/**/
  
  XYZDifferSum = differ_x + differ_y + differ_z;
  
//  differ_Vx =  Nxyz.x - Pxyz.x ;
//  differ_Vy =  Nxyz.y - Pxyz.y ;
//  differ_Vz =  Nxyz.z - Pxyz.z ;

//  int16 XYZDiffervSum = differ_Vx + differ_Vy + differ_Vz;
//  XYZDifferVSum = ABS( XYZDiffervSum );
}

uint8 Hmc5883GetCarState(void)                 //��ȡ��״̬
{ 
  if( Hmc5883_Get_Vector(&Nxyz) == 1)
  {
      CalculateXYZDifferSum();                         //�����ܱ仯��֮��
      QueueInput( XYZDifferSum ,Nxyz.y, QueueLength);        //���
      if( false == QueueMiddleOverEvent(QueueLength) )
        return CheckCarStateEmpty;        //����
      else
        return CheckCarStateDone;           // �����仯
  }
  else
  {
      return CheckCarStateEmpty;        // ����
  }
}

void UpdateHmc5883vector(void)                //���µش�����
{
  Pxyz.x = Nxyz.x;
  Pxyz.y = Nxyz.y;
  Pxyz.z = Nxyz.z;
}

void Hmc5883InitGetVector(void)                //��ʼ�����µش�
{
  double   b;
  uint32   xx,yy,zz;

  Hmc5883_Get_Vector(&Nxyz);                   //��ȡʵʱ����   ��֤�ϵ�ɼ���ȷ   
  
   xx = Nxyz.x*Nxyz.x;
   yy = Nxyz.y*Nxyz.y;
   zz = Nxyz.z*Nxyz.z;
   b= xx + yy + zz;
   b = sqrt(b);
   
   magnetic_intensity = (uint32)b;
  
//  Hmc5883_Get_Vector(&Nxyz);                   //��ȡʵʱ����   ��֤�ϵ�ɼ���ȷ   
//  Hmc5883_Get_Vector(&Nxyz);                   //��ȡʵʱ����      
//  Hmc5883_Get_Vector(&Nxyz);                   //��ȡʵʱ����      
//  UpdateHmc5883vector();
}

void  systemuncarmodework(void)
{
  if( MoterOnDelay != idle )
  {
      MoterOnDelay--; 
      return ;
  }
  if( (KeyStatus == carlock90_180state) || (KeyStatus == carlock90state) )       //��������ӳٲɼ�AD
  {
     ToHubReportFlag = Disable;                                                  //�г�ģʽ����޳�������־
     stop();
     MoterOnDelay = MoterOver90DelayN ;
     return;
  }
  if(( MoterState == Runing) && (MoterOnDelay == idle) && (KeyStatus != carlock90state) )
  {
      MotorADValue_carlock=get_adc_motor();
      if(( MotorADValue_carlock > MotorDownADValueN ) && ( MotorTarget == carlock0degree))   //to 0�����
      {
           stop();
           MotorADValue_carlock = idle;                     //  clear the ad
           MotorTarget = carlock90degree;                   //set carlock as 90��
           MoterOnDelay = MoterOnDelayN;
      }
      if(( MotorADValue_carlock > MotorUpADValueN ) && ( MotorTarget == carlock90degree))   //to 90�����
      {
           stop();
           MotorADValue_carlock = idle;                     //  clear the ad
           MotorTarget = carlock0degree;                   //set carlock as 0��
           MoterOnDelay = MoterOnDelayN;
           UncarADerrtimes++;                              //external fault times
           malignant_fault_count = UncarADerrtimes;        //  ���Թ���ͳ��
           CarLockDownedTime = CarLockDownedTimeN;
//           if( UncarADerrtimes >= UncarADerrtimesRage3 )
           if( UncarADerrtimes >= 1 )
           {
              UncarADerrtimes = UncarADerrtimesRage3;
              MotorTarget = carlock90degree;               //set carlock as 90��  ��ʱ�ر�
              ready_report_hub_para( REPORT_STATE_INFO );
              ToHubReportFlag = Enable;
           }
           if( ToHubReportFlag == Enable )
           {
               buzzer_control(1);  
           }
           else
           {
              BeepRunTime = BeepRunTimeRage;
           }
        }
  }  
  if(UncarADerrtimes > idle)
  {
    if( BeepRunTime !=idle )
    {
       BeepRunTime--;
       buzzer_control(1);
       led_control(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, 100, 100, 1500, 1);
    }
    else if( ToHubReportFlag != Enable )
    {
       buzzer_control(0);
       led_control(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, 1, 0, 1500, 0);
    }
  }
  if(CarLockDownedTime != 0)
  {
     CarLockDownedTime--;
  }
  else if(( MotorTarget == carlock0degree ) && ( KeyStatus == carlock0degree))
  {
     MoterOnDelay = MoterOnDelayN;
     MotorTarget = carlock90degree;                   //set carlock as 90��
  }
}

void  systemcaredmodework(void)                       //�г�ģʽ
{  
#if !defined(test)
  if( BeepTimes != 0)
  {
    if( BeepRunTime < 20)
    {
       BeepRunTime++;
       buzzer_control(1);
    }
    else if( BeepRunTime < 50)
    {
       BeepRunTime++;
       buzzer_control(0);
    }
    else
    {
       BeepTimes--;
       BeepRunTime = 0; 
    }
  }
  else
  {
      buzzer_control(0);
  }
#else
  if( BeepTimes != idle)
  {
       BeepTimes--;
  } 
#endif 
  if( KeyStatus == carlock90_180state )       //��������ӳٲɼ�AD
  {
     stop();
     MoterOnDelay = MoterOver90DelayN ;
     return;
  }
   if(Caredtime !=  idle  )
   {
     Caredtime--;
     if( ( Caredtime == 1 ) &&( CheckCarState != CheckCarStateCome ) )//��ʱ���ﳵδʻ��  ���Զ�̧��
     {                                                            
          stop();     
          MotorTarget = carlock90degree;              //set carlock as 90��       //������Ϊ������
          MoterOnDelay = MoterOnDelayN;                                         //�����ӳ�����ʱ��
          BeepRunTime = 0;
          BeepTimes = 4 ;
     }
     else if( Caredtime == 1 )
     {
           BeepRunTime = 0;
           BeepTimes = 4 ;
           set_use_times();                                                   //��¼ʹ�ô�������
           ready_report_hub_para( REPORT_STATE_INFO );                                   //���泵����
     }
     if( ( Caredtime % Hmc5883CaredUpdatetimeN) != 0 )                        //10ms * x  1s  �ɼ�һ������  һ���ж�
     {
       return ;                                      
     }
     if( ( KeyStatus == carlock0degree ) && ( CaredModeStartFlg == Enable ))
     {
         CaredModeStartFlg = Disable ;
         MoterOnDelay = MoterOnDelayN;                                         //�����ӳ�����ʱ��   
         
         return;
     }
#if defined(test)
     if( (KeyStatus == carlock0degree) && ( BeepTimes == idle) )
#else
//     if( (KeyStatus == carlock0degree) && ( BeepTimes == idle)  )
     if( (KeyStatus == carlock0degree) && ( BeepTimes == idle) && ( CheckCarState != CheckCarStateCome ) )
#endif 
     {
         
         CheckCarChange=Hmc5883GetCarState();                                     //���ش�����
         if( CheckCarChange != CheckCarStateEmpty )                               // ����״̬����̫
         { 
           stop();     
           MotorTarget = carlock0degree;            //set carlock as 90��        //������Ϊ������
           MoterOnDelay = MoterOnDelayN;                                         //�����ӳ�����ʱ��
           CheckCarState = CheckCarStateCome;                                    //���ó���Ϊ��״̬
           CheckCarUpdateFlg = CheckCarStateCome;
           BeepRunTime = 0;
           BeepTimes = 2 ;
           UncarADerrtimes = idle;                                              //�г�ģʽ����޳�������־
         }
     }
     else 
     {
       return ;                                      
     }
     return ;                                         //��ʱ����  ͣ��ʱ��  Ŀǰ30s
  }
 //----------------------����Ϊ�ж��Ƿ��ж�ת����-----------------------------------------/  
  if( MoterOnDelay != idle )
  {
      MoterOnDelay--;    
  }
  if(( MoterState == Runing ) && ( MoterOnDelay == idle ) ) //�ж��Ƿ���0�㵽 90���ʱ�����
  {
      KeyStatus = keyscan();
      if(  KeyStatus == carlock0state  )                    //��ز�������״̬
      {
         return ;
      }
      MotorADValue_carlock=get_adc_motor();
      //Motor AD overflow ,cared fault,����ģʽ
      if(( MotorADValue_carlock > MotorDownADValueN ) && ( MotorTarget == carlock0degree))        //to 0�����  
      {
           stop();
           MotorADValue_carlock = idle;                     //  clear the ad
           MotorTarget = carlock90degree;                   //  set carlock as 90��
           MoterOnDelay = MoterOnDelayN;                    //  �����ӳ�����ʱ��
      }
      else if(( MotorADValue_carlock > MotorUpADValueN ) && ( MotorTarget == carlock90degree))    //to 90�����
      {
           stop();
           MotorADValue_carlock = idle;                    //  clear the ad
           MotorTarget = carlock0degree;                   //  set carlock as 0��
           MoterOnDelay = MoterOnDelayN;                   //  �����ӳ�����ʱ��
           CheckCarState = CheckCarStateCome;              //  set as cared mode
           CheckCarUpdateFlg = CheckCarStateCome;
           CaredADerrtimes++;                              //  external fault times
           conventional_fault_count = CaredADerrtimes;     //  �������ͳ��         
           CarLockDownedTime = CarLockDownedTimeN;
           Hmc5883CaredUpdatetime = idle;                  //  ��������
           BeepRunTime = 0;
           BeepTimes = 5  ;
           if( CaredADerrtimes >= UseTimesN )              //  ���д�������
           {
              CaredADerrtimes = UseTimesN;
              ready_report_hub_para( REPORT_STATE_INFO );             // ���泵����
           }
        }
  }    
  
  /*if(CaredADerrtimes > idle)
  {
    if( BeepRunTime !=idle )
    {
       BeepRunTime--;
       buzzer_control(1);
       led_control(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, 100, 100, 1500, 1);
    }
    else if( ToHubReportFlag != Enable )
    {
       buzzer_control(0);
       led_control(HAL_LED_1 | HAL_LED_2 | HAL_LED_3, 1, 0, 1500, 0);
    }
  }*/
  if(CarLockDownedTime != idle)  //carlock to 0�� delaytime
  {
     CarLockDownedTime--;
  }
#if defined(test)
  else if(( MotorTarget == carlock0degree ) && ( KeyStatus == carlock0degree ) && (CarLockDownedTime == idle)  
                                             && (Caredtime == idle)) //waiting car parking            
#else
  else if(( MotorTarget == carlock0degree ) && ( KeyStatus == carlock0degree ) && (CarLockDownedTime == idle)  
            && (Caredtime == idle)  && (CarGotime == idle) && ( CarGoChecktime == idle )) //waiting car parking            
#endif 
  {                                                          //����Ŀ��Ϊ0��λ��0��ʱ���ж��Ƿ����ˣ��������������� ���شż����� ��
      if( CheckCarUpdateFlg == CheckCarStateCome )           //ȷ�ϳ��ڻ��߳���ʱ���µش�һ��
      {
          CheckCarUpdateFlg = CheckCarStateEmpty;            //  ���־λ
          QueueClear(QueueLength);                           //  clear
          while( Hmc5883_Get_Vector(&Nxyz) != 1);            //  ȷ�ϵش����ݶ�ȡ�ɹ�
          UpdateHmc5883vector();                             //  ���µش�����         
          return;
      }
     if(Hmc5883CaredUpdatetime !=  idle)
     {
       Hmc5883CaredUpdatetime--;
     }
#if defined(test)
     else if( BeepTimes == idle )                        // ����ʱ�䵽��,���Ե�����ȥ��ֹͣ����
#else
     else if( (BeepTimes == idle) && (CarGotime == idle) )                        // ����ʱ�䵽��,���Ե�����ȥ��ֹͣ����
#endif 
     {
        stop();   
        Hmc5883CaredUpdatetime = Hmc5883CaredUpdatetimeN;                        //�����г�ģʽ�ļ��ش�ʱ�䣨 �� ��
        CheckCarChange = Hmc5883GetCarState();                                   //���ش�����
        if( CheckCarChange == CheckCarStateDone )                                  //�鵽�г�����
        {
            CarGoChecktime = CarGoChecktimeN;
            BeepTimes = 1 ;
        }

     }
  }
  else if( ( KeyStatus == carlock90degree ) && ( MotorTarget == carlock90degree ) ) 
                                                      //Ŀ���λ�ö�Ϊ 90��ʱ��ȷ�ϳ�ȷʵ���˺󣬽����޳�ģʽ
  {
     MoterOnDelay = MoterOnDelayN;                    //set uncar mode MoterOn delay 
     CheckCarState = CheckCarStateGo; 
     Carlockmode = carlockmode_uncar;                 //set uncar mode                   
     ready_report_hub_para( REPORT_STATE_INFO );                 //���泵����
  }
  if(CarGoChecktime !=  idle)
  {
     CarGoChecktime--;
  }
  if(CarGoChecktime ==  CarGoChecktimeArrive)
  {
      if( false != Hmc5883Change_xy())
      { 
         CarGotime = CarLockCarUpTimeN - CarGoChecktimeN;
         BeepRunTime = 0;
         BeepTimes = 3 ;
      } 
  
  }
  if(CarGotime !=  idle)
  {
     CarGotime--;
  }
  if(CarGotime ==  CarLockupedTimeArrive)
  {
     CheckCarState = CheckCarStateGo; 
     MoterOnDelay = MoterOnDelayN;
     MotorTarget = carlock90degree;                   //set carlock as 90�㣬������������ֱ����
  }
}

void systemcontrol(void)
{ 
/*  uint8   CheckCarChange;

   if( Hmc5883UpdateTime != idle )
   {
       Hmc5883UpdateTime--;
   }
   else
   {
       Hmc5883UpdateTime = Hmc5883UpdateTimeN1;      //��ֵ�شŸ���ʱ��
       CheckCarChange = Hmc5883GetCarState();
       if( CheckCarChange == CheckCarStateEmpty  )    //Ϊ��ʱ�����µش�����
       {
           UpdateHmc5883vector();
       }
       else
       {
           Hmc5883UpdateTime = Hmc5883UpdateTimeN2;  //�ͽ�һ�θ���
       } 
  }*/

  switch(Carlockmode)
  {
    case carlockmode_uncar:     systemuncarmodework();     break; 
    case carlockmode_cared:     systemcaredmodework();     break; 
    default: break;
  }
}
void  carlockseatctrol(void)
{  
   if( SystemStabletime != idle )       //�ϵ��ӳ�
   {
       SystemStabletime--;
   }
   else                                 //�ϵ�ϵͳ 5s  �ȶ���ִ��
   {
     KeyStatus = keyscan();
     systemcontrol();  
     MoterState=motorctrol(Motorfuction, KeyStatus,MotorTarget);
   }
}

unsigned char is_lock(void)
{
   if(KeyStatus==carlock90state)
   {
      return 1;
   }
   else
   {
      return 0;
   }
}

unsigned char get_lock_state(void)
{
  unsigned char ret = 0;
  
  if(0 == is_lock())
    ret = 4;
  if(1 == is_lock())
    ret = 5;
  
  return ret;
}

unsigned char get_notify_data(void)
{
  unsigned char tmp = 0;
  
  lock_status_M = get_battery_level();
  lock_status_N = get_lock_state();
  
  if( lock_status_M == 1)
  {
    ready_report_hub_para( REPORT_STATE_INFO );
  }
  
  
  tmp = (lock_status_M << 4) & 0xf0;
  tmp |= lock_status_N & 0x0f;
  
  return tmp;
}

////////////////////////470M//////////////////////////////////////////

uint8 Read_Dio0(void)
{
  return P1_3;
}


////////////////////////////////////////////////////////////////////////////////
// �������� : ���߽��������ж����
// ������� : ��
// ���ز��� : ��
// ˵��     : DIO3�ж�����CAD������ʱ��ʱ�䵽����˯�ߡ� DIO0�����ж����
////////////////////////////////////////////////////////////////////////////////
#pragma vector = P1INT_VECTOR    
__interrupt void P1_ISR(void) 
{ 
    rf470m_irq();
    P1IFG &= ~(1<<3);                                       //����жϱ�־λ
    P1IF = 0;
    
    LSD_RF_RXmode();                                     //set RXmode
    /*
    //CAD����ʱ�䵽��ִ��˯�߲���
    if(DIO3_IFG&&DIO3_IE)//CAD��ʱָʾ�жϣ���ʱʱ�䵽�����½���˯��
    {
        DIO3_IFG = 0;            //���MCU�жϱ�־λ
        LSD_RF_WOR_Execute(0);   //��ʱʱ�䵽�����½���˯��ģʽ
    }
    //���Ѱ��жϻ�����͸�����ڴ���
    else if(DIO0_IFG&&DIO0_IE)    //���ݴ����жϴ���
    {
        DIO0_IFG = 0;                                  //���DIO0�жϱ�־λ
        LSD_RF_RxPacket(Rxbuffer);   //�������ݰ�
        if((Rxbuffer[0]==5)&&(Rxbuffer[1]==6)&&(G_LoRaConfig.PayloadLength==2))
        {
            //Ŀǰ�����ϴν��յĳ���2
            G_LoRaConfig.PayloadLength = 30;//�����ѣ��ϴ�����
            LSD_RF_SendPacket(TXbuffer);
        }
        //��������WOR��������ʾWOR����
        OFF_Timerout();                      //�رճ�ʱ��ʱ��
        LSD_RF_WORInit();                   //WOR��ʼ��
        LSD_RF_WOR_Execute(0);              //����ִ��WOR
 
    }
    else;
    P1IFG=0;
    */
} 

////////////////////////////////////////////////////////////////////////////////
// �������� : ���߻����ж�ָʾ
// ������� : ��
// ���ز��� : ��
// ˵��     : DIO1����CADDetectde����ʾǰ��������
////////////////////////////////////////////////////////////////////////////////
#pragma vector = P2INT_VECTOR    
__interrupt void P2_ISR(void) 
{ 
//     if(DIO1_IFG&&DIO1_IE)//����WOR�����жϴ�����
//     {
//         DIO1_IFG = 0;    //�����־λ
//         //G_LoRaConfig.PayloadLength = 30;
//         LSD_RF_WOR_Exit();  //��������ڿɱ�����ģʽ����Ч���˳�WOR�������ģʽ
//         SysTick2=0;
//         ON_Timerout();   //����һ����ʱ��ʱ����ʱ�䵽�ˣ����û���յ�������Ч���������½���WOR
//     }
//     else;
//     P2IFG=0;
}

//////////////////////////////////////////////////////////
//888888	:admin pwd / user pwd
//M/U   	:admin/user
//ABCQ: 	:cmd
//0123		:res
//xxxxxx	:control cmd / modify pwd 
//cs1234	:ssid
//ss:		:ss = ~(sum(21byte))
////////////////////////////////////////////////////////

uint8 rf470m_irq(void)
{    
  hub_user_data_t   hub;

  if(Read_Dio0() == 1)//interrupt
  {
    uint16 sum = 0;
//    uint16 ss = 0;
//    uint16 tmp = 0;
    
    osal_memset(hub_buf, 0, sizeof(hub_buf));
    LSD_RF_RxPacket(hub_buf);
    
    if(1 == osal_memcmp(hub_buf + 26, "ss", 2))//��27, 28�ֽ��Ƿ�Ϊ�ϱ���ACK
    {
      if(1 == osal_memcmp(hub_buf, report_buf, sizeof(report_buf)))
      {
        last_report_get_ack_flag = REPORT_IDLE;  //�յ����µ��ϱ���ACK
        Rf470_setfre1();
        return 1;
      }
    }
    
    //�����ǽ��ܵ�������
    for(int i=0; i<21; i++)
    {
      sum += hub_buf[i];
    }  
    if( hub_buf[21] != 's' &&  hub_buf[22] != 's')
    {   
      return 0;
    }
//    ss = ~sum;
//    tmp = ((hub_buf[21] << 8) & 0xff00) | hub_buf[22] ;
    
   // if(tmp != ss)
   // {
   //   return 0; //data error
   // }
    
    osal_memcpy(hub.pwd, hub_buf, 6);
    hub.id = hub_buf[6];
    hub.cmd = hub_buf[7];
    hub.res = hub_buf[8];
    osal_memcpy(hub.num, hub_buf + 9, 6);
    osal_memcpy(hub.lockid, hub_buf + 15, 6);
    
    hub_process(hub);
    
    return 1;
 }
 else
 {
  return 0;
 }
  
}

void get_use_times(uint8 use_times[2])
{
  if(NV_OPER_FAILED == osal_snv_read(use_lock_times_start_addr, 2, use_times))//16 ����
  {
    return ;
  }
}

void set_use_times(void)
{
 uint16 use_times0;
 uint8  use_times[2];

  if(NV_OPER_FAILED == osal_snv_read(use_lock_times_start_addr, 2, use_times))//16 ����
  {
    return ;
  }
  use_times0 =  (use_times[0] <<8) +  use_times[1];
  use_times0 ++;
  use_times[0] = (use_times0&0xff00) >> 8;
  use_times[1] = (use_times0&0x00ff) ;
 
  if(NV_OPER_FAILED == osal_snv_write(use_lock_times_start_addr, 2, use_times))//16 ����
  {
    return ;
  }
}

void get_lock_info(uint8 lock_info[9])
{
	uint8 bat = 0;
//	uint8 percent = 0;
	
//	bat = get_adc_battery()*2;
//	percent = (uint8)(100 * (100.0 * bat) / 256);//changge to 0% ~ 100%
  bat = get_battery_level()-1;

	//�ַ�����
  if(bat == 10)
  {
    lock_info[0] = '9'; 
    lock_info[1] = '9'; 
  }
  else
  {
    lock_info[0] = '0'; 
    lock_info[1] =  bat  + '0'; 
  }
   
	//16����
	lock_info[2] = (conventional_fault_count >> 8) & 0xff;
	lock_info[3] =  conventional_fault_count       & 0xff;
	
	//16����
	lock_info[4] = malignant_fault_count  + '0';                
	
	//16����
	lock_info[5] = (magnetic_intensity >> 24) & 0xff;     
	lock_info[6] = (magnetic_intensity >> 16) & 0xff;     
	lock_info[7] = (magnetic_intensity >> 8)  & 0xff;    
	lock_info[8] =  magnetic_intensity        & 0xff;  
}

void ready_report_hub_para(uint8 report_type)
{
extern uint16  report_countnum ;

  last_report_get_ack_flag = REPORT_START;
   save_report_tpye = report_type; 
   report_countnum = 1;
}

static uint8 tmp[6] = {0};
static uint8 use_times[2] = {0};
static uint8 lock_status = 0;
static uint8 lock_info[9] = {0};
//static uint8 ss[2] = {0};
void report_hub( void )
{
  osal_memset(report_buf, '0', 30);
  RfSentRXmodeEnFlag = Enable; 
  RfSentDelaytime =  RfSentRXmodeStartTrigN;

  if( save_report_tpye != REPORT_LOOP_STATE)
  {
    //LSD_RF_Init(frefunction2);
  }
	switch(save_report_tpye)
	{
		case REPORT_STATE_INFO :
			osal_memcpy(report_buf, "ZT", 2);
			
      if(NV_OPER_FAILED == osal_snv_read(carlock_id_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 2, tmp, 6);

      if(NV_OPER_FAILED == osal_snv_read(ssid_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 8, tmp, 6);
      
      if(NV_OPER_FAILED == osal_snv_read(use_lock_times_start_addr, 2, use_times))//16 ����
      {
        return ;
      }
			osal_memcpy(report_buf + 14, use_times, 2);
			
      lock_status = get_lock_state() + '0';
			report_buf[16] = lock_status;
			
			get_lock_info(lock_info);
			osal_memcpy(report_buf + 17, lock_info, 9);
			
			osal_memcpy(report_buf + 26, "ss", 2);
			break;
		case REPORT_MODIFY_ADMIN_PWD :
			osal_memcpy(report_buf, "GM", 2);

      if(NV_OPER_FAILED == osal_snv_read(carlock_id_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 2, tmp, 6);

      osal_memcpy(tmp, old_admin_pwd, 6);
			osal_memcpy(report_buf + 8, tmp, 6);
			
			osal_memcpy(report_buf + 14, "MR0", 3);

      if(NV_OPER_FAILED == osal_snv_read(admin_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 17, tmp, 6);
			
			osal_memcpy(report_buf + 23, "000", 3);

			osal_memcpy(report_buf + 26, "ss", 2);
			break;
		case REPORT_MODIFY_USER_PWD :
			osal_memcpy(report_buf , "GM", 2);

      if(NV_OPER_FAILED == osal_snv_read(carlock_id_flash_start_addr, 6, tmp))
		  {
			  return;
		  }
			osal_memcpy(report_buf + 2, tmp, 6);
			
      if(NV_OPER_FAILED == osal_snv_read(admin_flash_start_addr, 6, tmp))
		  {
			  return;
		  }
			osal_memcpy(report_buf + 8, tmp, 6);
			
			osal_memcpy(report_buf + 14, "MR1", 3);
      
      if(NV_OPER_FAILED == osal_snv_read(user_flash_start_addr, 6, tmp))
		  {
			  return;
		  }
			osal_memcpy(report_buf + 17, tmp, 6);
			
			osal_memcpy(report_buf + 23, "000", 3);

			osal_memcpy(report_buf + 26, "ss", 2);
			break;
		case REPORT_MODIFY_PWD_DEAD_DATE :
			osal_memcpy(report_buf , "GM", 2);
			
      if(NV_OPER_FAILED == osal_snv_read(carlock_id_flash_start_addr, 6, tmp))
		  {
			  return;
		  }
			osal_memcpy(report_buf + 2, tmp, 6);

      if(NV_OPER_FAILED == osal_snv_read(admin_flash_start_addr, 6, tmp))
		  {
			  return;
		  }
			osal_memcpy(report_buf + 8, tmp, 6);
			
			osal_memcpy(report_buf + 14, "MR2", 3);
			
      if(NV_OPER_FAILED == osal_snv_read(dead_date_flash_start_addr, 6, tmp))
		  {
			  return;
		  }
			osal_memcpy(report_buf + 17, tmp, 6);
			
			osal_memcpy(report_buf + 23, "000", 3);
			
			osal_memcpy(report_buf + 26, "ss", 2);
			break;
		case REPORT_MODIFY_SSID :
			osal_memcpy(report_buf , "GM", 2);
			
      if(NV_OPER_FAILED == osal_snv_read(carlock_id_flash_start_addr, 6, tmp))
		  {
			  return;
		  }
			osal_memcpy(report_buf + 2, tmp, 6);
      
      if(NV_OPER_FAILED == osal_snv_read(admin_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 8, tmp, 6);
			
			osal_memcpy(report_buf + 14, "MR3", 3);

      if(NV_OPER_FAILED == osal_snv_read(ssid_flash_start_addr, 6, tmp))
		  {
			  return;
		  }
			osal_memcpy(report_buf + 17, tmp, 6);
			
			osal_memcpy(report_buf + 23, "000", 3);

			osal_memcpy(report_buf + 26, "ss", 2);
			break;
    case REPORT_MODIFY_LOCK_ID :
			osal_memcpy(report_buf , "GM", 2);
			
      if(NV_OPER_FAILED == osal_snv_read(carlock_id_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 2, tmp, 6);

      if(NV_OPER_FAILED == osal_snv_read(admin_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 8, tmp, 6);
			
			osal_memcpy(report_buf + 14, "MR4", 3);

      if(NV_OPER_FAILED == osal_snv_read(carlock_id_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 17, tmp, 6);
			
			osal_memcpy(report_buf + 23, "000", 3);

			osal_memcpy(report_buf + 26, "ss", 2);
			break;
		case REPORT_LOOP_STATE :
			osal_memcpy(report_buf , "LX", 2);

      if(NV_OPER_FAILED == osal_snv_read(carlock_id_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 2, tmp, 6);
            
      if(NV_OPER_FAILED == osal_snv_read(ssid_flash_start_addr, 6, tmp))
      {
        return;
      }
			osal_memcpy(report_buf + 8, tmp, 6);

      if(NV_OPER_FAILED == osal_snv_read(use_lock_times_start_addr, 2, use_times))//16 ����
      {
        return ;
      }
			osal_memcpy(report_buf + 14, use_times, 2);
			
      lock_status = get_lock_state() + '0';
			report_buf[16] = lock_status;
			
			get_lock_info(lock_info);
			osal_memcpy(report_buf + 17, lock_info, 9);

			osal_memcpy(report_buf + 26, "ss", 2);
			break;
		default :	
			break;
	}
  LSD_RF_SendPacket(report_buf);                        //���յ���������ΪACK      
  LSD_RF_RXmode();                                     //set RXmode
}
void Rf470_setfre1(void)   //��ԭΪƵ��һ
{
   //LSD_RF_Init(frefunction1);               //��ԭƵ��һ  set RXmode
}


uint8 hub_process(hub_user_data_t hub)
{    
    static	unsigned char pwd_state 	= 0;
	
    osal_memset(pwd, '0', sizeof(pwd));//change to character
    osal_memset(lockid, '0', sizeof(lockid));//change to character
    
    //�ж�lock_id�Ƿ���ȷ
		if(NV_OPER_FAILED == osal_snv_read(carlock_id_flash_start_addr, 6, lockid))
		{
			return 0;
		}
    if(1 == osal_memcmp(hub.lockid, lockid, sizeof(lockid)))//check admin pwd
		{
			 lockid_state = lockidright;//lock id is right
		}
		else
		{
			 lockid_state = lockiderr;//admin pwd is err
		}
    
    if(lockid_state == lockiderr)
    {
      return 0;
    }
    else  
    {
      RfSentRXmodeEnFlag = Enable; 
      RfSentDelaytime =  RfSentRXmodeStartTrigN;
      LSD_RF_SendPacket(hub_buf);                        //���յ���������ΪACK 
      LSD_RF_RXmode();                                     //set RXmode
    }

	if(hub.id == 'U')
	{
      
		if(NV_OPER_FAILED == osal_snv_read(user_flash_start_addr, 6, pwd))
		{
			return 0;
		}
			  
		if(1 == osal_memcmp(hub.pwd, pwd, sizeof(pwd)))//check pwd
		{
			pwd_state = upwdright;//pwd is right
		}
		else
		{
			pwd_state = upwderr;//pwd is err
		}
    
    if( pwd_state ==  upwderr)
      return 0;
	}
	else if(hub.id == 'M')
  {
      
		if(NV_OPER_FAILED == osal_snv_read(admin_flash_start_addr, 6, pwd))
		{
			return 0;
		}
		
		if(1 == osal_memcmp(hub.pwd, pwd, sizeof(pwd)))//check admin pwd
		{
			 pwd_state = mpwdright;//admin pwd is right
		}
		else
		{
			 pwd_state = mpwderr;//admin pwd is err
		}
    
    if( pwd_state == mpwderr )
      return 0;
  
    switch(hub.cmd)
    {
      case 'A':
        if(1 == osal_memcmp(hub.num, "200000", 6))//admin control up lock eg:888888MA0200000cs1234ss
        {
           cargo_init();
        }
        else if(1 == osal_memcmp(hub.num, "100000", 6))//admin control down lock eg:888888MA0100000cs1234ss
        {
           carcome_init();
        }
        else if(1 == osal_memcmp(hub.num, "3", 1))//admin control lock buzzer eg:888888MA0300000cs1234ss
        {
          control_buzzer_flag = 1; 
          buzzer_control(1);
          buzzer_on_time = (hub.num[4]-'0')*10;
          buzzer_on_time += (hub.num[5]-'0'); 
          buzzer_on_time *= buzzer_on_time;
         //beepontime=0;
        }
        else
        {
          return 0;
        }
        break;
        
      case 'B':
        if(hub.res == '1')
        {
          if(NV_OPER_FAILED == osal_snv_write(user_flash_start_addr, 6, hub.num))//modify user password eg:888888MB1xxxxxxcs1234ss
          {
            return 0;
          }
          ready_report_hub_para(REPORT_MODIFY_USER_PWD);
        }
        else if(hub.res == '2')
        {
          if(NV_OPER_FAILED == osal_snv_write(dead_date_flash_start_addr, 6, hub.num))//modify user password dead date eg:888888MB2xxxxxxcs1234ss
          {
            return 0;
          }
          ready_report_hub_para(REPORT_MODIFY_PWD_DEAD_DATE);
        }
        else if(hub.res == '3')
        {
          if(NV_OPER_FAILED == osal_snv_read(admin_flash_start_addr, 6, old_admin_pwd))//get old admin password 
          {
            return 0;
          }
          
          if(NV_OPER_FAILED == osal_snv_write(admin_flash_start_addr, 6, hub.num))//modify admin password eg:888888MB3xxxxxxcs1234ss
          {
            return 0;
          }
          ready_report_hub_para(REPORT_MODIFY_ADMIN_PWD);
        }
        else if(hub.res == '4')
        {
          if(NV_OPER_FAILED == osal_snv_write(ssid_flash_start_addr, 6, hub.num))//modify lock ssid eg:888888MB4xxxxxxcs1234ss
          {
            return 0;
          }

          ssid_modify_flag = 1;
          osal_memcpy(new_ssid, hub.num, 6);
          ready_report_hub_para(REPORT_MODIFY_SSID);
        }
        break;
      case 'C':
        if(hub.res == '1')
        {
          if(1 == osal_memcmp(hub.num, "000000", 6))//control lock init eg:888888MC1000000cs1234ss
          {
            lock_init();
          }
        }
        break;
			case 'D':
			  if(hub.res == '1')
				{
					if(1 == osal_memcmp(hub.num, "000000", 6))//control lock soft reset eg:888888MD1000000
					{
						HAL_SYSTEM_RESET();//systerm reset 
					}
				}
				break;	
      case 'Q':
        if(hub.res == '1')//query eg:888888MQ1000000cs1234ss
        {
          ready_report_hub_para(REPORT_STATE_INFO);
        }
        break;
      default:
        break;
    }
	}
  
  return 1;
}

////////////////////////////end 470M///////////////////////////////////////////