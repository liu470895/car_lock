#include "sx1276-LoRa.h"
#include "SX1276_LPC1114.h"
#include "LSD_RF_SX1276.h"
#include "LSD_RF_APPrf.h"
#include "comdef.h"
#include "npi.h"
#include "app_process.h"

uint8 start_sleep_timerout_flag = 0;
extern uint8 start_timerout_flag = 0;

/*
  ע�⣬���ļ��ĺ���ʵ���˴�RF��������MCU�ļ���

*/
////////////////////////////////////////////////////////////////////////////////
// �������� : RF�������ݰ�
// ������� : uint8*data����������ָ��
// ���ز��� : ��
// ˵��     : ���ݷ�����ɺ�DIO0�ӵ͵�ƽ��ɸߵ�ƽ��ÿ�ε��ô˺��������Զ��Ƚ�DIO0��Ϊ�ͣ��ȴ��ߵ�ƽ
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_SendPacket(uint8*cbuf)
{
  uint32 j=0xffff;	
  
    P1IEN &= ~(1<<3);                                   //��ֹDIO0�ж�
    SX1276_TxPacket(cbuf);                              //��������
    while( P1_3 == 0 )
    {
       j--;
       if( j==0 )
       {
          break;
       }
    }
    P1IFG &= ~(1<<3);                                   //����жϱ�־λ
/*
    unsigned long  j=16777215;                          //��ʱ�ã��û���Ҫ����ʵ�����������
	
    P1IEN &= ~(1<<3);                                   //��ֹDIO0�ж�
    SX1276_TxPacket(cbuf);                              //��������
    while( P1_3 != 1 )
    {
		  j--;                                  //�ȴ�GDIO0��ƽΪ��
    }
    P1IFG &= ~(1<<3);                                   //����жϱ�־λ
*/}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF�������״̬
// ������� : 
// ���ز��� : ��
// ˵��     : ����������ɺ�DIO0�ӵ͵�ƽ��ɸߵ�ƽ��
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_RXmode(void)
{
    Rx_mode();                      //RF���ջ��л���RXģʽ
    P1IEN |= (1<<3);                      //ENABLE DIO0�ж�
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF�������״̬
// ������� : 
// ���ز��� : ��
// ˵��     : ����������ɺ�DIO0�ӵ͵�ƽ��ɸߵ�ƽ��
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_RXmode1(void)
{
    //Rx_mode1();                      //RF���ջ��л���RXģʽ
    P1IEN &= ~(1<<3);                      //ENABLE DIO0�ж�
}

////////////////////////////////////////////////////////////////////////////////
// �������� : RF����Sleep״̬
// ������� : ��
// ���ز��� : ��
// ˵��     : ��
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_Sleepmode(void)
{
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );     
    SX1276Write(REG_LR_IRQFLAGS,0xff);                           
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_SLEEP );       
    PA_RX_L;           //PA_RX ��ʼ�����Ϊ0
    PA_TX_L;           //PA_TX ��ʼ�����Ϊ0   Ŀ���ǽ��ʹ�������
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF  CAD��ʼ��
// ������� : ��
// ���ز��� : ��
// ˵��     : DIO1--CADDetected    DIO3---CADDone
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_CADinit(void)
{
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );  
    SX1276Write( REG_LR_PREAMBLEMSB,0xf0);
    SX1276Write( REG_LR_PREAMBLELSB,0xff);
    SX1276Write( REG_LR_IRQFLAGSMASK,\
	~(RFLR_IRQFLAGS_CADDONE|RFLR_IRQFLAGS_CADDETECTED));
    //
    SX1276Write( REG_LR_DIOMAPPING1,\
	  RFLR_DIOMAPPING1_DIO3_00 | RFLR_DIOMAPPING1_DIO1_10);
    SX1276WriteRxTx(false);     //set RF switch to RX path
    SX1276Write(REG_LR_IRQFLAGS,0xff);      
}
////////////////////////////////////////////////////////////////////////////////
// �������� : RF����CAD�������ŵ����һ��
// ������� : ��
// ���ز��� : ��
// ˵��     :   ����ʱ��ԼΪ(2^SF+32)/BW
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_CAD_Sample(void)
{
    SX1276WriteRxTx(false);     //set RF switch to RX path
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );  
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_CAD ); 
}
////////////////////////////////////////////////////////////////////////////////
// �������� : WOR��ʼ��
// ������� : ��
// ���ز��� : ��
// ˵��     :   DIO1 :�����ж�   DIO3��CAD��ʱ�жϣ�Ҳ������Ϊ�ǽ��ռ���жϣ�
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_WORInit(void)
{
  LSD_RF_CADinit();        //CAD���ܳ�ʼ��
  //CADDoneʹ��
//  SX_DIO3_DIR=0;                       //������
//  DIO3_IFG = 0;                        //���DIO0�жϱ�־λ
//  DIO3_IES = 0;                        //����DIO0�����ش�����ʽ
//  DIO3_IE = 1;                         //ʹ��DIO0�ж�
//  //CADDetectedʹ��
//  SX_DIO1_DIR=0;                       //������
//  DIO1_IFG = 0;                        //���DIO0�жϱ�־λ
//  DIO1_IES = 0;                        //����DIO0�����ش�����ʽ
//  DIO1_IE = 1;                         //ʹ��DIO0�ж�
  
  P1IEN &= ~(1<<3);                     //��ֹDIO0�ж�
}
////////////////////////////////////////////////////////////////////////////////
// �������� : ִ��WOR����
// ������� : uint8 cclen  0������˯�ߡ�1������CAD���ģʽ
// ���ز��� : ��
// ˵��     :   
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_WOR_Execute(uint8 cclen)
{
  switch(cclen)
  {
    case 0:   //����˯��
      LSD_RF_Sleepmode();      //����˯��ģʽ
      ON_Sleep_Timerout();     //����˯�߳�ʱ��ʱ��
      break;
    case 1:   //����CAD���ģʽ
      OFF_Sleep_Timerout();    //�ر�˯�߳�ʱ��ʱ��
      LSD_RF_CAD_Sample();     //����CADһ�� 
      break;
    default: 
      break;
  }
}
////////////////////////////////////////////////////////////////////////////////
// �������� : WOR��RX
// ������� : ��
// ���ز��� : ��
// ˵��     : �˳�WOR������RXģʽ��ǰ��preamble��Ȼ�����������ֵ��
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_WOR_Exit()
{
    OFF_Sleep_Timerout();
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );      
    SX1276Write( REG_LR_PAYLOADLENGTH,G_LoRaConfig.PayloadLength);                    
    SX1276Write( REG_LR_IRQFLAGSMASK, ~(RFLR_IRQFLAGS_RXDONE)); 
    SX1276Write( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_00 );
    SX1276WriteRxTx(false);                                     //set RF switch to RX path
    SX1276Write( REG_LR_FIFOADDRPTR,0x00);                      
    SX1276Write(REG_LR_IRQFLAGS,0xff);                          
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_RECEIVER );   
    P1IEN |= (1<<3);                      //ENABLE DIO0�ж�
    
//    DIO1_IE = 0;                         //��ֹDIO1
//    DIO3_IE = 0;                         //��ֹDIO3
}



////////G_LoRaConfig.PayloadLength = 2; 
////////        uint16_t Preamble_Length = (uint16_t)(2200/G_TsXms+0.5);//����ǰ������4.2s
////////        LSD_RF_Awake(WakeAddr,Preamble_Length);


////////////////////////////////////////////////////////////////////////////////
// �������� : RF���ͻ��Ѱ�
// ������� : uint8*data����������ָ��,ǰ������
// ���ز��� : ��
// ˵��     : 
////////////////////////////////////////////////////////////////////////////////
void LSD_RF_Awake(uint8*cbuf,uint16 Preamble_Length)
{
	  P1IEN &= ~(1<<3);             //��ֹDIO0�ж�
    
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );      
    SX1276Write( REG_LR_PAYLOADLENGTH,G_LoRaConfig.PayloadLength);                    
    SX1276WriteRxTx(true);                                      
    SX1276Write( REG_LR_FIFOADDRPTR,0x80);                      
    SX1276WriteBuffer(REG_LR_FIFO,cbuf,G_LoRaConfig.PayloadLength);                   
    SX1276Write(REG_LR_IRQFLAGS,0xff);                          
    SX1276Write( REG_LR_IRQFLAGSMASK, ~(RFLR_IRQFLAGS_TXDONE));  
    SX1276Write( REG_LR_DIOMAPPING1, RFLR_DIOMAPPING1_DIO0_01 );
    SX1276Write( REG_LR_PREAMBLEMSB,(uint8)(Preamble_Length>>8));//set preamble length
    SX1276Write( REG_LR_PREAMBLELSB,(uint8)Preamble_Length);//set preamble length
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_TRANSMITTER );  
    while(P1_3 != 1);                               //�ȴ�GDIO0��ƽΪ��
 
    P1IFG &= ~(1<<3);                                             //����жϱ�־λ
    //�����껽�����ݰ��󣬽�ǰ��ʱ��Ļ�����Ĭ��ֵ��
    SX1276Write( REG_LR_OPMODE, 0x80|RFLR_OPMODE_STANDBY );     
    SX1276Write( REG_LR_PREAMBLEMSB,0);//set preamble length
    SX1276Write( REG_LR_PREAMBLELSB,10);//set preamble length    
}

//for test
void SX1278_test(void)
{
	if(SX127x_init( frefunction1 )!=NORMAL)	
	{
		NPI_PrintString("SX1278 init failed\r\n");
	}		
  LSD_RF_RXmode();
}

void LSD_RF_Init(uint8 frefunction)
{
  if(SX127x_init( frefunction )!=NORMAL)	
  {
    NPI_PrintString("SX1278 init failed!\r\n");
  }		
  else
  {
     NPI_PrintString("SX1278 init success!\r\n");
  }
  
  LSD_RF_RXmode();  
  
//  LSD_RF_WORInit();                   //WOR��ʼ��
//  LSD_RF_WOR_Execute(0);              //����ִ��WOR
}



////////////////////////////////////////////////////////////////////////////////
// �������� : ����˯�߳�ʱ��ʱ��
// ������� : ��
// ���ز��� : ��
// ˵��     : 
////////////////////////////////////////////////////////////////////////////////
void ON_Sleep_Timerout(void)
{    
  start_sleep_timerout_flag = 1;
//    TA0R =0;           //�����ʱ��������
//    TA0CTL |= TASSEL_1 | MC_1;    //������ʱ��ʱ��        
}
////////////////////////////////////////////////////////////////////////////////
// �������� : �ر�˯�߳�ʱ��ʱ��
// ������� : ��
// ���ز��� : ��
// ˵��     : 
////////////////////////////////////////////////////////////////////////////////
void OFF_Sleep_Timerout(void)
{    
  start_sleep_timerout_flag = 0;
////    TA0R =0;           //�����ʱ��������
//    TA0CTL = TASSEL_1 | MC_0;    //�رն�ʱ��       
}

////////////////////////////////////////////////////////////////////////////////
// �������� : ������ʱ��ʱ��
// ������� : ��
// ���ز��� : ��
// ˵��     : 
////////////////////////////////////////////////////////////////////////////////
void ON_Timerout(void)
{          
  start_timerout_flag = 1;
//    TA1R =0;           //�����ʱ��������
//    TA1CTL |= TASSEL_1 | MC_1;    //������ʱ��ʱ��        
}
////////////////////////////////////////////////////////////////////////////////
// �������� : �رճ�ʱ��ʱ��
// ������� : ��
// ���ز��� : ��
// ˵��     : 
////////////////////////////////////////////////////////////////////////////////
void OFF_Timerout(void)
{   
  start_timerout_flag = 0;
////    TA0R =0;           //�����ʱ��������
//    TA1CTL = TASSEL_1 | MC_0;    //�رն�ʱ��       
}


