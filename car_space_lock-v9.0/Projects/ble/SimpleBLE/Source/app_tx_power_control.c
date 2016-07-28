#include "app_tx_power_control.h"
#include "hci.h"

/********************************************
* @��������Tx_Power_Cmd_Control
* @��������: ���Ʒ���Ĺ���
* @��ڲ��� :tx_power_mode ,�����ֻ�����������ģ�鷢�͵�����ֵ��ʵ�ַ��书�ʵĸı�
* @          ����4��ģʽ����0-3ģʽ������Ĺ�����������,Ĭ����ģʽ3,������书��
* @����ֵ:void
*********************************************/
void tx_power_cmd_control(uint8 tx_power_mode)
{
    HCI_EXT_SetTxPowerCmd( tx_power_mode);    
}