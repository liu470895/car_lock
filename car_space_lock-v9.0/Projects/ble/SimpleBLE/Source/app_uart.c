#include "hal_uart.h"
#include "app_uart.h"
#include "OSAL.h"
#include "npi.h"


/*
1, ˼·:  �������յ����ݺ󣬾ͻ����ϵ������»ص���������ʵ�ʲ����з��֣��˻ص�
��������Ƶ���� ����㲻ִ��NPI_ReadTransport�������ж�ȡ�� ��ô����ص������ͻ�
Ƶ���ر�ִ�У����ǣ���ͨ�����ڷ���һ�����ݣ� �㱾�����봦����һ����һ�ε����ݣ����ԣ�
����������������ʱ��Ĵ������� Ҳ�����յ����ݹ�����߳�ʱ���Ͷ�ȡһ�����ݣ� 
Ȼ����ݵ�ǰ��״̬����ִ�У����û�������ϣ��Ͱ��������ݵ���AT����� �������
���ˣ��Ͱ������͵��Զˡ�  ---------------amomcu   2014.08.17
*/
void NpiSerialCallback( uint8 port, uint8 events )
{
    (void)port;//�Ӹ� (void)����δ�˱������澯����ȷ���߻�������������������

    if (events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))   //����������
    {
        uint8 numBytes = 0;

        numBytes = NPI_RxBufLen();           //�������ڻ������ж����ֽ�
        
        if(numBytes == 0)
        {
            return;
        }
        else
        {
            //���뻺����buffer
            uint8 *buffer = osal_mem_alloc(numBytes);
            if(buffer)
            {
                //��ȡ��ȡ���ڻ��������ݣ��ͷŴ�������   
                NPI_ReadTransport(buffer,numBytes);   

                //���յ������ݷ��͵�����-ʵ�ֻػ� 
                NPI_WriteTransport(buffer, numBytes);  

                //�ͷ�����Ļ�����
                osal_mem_free(buffer);
            }
        }
    }
}