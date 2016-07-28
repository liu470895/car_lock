#include "app_led_control.h"
#include "hal_led.h"

/********************************************
* @��������led_control
* @��������: ����LED������
* @��ڲ��� :leds: ������ѡ���LED������ ����5��LED 
* @          numBlinks: ����LED��˸�Ĵ���  
* @          percent: ����LED����ռ�ܿ������ڵİٷֱ�
* @          period: ����LEDһ�ε�ʱ������
* @          led_status: ����LED��״̬��Ϊ1������Ϊ0����
* @����ֵ:void
*********************************************/
void led_control(uint8 leds, uint8 numBlinks, uint8 percent, uint16 period, uint8 led_status)
{
#if (defined (BLINK_LEDS)) && (HAL_LED == TRUE)
  leds &= HAL_LED_ALL;
  if(led_status == 1)
    HalLedBlink( leds, numBlinks, percent, period);  
  else
    HalLedBlink( leds, numBlinks,0, period);
#endif
}
