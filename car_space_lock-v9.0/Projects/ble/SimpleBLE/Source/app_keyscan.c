#include "app_keyscan.h"
#include "hal_board_cfg.h"

uint8 keyscan(void)
{
  uint8 keystatus=0;
  
  if(KEY&KEY_X1) 
  {
     keystatus |= KEY_X1;
  }
  else 
  {
     keystatus &= ~KEY_X1;
  }
  if(KEY&KEY_X2) 
  {
     keystatus |= KEY_X2;
  }
  else 
  {
     keystatus &= ~KEY_X2;
  }

  return keystatus;
}