#include "app_motor.h"
#include "app_keyscan.h"
#include "hal_board_cfg.h"
#include "app_keyscan.h"

void dir_down(void)
{
    HAL_MOTOR_DOWN();
}

void up(void)
{
   HAL_MOTOR_ON();	
   HAL_MOTOR_UP();
}

void down(void)
{
    HAL_MOTOR_ON();	
    HAL_MOTOR_DOWN();           
}

void stop(void)
{
    HAL_MOTOR_OFF();
    HAL_MOTOR_DOWN();
}

uint8 motorctrol( uint8 fuction ,uint8 keystatus,uint8 motor_target)
{    
  uint8  moterstate;
  if(fuction==Enable)
  {
      if( keystatus != motor_target ) 
      {
        switch(motor_target)
        {
          case carlock0degree: 
               down();
               moterstate = Runing;
               break;
          case carlock90degree: 
                switch( keystatus )
                { 
                  case carlock90degree:    stop(); moterstate = Stoped;  break;
                  case carlock90_180state: down(); moterstate = Runing;  break;
                  default: up();moterstate = Runing; break;
                } 
                break;
          case carlockidle: 
               stop();
               moterstate = Stoped;  
               break;               
          default:break;
        }
      }
      else
      {
        stop();
        moterstate = Stoped;  
      }
  }
  else
  {
        stop();
        moterstate = Stoped;  
  }
  return moterstate;
}