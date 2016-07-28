#include "app_hmc5883.h"
#include "npi.h"
#include "stdio.h" 
#include "hal_hmc5883.h"

void Hmc5883_Set_Mode(uint8 mode)
{
  Hmc5883_WriteReg(MODE_REG, &mode, 1);
}

uint8 Hmc5883_Get_Status(void)
{
  uint8 status;
  
  Hmc5883_ReadReg(STATUS_REG, &status, 1);
  
  return status;
}

bool is_Hmc5883_Lock(void)
{
   uint8 status;
   uint8 ret;
   
   status = Hmc5883_Get_Status();
   if(status & 0x02)
    ret = 1;
   else
     ret = 0;
   
   return ret;
}

bool is_Hmc5883_Ready_Data(void)
{
   uint8 status;
   uint8 ret;
   
   status = Hmc5883_Get_Status();
   if(status & 0x01)
    ret = 1;
   else
     ret = 0;
   
   return ret;
}

//read xyz axis data
uint8 Hmc5883_Read_XYZ_Axis_data(XYZ_DataN_t *Nxyz)
{
  uint8 tmp[6] = {0};
  
  if(Nxyz == NULL){
  
    return 0;
  }
  
  Hmc5883_Multiple_ReadReg(HMC5883_DATA_START_ADDR, tmp, 6);
  Nxyz->x = (tmp[0] << 8) | tmp[1]; 
  Nxyz->z = (tmp[2] << 8) | tmp[3]; 
  Nxyz->y = (tmp[4] << 8) | tmp[5]; 
  
  return 1;
}


uint16  Hmc5883DataTurn( uint16 xyz_data )
{
  if( xyz_data & 0x8000 )           //  如果是负数
  {
     xyz_data -=1;                  // 补码转反码
     xyz_data = ~xyz_data;          // 反码转源码
  }
  else
  {
     xyz_data <<= 1;                // 正数乘以2
  }
  return xyz_data;  
}
//get xyz vector
uint8 Hmc5883_Get_Vector(  XYZ_DataN_t *Nxyz)
{
  uint16   x,y,z;
  uint8    tmp = 0;

  if(Nxyz == NULL ) {return 0;}
   Nxyz->x = 0 ;
   Nxyz->y = 0 ;
   Nxyz->z = 0 ;
  if( Hmc5883_ReadReg(STATUS_REG, &tmp, 1) == 1 )//wait for data ready
  {
     if(tmp & STATUS_REG_RDY)
     {
       Hmc5883_Read_XYZ_Axis_data(Nxyz);
       
       x = Hmc5883DataTurn( Nxyz->x );
       y = Hmc5883DataTurn( Nxyz->y );
       z = Hmc5883DataTurn( Nxyz->z );
       Nxyz->x = x ;
       Nxyz->y = y ;
       Nxyz->z = z ;
       
       return 1;
     }
     else
     {
        return 0;
     }
  }
  else
  {
     return 0;
  }
  
}

