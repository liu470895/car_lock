#ifndef APP_HMC5883_H
#define APP_HMC5883_H

#include "OSAL.h"

#define SAMPLE_TIMES        5//10

typedef struct
{
  uint16 x;
  uint16 y;
  uint16 z;
}XYZ_DataN_t;

typedef struct
{
  uint16 x;
  uint16 y;
  uint16 z;
}XYZ_DataP_t;

void Hmc5883_Set_Mode(uint8 mode);
uint8 Hmc5883_Get_Status(void);
bool is_Hmc5883_Lock(void);
bool is_Hmc5883_Ready_Data(void);
uint8 Hmc5883_Read_XYZ_Axis_data(XYZ_DataN_t *xyz);
uint8  Hmc5883_Get_Vector(XYZ_DataN_t *Nxyz);
#endif//APP_HMC5883_H