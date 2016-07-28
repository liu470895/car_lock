#include "hal_hmc5883.h"
#include "hal_i2c.h"

static uint8 buffer[24] = {0};

//init hmc5883 
void Hmc5883_Init(uint8 dev_addr )
{
  HalI2CInit(dev_addr, i2cClock_123KHZ);
}

bool Hmc5883_ReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
  uint8 i = 0;
  
//#define HMC5883_WHO_AM_I_ADDR    0x3C  /*HMC5883_WHO_AM_IADDR = HMC5883_ADDR << 1 */
//#define HMC5883_DATA_START_ADDR  0x03  /*HMC5883_WHO_AM_IADDR = HMC5883_ADDR << 1 */
  
  /* Send address we're reading from */
  if (HalI2CWrite(HMC5883_ADDR_W,1,&addr) == 1)
  {
    /* Now read data */
    i = HalI2CRead(HMC5883_ADDR_R,nBytes,pBuf);
  }

  return i == nBytes;
}
bool Hmc5883_Multiple_ReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
  uint8 i = 0;

  /* Send address we're reading from */
  if (HalI2CWrite(HMC5883_ADDR_W,1,&addr) == 1)
  {
      /* Now read data */
      i = HalI2CRead(HMC5883_ADDR_R, nBytes, pBuf);
  }
  
  return i == nBytes;
}

bool Hmc5883_WriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes)
{
  uint8 i;
  uint8 *p = buffer;

  /* Copy address and data to local buffer for burst write */
  *p++ = addr;
  for (i = 0; i < nBytes; i++)
  {
    *p++ = *pBuf++;
  }
  nBytes++;

  /* Send address and data */
  i = HalI2CWrite(HMC5883_ADDR_W, nBytes, buffer);

  return (i == nBytes);
}


