#ifndef HAL_HMC5883_H
#define HAL_HMC5883_H

#include "hal_types.h"

#define HMC5883_ADDR    0x3C            /*low 7 bit addr*/
#define HMC5883_ADDR_W  0x3C          /*low 7 bit addr*/
#define HMC5883_ADDR_R  0x3D          /*low 7 bit addr*/
#define HMC5883_DATA_START_ADDR  0x03  /*HMC5883_WHO_AM_IADDR = HMC5883_ADDR << 1 */

//register 
#define CONFIG_A_REG        0x00  /*set data rate*/
#define CONFIG_B_REG        0x01  /*set device gain*/
#define MODE_REG            0x02  /*data out mode*/
#define DATA_OUT_X_MSB_REG  0x03  /*X axis data high byte*/
#define DATA_OUT_X_LSB_REG  0x04  /*X axis data lower byte*/
#define DATA_OUT_Z_MSB_REG  0x05  /*Z axis data high byte*/
#define DATA_OUT_Z_LSB_REG  0x06  /*Z axis data lower byte*/
#define DATA_OUT_Y_MSB_REG  0x07  /*Y axis data high byte*/
#define DATA_OUT_Y_LSB_REG  0x08  /*Y axis data lower byte*/
#define STATUS_REG          0x09  /*show the devive status*/
#define ID_A_REG            0x0A  /*id for the device*/
#define ID_B_REG            0x0B  /*id for the device*/
#define ID_C_REG            0x0C  /*id for the device*/

//register STATUS_REG_RDY
#define STATUS_REG_RDY      0x01  /*id for the device*/

void Hmc5883_Init(uint8 dev_addr );
bool Hmc5883_ReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes);
bool Hmc5883_Multiple_ReadReg(uint8 addr, uint8 *pBuf, uint8 nBytes);
bool Hmc5883_WriteReg(uint8 addr, uint8 *pBuf, uint8 nBytes);

#endif//HAL_HMC5883_H