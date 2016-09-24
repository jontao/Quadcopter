#ifndef _I2CSIM_H
#define _I2CSIM_H
#include "sys.h"
#define I2C_SCL       PBout(8)
#define I2C_SDA       PBout(9)
#define I2C_SDA_READ  PBin(9)
#define I2C_SDAin              \
{ \
  GPIOB->MODER &= 0xFFF3FFFF; \
}
#define I2C_SDAout             \
{ \
  GPIOB->MODER &= 0xFFF3FFFF; \
	 GPIOB->MODER |= 0x00040000; \
}
#endif
void Single_WriteI2C(u8 salve_id,u8 REG_Address,u8 REG_data);
void Single_ReadI2C(u8 salve_id,u8 REG_Address,u8*REG_data);
void I2C_Init(void);

