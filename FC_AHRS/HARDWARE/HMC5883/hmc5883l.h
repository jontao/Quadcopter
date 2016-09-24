#ifndef _HMC5883_H
#define _HMC5883_H

#include "sys.h"
#include "I2C.h"
//#define	SlaveAddress   0x3C	  //定义器件在IIC总线中的从地址
#define	HMC_ADDR   0x3C>>1
void read_hmc5883(void);
extern char BufData[32];
extern void Init_HMC5883L(void);
extern void Multiple_Read_HMC5883L(void);
extern unsigned char BUF[8];
float Read_HMC5883L(void);
void mpu_get_compass_reg(signed short *mag);
#endif
