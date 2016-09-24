#include "mpu6050.h"
#include "delay.h"
#include "I2C.h"
#include "math.h"
//**************************************
//init MPU6050
//**************************************
void InitMPU6050()
{
  u8 pbuffer;
  I2C1_Init();	
  I2C1SdData(MPU_ADDR,USER_CTRL,0x00);
	 I2C1SdData(MPU_ADDR,BYPASS,0x02);
//  I2C1SdData(MPU_ADDR,PWR_MGMT_1,  0x80);	
//	 I2C1SdData(MPU_ADDR,PWR_MGMT_1,  0x00);
	 I2C1SdData(MPU_ADDR,PWR_MGMT_1,  0x00);//01

	 I2C1SdData(MPU_ADDR,SMPLRT_DIV,  0x02);//07
	 I2C1SdData(MPU_ADDR,MPUCONFIG,   0x02);
	 I2C1SdData(MPU_ADDR,GYRO_CONFIG, 0x18);//2000    +-500¶È/s//01
	 I2C1SdData(MPU_ADDR,ACCEL_CONFIG,0x11);//8G   +-4G//08
	 I2C1RdData(MPU_ADDR,WHO_AM_I,&pbuffer);
}
//**************************************
//put package 
//**************************************
short GetData(uchar REG_Address)
{
	u8 H,L;
	I2C1RdData(MPU_ADDR,REG_Address,&H);
	I2C1RdData(MPU_ADDR,REG_Address+1,&L);
	return (H<<8)+L; 
}

