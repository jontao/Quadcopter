#include "delay.h"
#include "hmc5883l.h"
#include  <math.h>
#include "mpu6050.h"

//******************************************************
//
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
//
//******************************************************
u8 BUF[8];
void Multiple_Read_HMC5883L(void)
{  
	i2cread(HMC_ADDR, 0x03, 6,BUF);
}

//初始化HMC5883，根据需要请参考pdf进行修改****  
void Init_HMC5883L()
{
   I2C1SdData(HMC_ADDR,0x02,0x00);  
}
float Read_HMC5883L()
{    
	   short x,y;//,z
	   float angle;
//	   I2C1SdData(MPU_ADDR,USER_CTRL,0x00);
//	   I2C1SdData(MPU_ADDR,BYPASS,0x02);
     Multiple_Read_HMC5883L();
		 x=BUF[0] << 8 | BUF[1]; //Combine MSB and LSB of X Data output register
		// z=BUF[2] << 8 | BUF[3]; //Combine MSB and LSB of Z Data output register
		 y=BUF[4] << 8 | BUF[5]; //Combine MSB and LSB of Y Data output register
		 angle= atan2((double)y,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
		 angle*=1; 
	   return angle;
}

void mpu_get_compass_reg(signed short *mag)
{    
   Multiple_Read_HMC5883L();
		 mag[0]=BUF[0] << 8 | BUF[1]; //Combine MSB and LSB of X Data output register
		 mag[1]=BUF[2] << 8 | BUF[3]; //Combine MSB and LSB of Z Data output register
		 mag[2]=BUF[4] << 8 | BUF[5]; //Combine MSB and LSB of Y Data output register
}

