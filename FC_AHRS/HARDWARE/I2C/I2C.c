#include "delay.h"
#include "I2C.h"
#include "stdbool.h"

#define TRUE  0
#define FALSE -1
//0表示写
#define	I2C_Direction_Transmitter   0
//１表示读
#define	I2C_Direction_Receiver      1	 
//**************************************
//I2Cstart
//**************************************
void I2C1_Start()
{
   I2C1 -> CR1 |=   1<<8;      //I2C1产生起始条件                
}

//**************************************
//I2Cstop
//**************************************
void I2C1_Stop()
{
   I2C1 -> CR1 |=   1<<9;      //I2C1产生停止条件               
}
//**************************************
//I2C Write a byte data
//**************************************
 
void  I2C1_Write(u8 data) 
{ 
   I2C1 -> DR = data; 
} 
//**************************************
// I2C received a byte data
//**************************************

u8  I2C1_Read() 
{ 
    while(!(I2C1 -> SR1 & (1<<6)));      //接收到数据标志位 
    return I2C1 -> DR; 
} 


void I2C1SdData(u8 slaveID, u8 Addr, u8 data)
{
		while((I2C1->SR2 & (1 << 1)));//总线忙，等待？
		I2C1_Start(); 
		while(!(I2C1->SR1 & (1 << 0)));//SB位置位说明已产生启示条件
		I2C1_Write(slaveID<< 1 | I2C_Direction_Transmitter);//将从机地址写入数据寄存器   
		while(!(I2C1->SR1 & (1 << 1)));//判断addr是否置位（当从机返回ACK时由硬件置位）
		I2C1->SR2;//读SR1、2会将addr标志清零  
		I2C1_Write(Addr & 0xff);//发送地址
		while(!(I2C1->SR1&0x0004));//当前字节即将被发送且数据寄存器为空时
		while(!(I2C1->SR1 & (1 << 7)));//发送数据寄存器为空
		I2C1_Write(data);
		while(!(I2C1->SR1 & (1 << 2)));// !(I2C1->SR1 & (1 << 7))判断BTF是否置位
		I2C1_Stop();//产生停止位
}


//**************************************
//I2C read a byte
//**************************************
void I2C1RdData(u8 slaveID, u8 Addr, u8* pbuffer)
{
		while(I2C1->SR2 & (1 << 1));//总线忙，等待？
		I2C1_Start();//产生起始条件
		while(!(I2C1->SR1 & (1 << 0)));//SB位置位说明已产生启示条件
		I2C1_Write(slaveID<< 1 | I2C_Direction_Transmitter);//将从机地址写入数据寄存器    
		while(!(I2C1->SR1 & (1 << 1)));//判断addr是否置位（当从机返回ACK时由 硬件置位）
		I2C1->SR2;//读SR1、2会将addr标志清零
		I2C1_Write(Addr & 0xff);//发送fm24cl16的字地址（每页的最大256个字节）
		while(!(I2C1->SR1& (1 << 2)));//当前字节即将被发送且数据寄存器为空时BTF置位 
		I2C1->CR1 |= 1 << 8;//重新产生起始条件 
		while(!(I2C1->SR1 & (1 << 0)));//SB位置位说明已产生启示条件
		I2C1_Write(slaveID << 1 | I2C_Direction_Receiver);//将从机地址写入数据寄存器
		while(!(I2C1->SR1 & (1 << 1)));//判断addr是否置位（当从机返回ACK时由硬件置位）
		I2C1->SR2;//读SR1、2会将addr标志清零
	 I2C1->CR1 &= ~(1 << 10);//清除ACK位
		*pbuffer = I2C1_Read();      
		//*temp = I2C1->DR;//读最后一位数据
		I2C1_Stop();//产生停止条件
}




bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	 int i;
	 for (i = 0; i < len; i++) 
	 {
	   I2C1SdData(addr, reg++, data[i]);
		}
	 return true;
}
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
	 if(i2cWriteBuffer(addr,reg,len,data))
	 {
		 return TRUE;
	 }
	 else
	 {
	  return FALSE;
	 }
}
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	  while (len) {
        I2C1RdData(addr, reg++, buf++);
        len--;
    }
	  return true;
}

int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
	  	if(i2cRead(addr,reg,len,buf))
	  {
	  	return TRUE;
	   }
	  else
	  {
	  	return FALSE;
	  }
}


bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
	  I2C1SdData(addr, reg, data);
	  return true;
}






void I2C1MulRdData(uint8_t slaveID, uint8_t Addr,  uint8_t num,uint8_t* pbuffer)
{
		uint8_t* temp = pbuffer;
	
		while(I2C1->SR2 & (1 << 1));//总线忙，等待？
		I2C1->CR1 |= 1 << 8;//产生起始条件
		
		while(!(I2C1->SR1 & (1 << 0)));//SB位置位说明已产生启示条件
		I2C1->DR = slaveID;//将从机地址写入数据寄存器
		
		while(!(I2C1->SR1 & (1 << 1))){;};//判断addr是否置位（当从机返回ACK时由 硬件置位）
		I2C1->SR2;//读SR1、2会将addr标志清零
		
		I2C1->DR = Addr;//发送fm24cl16的字地址（每页的最大256个字节）
		while(!(I2C1->SR1& (1 << 2)));//当前字节即将被发送且数据寄存器为空时BTF置位
		
		I2C1->CR1 |= 1 << 8;//重新产生起始条件
		
		while(!(I2C1->SR1 & (1 << 0)));//SB位置位说明已产生启示条件
		I2C1->DR =slaveID+1 ;//将从机地址写入数据寄存器
		
		while(!(I2C1->SR1 & (1 << 1)));//判断addr是否置位（当从机返回ACK时由硬件置位）

			I2C1->SR2;//读SR1、2会将addr标志清零
		//delay_us(100);
			if(num < 2)
			{
						I2C1->CR1 &= ~(1 << 10);//清除ACK位
		    (void)I2C1->SR2;
		    I2C1->CR1 |= 1 << 9;//产生停止条件
				  	while(!(I2C1->SR1 & (1 << 6)));//判断RxNE是否置位
						*temp = I2C1->DR;
				  while(I2C1->CR1 & I2C_CR1_STOP);
				   I2C1->CR1 |= 1 << 10;
			}
			else{
		     for(; num > 0; num--)
		      {
          if(num==1){	I2C1->CR1 &= ~(1 << 10);}	//清除ACK位
					    	//while(!(I2C1->SR1 & (1 << 6)));//判断RxNE是否置位
						    *temp++ = I2C1->DR;					
								}	
								
								I2C1->CR1 |= 1 << 9;//产生停止条件											
	    	}   
  
}
//**************************************
//I2Cinit
//**************************************
void I2C1_Init()
{
		 RCC->AHB1ENR |= 1<<1;        //使能GPIOB时钟
	  RCC->APB1ENR |= 1<<21;
	  GPIOB->AFR[1]&= 0xFFFFFF00;
	  GPIOB->AFR[1]|= 0x00000044;
	  GPIOB->MODER &= 0xFFF0FFFF;  //设置PB0,1 ouput
	  GPIOB->MODER |= 0x000A0000;
   GPIOB->OTYPER|= 0x0300;
	  GPIOB->PUPDR &= 0xFFF0FFFF;
	  RCC->APB1RSTR  |= 1<<21;           //复位I2C1
	  RCC->APB1RSTR  &= ~(1<<21);            //复位结束I2C1
		 I2C1->CR1 |= 1 << 15;//复位
		 I2C1->CR1 &= ~(1 << 15);
		 I2C1->CR1 |= 0 << 0;
	  I2C1->CR1 |= 0 << 1;
	  I2C1->CR1 |= 1 << 3;
	  I2C1->CR2 |= 36 << 0;//I2C1输入时钟频率设置为40MHz
		 I2C1->CCR |= 1 << 15;//设置成快速模式
		 I2C1->CCR |= 0 << 14;//占空比设置1:2
		 I2C1->CCR |= 50 << 0;//时钟控制分频系数设置为3（400KHz = 2.5us = （16+9）*CCR*Tpclk1）
		 I2C1->TRISE |= 10 << 0;//设置主模式时的最大上升时间（标准模式为1000ns，快速为300ns，超快为120ns）
		 I2C1->CR1 |= 1 << 10;//应答使能（ACK）
		 I2C1->OAR1 |= 0<<15;//寻址模式   1 响应10位地址  0  响应7位地址
		 I2C1->OAR1 |= 1<<14;//必须始终由软件保持为 1
		 I2C1->OAR1 |=  0x30 <<1 ;            //设置接口地址的 7~1位	
			I2C1->CR1 |= 1 << 10;
		 I2C1->CR1 |= 1 << 0;//启用I2C模块

}
