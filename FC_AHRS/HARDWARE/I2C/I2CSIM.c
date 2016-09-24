#include "delay.h"
#include "I2CSIM.h"

//**************************************
//I2Cstart
//**************************************
void I2C_Start()
{
    I2C_SDA=1;
    I2C_SCL=1;                   
    delay_us(2);                
    I2C_SDA=0;                   
    delay_us(2);                 
    I2C_SCL=0;                  
}
//**************************************
//I2Cstop
//**************************************
void I2C_Stop()
{
    I2C_SDA=0;                    
    I2C_SCL=1;                    
    delay_us(2);                 
    I2C_SDA=1;                    
    delay_us(2);                
}
//**************************************
//I2C transmit
//ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(u8 ack)
{
    I2C_SDA=ack;                 
    I2C_SCL=1;                    
    delay_us(2);                 
    I2C_SCL=0;                    
    delay_us(2);                
}
//**************************************
//I2C received signal
//**************************************

u8 I2C_RecvACK()
{
	  u8 CY;
	  I2C_SCL = 1;
	  delay_us(2);
    I2C_SDAin;
    delay_us(2);
    CY = I2C_SDA_READ;
	  delay_us(2);
    I2C_SDAout;
    I2C_SCL=0;                
    delay_us(2);
    return CY;
}

//**************************************
//I2C transmit a byte data
//**************************************
u8 I2C_SendByte(u8 dat)
{
    u8 i;  
    for (i=0; i<8; i++)         
    {
			I2C_SDA =((dat<<i)&0x80)>>7; 
		  delay_us(2); 				 
		  I2C_SCL=1;                
		  delay_us(2);                          
		  I2C_SCL=0;             
    }
    return I2C_RecvACK();
}

//**************************************
// I2C received a byte data
//**************************************
u8 I2C_RecvByte()
{
    u8 i;
    u8 dat = 0;
    I2C_SDA=1; 
    delay_us(2);
	  I2C_SDAin;
    for (i=0; i<8; i++)         
    {
			  delay_us(2);
        dat <<= 1;
        I2C_SCL=1;                
        delay_us(2); 			
        dat |= I2C_SDA_READ;                          
        I2C_SCL=0;               
        delay_us(2);          
    }
		I2C_SDAout;
    return dat;
}
//**************************************
//I2C write a byte data
//**************************************
void i2cWriteBuffer(u8 salve_id,u8 REG_Address,u8 len,u8 *data)
{
	   int i;
    I2C_Start();                
    I2C_SendByte(salve_id);
    if (I2C_RecvACK()) 
				{
       I2C_Stop();
				}
    I2C_SendByte(REG_Address);
				I2C_RecvACK();
				    for (i = 0; i < len; i++) {
        I2C_SendByte(data[i]);
        if (I2C_RecvACK()) {
            I2C_Stop();
        } 
    I2C_Stop();   
}
								

void i2cWrite(u8 salve_id,u8 REG_Address,u8 data)
{
	   int i;
    I2C_Start();                
    I2C_SendByte(salve_id);
    if (I2C_RecvACK()) 
				{
       I2C_Stop();
				}
    I2C_SendByte(REG_Address);
				I2C_RecvACK();
    I2C_SendByte(data);
    I2C_RecvACK();

    I2C_Stop();   
}
//**************************************
//I2C read a byte
//**************************************
void i2cRead(u8 salve_id,u8 REG_Address,uint8_t len,u8*buf)
{

		I2C_Start();                   
		I2C_SendByte(salve_id);
    if (I2C_RecvACK()) 
				{
       I2C_Stop();
				}	
		I2C_SendByte(REG_Address);     
		I2C_Start();                   
		I2C_SendByte(salve_id+1); 
		*REG_data=I2C_RecvByte();       
		I2C_RecvACK();
				    while (len) {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NoAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
		I2C_Stop();                   

}
void I2C_Init()
{
		RCC->AHB1ENR |= 1<<1; //使能GPIOB时钟
  GPIOB->MODER  &= 0xFF0FFFFF; //设置PB0,1 ouput
  GPIOB->MODER  |= 0x00500000; 
  GPIOB->OSPEEDR&= 0xFF0FFFFF; //PB0,1 速度100m
  GPIOB->OSPEEDR|= 0x00F00000;
  GPIOB->OTYPER &= 0xf3ff;
  GPIOB->OTYPER |= 0x0C00;
}
