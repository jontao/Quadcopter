#include "delay.h"
#include "I2C.h"
#include "stdbool.h"

#define TRUE  0
#define FALSE -1
//0��ʾд
#define	I2C_Direction_Transmitter   0
//����ʾ��
#define	I2C_Direction_Receiver      1	 
//**************************************
//I2Cstart
//**************************************
void I2C1_Start()
{
   I2C1 -> CR1 |=   1<<8;      //I2C1������ʼ����                
}

//**************************************
//I2Cstop
//**************************************
void I2C1_Stop()
{
   I2C1 -> CR1 |=   1<<9;      //I2C1����ֹͣ����               
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
    while(!(I2C1 -> SR1 & (1<<6)));      //���յ����ݱ�־λ 
    return I2C1 -> DR; 
} 


void I2C1SdData(u8 slaveID, u8 Addr, u8 data)
{
		while((I2C1->SR2 & (1 << 1)));//����æ���ȴ���
		I2C1_Start(); 
		while(!(I2C1->SR1 & (1 << 0)));//SBλ��λ˵���Ѳ�����ʾ����
		I2C1_Write(slaveID<< 1 | I2C_Direction_Transmitter);//���ӻ���ַд�����ݼĴ���   
		while(!(I2C1->SR1 & (1 << 1)));//�ж�addr�Ƿ���λ�����ӻ�����ACKʱ��Ӳ����λ��
		I2C1->SR2;//��SR1��2�Ὣaddr��־����  
		I2C1_Write(Addr & 0xff);//���͵�ַ
		while(!(I2C1->SR1&0x0004));//��ǰ�ֽڼ��������������ݼĴ���Ϊ��ʱ
		while(!(I2C1->SR1 & (1 << 7)));//�������ݼĴ���Ϊ��
		I2C1_Write(data);
		while(!(I2C1->SR1 & (1 << 2)));// !(I2C1->SR1 & (1 << 7))�ж�BTF�Ƿ���λ
		I2C1_Stop();//����ֹͣλ
}


//**************************************
//I2C read a byte
//**************************************
void I2C1RdData(u8 slaveID, u8 Addr, u8* pbuffer)
{
		while(I2C1->SR2 & (1 << 1));//����æ���ȴ���
		I2C1_Start();//������ʼ����
		while(!(I2C1->SR1 & (1 << 0)));//SBλ��λ˵���Ѳ�����ʾ����
		I2C1_Write(slaveID<< 1 | I2C_Direction_Transmitter);//���ӻ���ַд�����ݼĴ���    
		while(!(I2C1->SR1 & (1 << 1)));//�ж�addr�Ƿ���λ�����ӻ�����ACKʱ�� Ӳ����λ��
		I2C1->SR2;//��SR1��2�Ὣaddr��־����
		I2C1_Write(Addr & 0xff);//����fm24cl16���ֵ�ַ��ÿҳ�����256���ֽڣ�
		while(!(I2C1->SR1& (1 << 2)));//��ǰ�ֽڼ��������������ݼĴ���Ϊ��ʱBTF��λ 
		I2C1->CR1 |= 1 << 8;//���²�����ʼ���� 
		while(!(I2C1->SR1 & (1 << 0)));//SBλ��λ˵���Ѳ�����ʾ����
		I2C1_Write(slaveID << 1 | I2C_Direction_Receiver);//���ӻ���ַд�����ݼĴ���
		while(!(I2C1->SR1 & (1 << 1)));//�ж�addr�Ƿ���λ�����ӻ�����ACKʱ��Ӳ����λ��
		I2C1->SR2;//��SR1��2�Ὣaddr��־����
	 I2C1->CR1 &= ~(1 << 10);//���ACKλ
		*pbuffer = I2C1_Read();      
		//*temp = I2C1->DR;//�����һλ����
		I2C1_Stop();//����ֹͣ����
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
	
		while(I2C1->SR2 & (1 << 1));//����æ���ȴ���
		I2C1->CR1 |= 1 << 8;//������ʼ����
		
		while(!(I2C1->SR1 & (1 << 0)));//SBλ��λ˵���Ѳ�����ʾ����
		I2C1->DR = slaveID;//���ӻ���ַд�����ݼĴ���
		
		while(!(I2C1->SR1 & (1 << 1))){;};//�ж�addr�Ƿ���λ�����ӻ�����ACKʱ�� Ӳ����λ��
		I2C1->SR2;//��SR1��2�Ὣaddr��־����
		
		I2C1->DR = Addr;//����fm24cl16���ֵ�ַ��ÿҳ�����256���ֽڣ�
		while(!(I2C1->SR1& (1 << 2)));//��ǰ�ֽڼ��������������ݼĴ���Ϊ��ʱBTF��λ
		
		I2C1->CR1 |= 1 << 8;//���²�����ʼ����
		
		while(!(I2C1->SR1 & (1 << 0)));//SBλ��λ˵���Ѳ�����ʾ����
		I2C1->DR =slaveID+1 ;//���ӻ���ַд�����ݼĴ���
		
		while(!(I2C1->SR1 & (1 << 1)));//�ж�addr�Ƿ���λ�����ӻ�����ACKʱ��Ӳ����λ��

			I2C1->SR2;//��SR1��2�Ὣaddr��־����
		//delay_us(100);
			if(num < 2)
			{
						I2C1->CR1 &= ~(1 << 10);//���ACKλ
		    (void)I2C1->SR2;
		    I2C1->CR1 |= 1 << 9;//����ֹͣ����
				  	while(!(I2C1->SR1 & (1 << 6)));//�ж�RxNE�Ƿ���λ
						*temp = I2C1->DR;
				  while(I2C1->CR1 & I2C_CR1_STOP);
				   I2C1->CR1 |= 1 << 10;
			}
			else{
		     for(; num > 0; num--)
		      {
          if(num==1){	I2C1->CR1 &= ~(1 << 10);}	//���ACKλ
					    	//while(!(I2C1->SR1 & (1 << 6)));//�ж�RxNE�Ƿ���λ
						    *temp++ = I2C1->DR;					
								}	
								
								I2C1->CR1 |= 1 << 9;//����ֹͣ����											
	    	}   
  
}
//**************************************
//I2Cinit
//**************************************
void I2C1_Init()
{
		 RCC->AHB1ENR |= 1<<1;        //ʹ��GPIOBʱ��
	  RCC->APB1ENR |= 1<<21;
	  GPIOB->AFR[1]&= 0xFFFFFF00;
	  GPIOB->AFR[1]|= 0x00000044;
	  GPIOB->MODER &= 0xFFF0FFFF;  //����PB0,1 ouput
	  GPIOB->MODER |= 0x000A0000;
   GPIOB->OTYPER|= 0x0300;
	  GPIOB->PUPDR &= 0xFFF0FFFF;
	  RCC->APB1RSTR  |= 1<<21;           //��λI2C1
	  RCC->APB1RSTR  &= ~(1<<21);            //��λ����I2C1
		 I2C1->CR1 |= 1 << 15;//��λ
		 I2C1->CR1 &= ~(1 << 15);
		 I2C1->CR1 |= 0 << 0;
	  I2C1->CR1 |= 0 << 1;
	  I2C1->CR1 |= 1 << 3;
	  I2C1->CR2 |= 36 << 0;//I2C1����ʱ��Ƶ������Ϊ40MHz
		 I2C1->CCR |= 1 << 15;//���óɿ���ģʽ
		 I2C1->CCR |= 0 << 14;//ռ�ձ�����1:2
		 I2C1->CCR |= 50 << 0;//ʱ�ӿ��Ʒ�Ƶϵ������Ϊ3��400KHz = 2.5us = ��16+9��*CCR*Tpclk1��
		 I2C1->TRISE |= 10 << 0;//������ģʽʱ���������ʱ�䣨��׼ģʽΪ1000ns������Ϊ300ns������Ϊ120ns��
		 I2C1->CR1 |= 1 << 10;//Ӧ��ʹ�ܣ�ACK��
		 I2C1->OAR1 |= 0<<15;//Ѱַģʽ   1 ��Ӧ10λ��ַ  0  ��Ӧ7λ��ַ
		 I2C1->OAR1 |= 1<<14;//����ʼ�����������Ϊ 1
		 I2C1->OAR1 |=  0x30 <<1 ;            //���ýӿڵ�ַ�� 7~1λ	
			I2C1->CR1 |= 1 << 10;
		 I2C1->CR1 |= 1 << 0;//����I2Cģ��

}
