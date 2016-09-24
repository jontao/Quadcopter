#include "jonta_fc.h"
#include "spi.h"
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u16 retry=0;				 
	while((SPI2->SR&1<<1)==0)		  /*waitting for flushing finsh */	
	{
		retry++;
		if(retry>=0XFFFE)return 0; 	/*Timeout and exit*/
	}			  
	SPI2->DR=TxData;	 	  		    /*Send a byte */
	retry=0;
	while((SPI2->SR&1<<0)==0) 		/*After waiting for receiving a byte*/  
	{
		retry++;
		if(retry>=0XFFFE)return 0;	/*Timeout and exit*/
	}	  						    
	return SPI2->DR;          		/*Returns the received data*/			    
}
void SPI2_Init(void)
{	 
    /* Enable the peripherals used to drive the SDC on SPI, and the CS */
	  RCC->APB1ENR|=1<<14;
   RCC->AHB1ENR|=1<<1;
    /* Configure the appropriate pins to be SPI instead of GPIO */
	  GPIOB->AFR[1] &= 0x000FFFFF;
	  GPIOB->AFR[1] |= 0x55500000;
	  GPIOB->MODER  &= 0x03FFFFFF; 
   GPIOB->MODER  |= 0xA8000000;
	  GPIOB->PUPDR  |= 0x54000000;
	  GPIOB->OSPEEDR|= 0xfC000000;
	  /* Configure the SPI port */
	  SPI2->CR1|=0<<10;		/*Full-duplex mode*/	
	  SPI2->CR1|=1<<9; 		/*software cs manage*/
	  SPI2->CR1|=1<<8;  

	  SPI2->CR1|=1<<2; 		/*SPI master*/
	  SPI2->CR1|=0<<11;		/*8bit data	*/
	  //SPI2->CR1|=1<<1; 		/*idle mode SCK is 1 CPOL=1*/
		/*The edge of the data sampling from the second time is start*/
		                    /*CPHA=1,the max speed is 36M.*/
	  SPI2->CR1|=1<<0; 		  
	  
	  SPI2->CR1|=2<<3; 		/*Fsck=Fpclk1/256*/
	  SPI2->CR1|=0<<7; 		/*MSBfirst*/ 
		 SPI2->CRCPR=7;
	  SPI2->CR1&=~(1<<1); 	//空闲模式下SCK为0 CPOL=0
	  SPI2->CR1&=~(1<<0); 	//数据采样从第1个时间边沿开始,CPHA=0  
	  SPI2->CR1|=1<<6; 		//SPI设备使能
		 SPI2_ReadWriteByte(0xFF);		
} 
