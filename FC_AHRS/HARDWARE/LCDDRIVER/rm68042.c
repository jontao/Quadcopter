/************************************************************************************
** Function Name : rm68042 lcd driver                                              **
** Description   : .                                                               **
** Arguments     :                                                                 **
=====================================================================================
** Author        : Jonta                                                           **
** Data          : 2013-9-2                                                        **
** Notes         :                                                                 **
************************************************************************************/

#include "rm68042.h"
#include "sys.h"
#include "delay.h"
#include "string.h"
#include "ascii16.h"
void LCD_WR_REG(u16 reg)
{
	Lcd->LCD_REG=reg;
}

void LCD_WR_DATA(u16 data)
{										    	   
	Lcd->LCD_RAM=data;		 
}

u16 LCD_RD_DATA(void)
{										    	   
	return Lcd->LCD_RAM;		 
}					      	 
void opt_delay(u8 i)
{
	while(i--);
}

void LCD_rm68042_Init(void)
{ 	
	u32 usController;
	u32 i=0x1fffff;	
	RCC->AHB3ENR|=1;         //使能FSMC时钟	  
  RCC->AHB1ENR|=1;         //使能PORTA时钟
	RCC->AHB1ENR|=1<<3;      //使能PORTD时钟
	RCC->AHB1ENR|=1<<4;      //使能PORTE时钟
	RCC->AHB1ENR|=1<<5;      //使能PORTF时钟
  RCC->AHB1ENR|=1<<6;      //使能PORTG时钟

	/*************lcd reset pin***********/
  GPIOA->MODER  &= 0xFFFFFFF0; //设置PA0,1 ouput
  GPIOA->MODER  |= 0x00000005; 
  GPIOA->OSPEEDR&= 0xffFFFFF0; //PA0,1 速度100m
  GPIOA->OSPEEDR|= 0x0000000F;
	/*************lcd CS pin***********/
	GPIOG->AFR[1] &= 0xfffff0ff;
	GPIOG->AFR[1] |= 0x00000C00;
  GPIOG->MODER  &= 0xFFCFFFFF; //设置PG10 ouput
  GPIOG->MODER  |= 0x00200000; 
  GPIOG->OSPEEDR&= 0xFFCFFFFF;
	GPIOG->OSPEEDR|= 0x00300000;

	/*****************FSMC D0~D15 NOE NWE*************/
	GPIOD->AFR[0] &= 0xff00ff00;
	GPIOD->AFR[0] |= 0x00CC00CC;
	GPIOD->AFR[1] &= 0x00fff000;
	GPIOD->AFR[1] |= 0xCC000CCC;
	
	GPIOD->MODER  &= 0x0FC0F0F0; //设置PD0,1,4,5,8,9,10,14,15
  GPIOD->MODER  |= 0xA02A0A0A; 
  GPIOD->OSPEEDR&= 0x0FC0F0F0; // 速度100m
  GPIOD->OSPEEDR|= 0xF03F0F0f;
	//GPIOD->OTYPER &= 0xC7CC;
//GPIOD->OTYPER |= ~0xC7CC;
	//GPIOD->PUPDR  &= 0x0FC0F0F0; 
//GPIOD->PUPDR  |= 0x00000000;

	GPIOE->AFR[0] &= 0x0fffffff;
	GPIOE->AFR[0] |= 0xC0000000;
	GPIOE->AFR[1] &= 0x00000000;
  GPIOE->AFR[1] |= 0xCCCCCCCC;
	
	GPIOE->MODER  &= 0x00003FFF; //设置PE7,8,9,10,11,12,13,14,15
  GPIOE->MODER  |= 0xAAAA8000; 
  GPIOE->OSPEEDR&= 0x00003FFF; // 速度100m
  GPIOE->OSPEEDR|= 0xFFFFC000;
	
//   GPIOD->OTYPER &= 0xFFFF007F;
// 	GPIOD->PUPDR  &= 0x00003FFF; 
//GPIOD->PUPDR  |= 0x00000000;
	

	/********************A0****************/
	GPIOF->AFR[0] &= 0xfffffff0;
  GPIOF->AFR[0] |= 0x0000000C;
	GPIOF->MODER  &= 0xFFFFFFFC; //设置A0 ouput
  GPIOF->MODER  |= 0x00000002; 
  GPIOF->OSPEEDR&= 0xffFFFFFC; //PF0 速度100m
  GPIOF->OSPEEDR|= 0x00000003;
	while(i--);    
	FSMC_Bank1->BTCR[4]=0X00000000;//01,23,45,67  BCR+TCR
	FSMC_Bank1->BTCR[5]=0X00000000;
	FSMC_Bank1E->BWTR[4]=0X00000000;
	FSMC_Bank1->BTCR[4]|=1<<12;		//存储器写使能
	FSMC_Bank1->BTCR[4]|=1<<14;		//读写使用不同的时序
	FSMC_Bank1->BTCR[4]|=1<<4; 		//存储器数据宽度为16bit 	    					    
	FSMC_Bank1->BTCR[5]|=0<<28;		//模式A 	 							  	 
	FSMC_Bank1->BTCR[5]|=1<<0; 		//地址建立时间（ADDSET）为2个HCLK 1/36M=27ns	 	 
	FSMC_Bank1->BTCR[5]|=0<<8;    //数据保存时间为16个HCLK	0xf 	 
	FSMC_Bank1E->BWTR[4]|=0<<28; 	//模式A 	 							    
	FSMC_Bank1E->BWTR[4]|=1<<0;		//地址建立时间（ADDSET）为1个HCLK  	 
	FSMC_Bank1E->BWTR[4]|=0<<8; 	//数据保存时间为4个HCLK	
	FSMC_Bank1->BTCR[4]|=1<<0;		//使能BANK1，区域3	
// 	i=0x1fffff;
// 	while(i--);
/********************** Reset LCD************************/
        delay_us(5);
	      LCD_REST=1;
	      delay_us(5);
	      //delay 15us
	      LCD_REST=0;
        LCD_REST=1;
	      //delay 15us to wait for resetting
	      delay_us(5);
	      LCD_REST=0;
	      delay_us(5);
	      LCD_REST=1;
	      //delay 120ms to wait reseted
	      delay_ms(120);
        LCD_WR_REG(0X11);//使能
	
    		delay_ms(20);
				
			 //LCD_WR_REG(0XBF);
			 //LCD_WR_REG(0X2E);
			 //usController=LCD_Read_Data();
       //usController=LCD_Read_Data();
			 //usController=LCD_Read_Data();
			 //usController=LCD_Read_Data();
			 //usController=LCD_Read_Data();
			 //if(usController==68042)
				
    		LCD_WR_REG(0XD0);//VCI1  VCL  VGH  VGL DDVDH VREG1OUT power amplitude setting
    		LCD_WR_DATA(0X07);
    		LCD_WR_DATA(0X42);
    		LCD_WR_DATA(0X1D);
    		LCD_WR_REG(0XD1);//VCOMH VCOM_AC amplitude setting
    		LCD_WR_DATA(0X00);
    		LCD_WR_DATA(0X1a);
    		LCD_WR_DATA(0X09);
    		LCD_WR_REG(0XD2);//Operational Amplifier Circuit Constant Current Adjust , charge pump frequency setting
    		LCD_WR_DATA(0X01);
    		LCD_WR_DATA(0X22);
    		LCD_WR_REG(0XC0);//REV SM GS
    		LCD_WR_DATA(0X10);
    		LCD_WR_DATA(0X3B);
    		LCD_WR_DATA(0X00);
    		LCD_WR_DATA(0X02);
    		LCD_WR_DATA(0X11);
    		LCD_WR_REG(0XC5);// Frame rate setting = 72HZ  when setting 0x03
    		LCD_WR_DATA(0X03);
    	  LCD_WR_REG(0XC8);//Gamma setting
    		LCD_WR_DATA(0X00);
    		LCD_WR_DATA(0X25);
    		LCD_WR_DATA(0X21);
    		LCD_WR_DATA(0X05);
    		LCD_WR_DATA(0X00);
    		LCD_WR_DATA(0X0a);
    		LCD_WR_DATA(0X65);
    		LCD_WR_DATA(0X25);
    		LCD_WR_DATA(0X77);
    		LCD_WR_DATA(0X50);
    		LCD_WR_DATA(0X0f);
    		LCD_WR_DATA(0X00);

    		LCD_WR_REG(0XF8);
    		LCD_WR_DATA(0X01);

    		LCD_WR_REG(0XFE);
    		LCD_WR_DATA(0X00);
    		LCD_WR_DATA(0X02);

    		LCD_WR_REG(0X20);//Exit invert mode

    		LCD_WR_REG(0X36);
     		LCD_WR_DATA(0X2b);/////b3=1,,

     		LCD_WR_REG(0X3A);
    		LCD_WR_DATA(0X55);//16位模式

    		LCD_WR_REG(0X2A);
    		LCD_WR_DATA(0X00);
    		LCD_WR_DATA(0X00);
    		LCD_WR_DATA(0X01);
    		LCD_WR_DATA(0XDF);

    		LCD_WR_REG(0X2B);
    		LCD_WR_DATA(0X00);
    		LCD_WR_DATA(0X00);
    		LCD_WR_DATA(0X01);
    		LCD_WR_DATA(0X3F);
				delay_ms(20);
				LCD_WR_REG(0X29);
    	  LCD_BL=1;
 				LCD_WR_REG(0x2C);
// 			for(;;){
 				for(usController=0;usController<(320*480);usController++)
 				{LCD_WR_DATA(0);}
// 			  for(usController=0;usController<(320*480);usController++)
// 				{LCD_WR_DATA(0Xf800);}
// 			}			        			
 }
/*****************************************************************************/



/************************************************************************************
** Function Name : lcd_backlight_on and  lcd_backlight_on                          **
** Description   : Turns on/off the back light.                                    **
**				   This function turns on/off the back light on the display.       **
** Arguments     :                                                                 **
** Return        : None                                                            **
** Output        :                                                                 **
=====================================================================================
** Author        : Jonta                                                    **
** Data          : 2012-12-3                                                       **
** Notes         :                                                                 **
************************************************************************************/
// void LCD_Backlight_On(void)
// {
//     // assert the signal that turns on the backlight.
// 	LCD_SET_LED(0xff);
// }

// void LCD_Backlight_Off(void)
// {
//     // Deassert the signal that turns off the backlight.
// 	LCD_SET_LED(0x00);
// }
/************************************************************************************/



/************************************************************************************
** Function Name : LCD_Display_On and LCD_Display_Off                              **
** Description   : Flushes any cached drawing operations.                          **
** Arguments     :                                                                 **
** Return        : None                                                            **
** Output        :                                                                 **
=====================================================================================
** Author        : Jonta                                                    **
** Data          : 2012-12-3                                                       **
** Notes         :                                                                 **
************************************************************************************/
void
LCD_Display_Off(void)
{
	LCD_WR_REG(0x28);
}
void
LCD_Display_On(void)
{
	LCD_WR_REG(0x29);
}
/************************************************************************************/

/************************************************************************************/
void LCD_Set_Cursor(u32 lX,u32 lY)
{ 
		LCD_WR_REG(0X2A);
		LCD_WR_DATA(lX>>8);
		LCD_WR_DATA(lX&0xff);
		LCD_WR_DATA(lX>>8);
		LCD_WR_DATA(lX&0xff);

		LCD_WR_REG(0X2B);
		LCD_WR_DATA(lY>>8);
		LCD_WR_DATA(lY&0xff);
		LCD_WR_DATA(lY>>8);
		LCD_WR_DATA(lY&0xff);
}
void LCD_rm68042_Pixel_Draw(u32 lX,u32 lY,u32 color)
{
		LCD_Set_Cursor(lX,lY);
	 // Write the pixel value.
		LCD_WR_REG(0x2C);
		LCD_WR_DATA(color);
}

u32 LCD_BGR_RGB_Convert(u32 ulvalue)
{  
	 return (((ulvalue>>11)&0x1f)|((ulvalue<<11)&0xf800)|(ulvalue&0x7E0));
}

u32 LCD_rm68042_Pixel_Read(u32 lX,u32 lY)
{
		LCD_Set_Cursor(lX,lY);
		LCD_WR_REG(0X2E);
		//LCD_Read_Data();
		//return (LCD_BGR_RGB_Convert(LCD_Read_Data()));
		return 0;
}

/*************************************************************************************/
void LCD_XorPixel(u32 xStart, u32 yStart)
{
	  LCD_Set_Cursor(xStart,yStart);
}
	
void LCD_Draw_HLine(u32 xStart, u32 yStart,  u32 xLong, u32 color) 
{
		LCD_WR_REG(0X2A);
		LCD_WR_DATA(xStart>>8);
		LCD_WR_DATA(xStart&0xff);
		LCD_WR_DATA((xStart+xLong-1)>>8);
		LCD_WR_DATA(((xStart+xLong)&0xff)-1);
		LCD_WR_REG(0X2B);
		LCD_WR_DATA(yStart>>8);
		LCD_WR_DATA(yStart&0xff);
		LCD_WR_DATA(yStart>>8);
		LCD_WR_DATA(yStart&0xff);
		LCD_WR_REG(0X2C);
		for(;xStart<=xLong;xStart++)
		{LCD_WR_DATA(color);}
	
}
void LCD_Draw_VLine(u32 xStart, u32 yStart, u32 yLong,u32 color)
{       
		LCD_WR_REG(0X2B);
		LCD_WR_DATA(xStart>>8);
		LCD_WR_DATA(xStart&0xff);
		LCD_WR_DATA(xStart>>8);
		LCD_WR_DATA(xStart&0xff);
		LCD_WR_REG(0X2B);
		LCD_WR_DATA(yStart>>8);
		LCD_WR_DATA(yStart&0xff);
		LCD_WR_DATA((yStart+yLong-1)>>8);
		LCD_WR_DATA(((yStart+yLong)&0xff)-1);
		LCD_WR_REG(0X2C);
		for(;yStart<=yLong;yStart++)
		{LCD_WR_DATA(color);}
}


void LCD_SetBox(u32 xStart,u32 yStart,u32 xLong,u32 yLong)

{
		LCD_WR_REG(0X2A);
	  
		LCD_WR_DATA(xStart>>8);
		LCD_WR_DATA(xStart&0xff);
		LCD_WR_DATA((xStart+xLong-1)>>8);
		LCD_WR_DATA(((xStart+xLong)&0xff)-1);

		LCD_WR_REG(0X2B);
		LCD_WR_DATA(yStart>>8);
		LCD_WR_DATA(yStart&0xff);
		LCD_WR_DATA((yStart+yLong-1)>>8);
		LCD_WR_DATA(((yStart+yLong)&0xff)-1); 
  
}
void LCD_Fill_Rect(u32 xStart,u32 yStart,u32 xLong,u32 yLong,u32 color )
{       
	  u32 count;
		LCD_SetBox(xStart,yStart,xLong,yLong);
		LCD_WR_REG(0X2C);
		count=xLong * yLong;
		while(count--)
		{LCD_WR_DATA(color);}
}

void Chinese_Display(u8*pt,u16 xStart, u16 yStart,u16 FontColor)

{
	  u32 i,j;
    u8 dat;
    u8 Xlow,Xhigh,Ylow,Yhigh; 
    u8*p;
	  p=pt;
    Xlow = xStart ;
    Xhigh = (xStart >> 8);
    Ylow = yStart;
    Yhigh=(yStart >> 8);
	  for(j = 0;j < 32;j++)
    { 
			dat = *p++;
      for(i = 0; i < 8; i++)
      { 
				LCD_WR_REG(0X2A);
				if(((j % 2 == 1) && ((Xlow + i + 8) > 0xff)) || ((Xlow + i) > 0xff))
				    {LCD_WR_DATA(0x01);}
				else{LCD_WR_DATA(Xhigh);}
		     LCD_WR_DATA(j % 2 == 1 ? Xlow + i + 8 : Xlow + i);
		     LCD_WR_DATA(0X01);
		     LCD_WR_DATA(0XDF);
		 if(!j)
		  {   LCD_WR_REG(0X2B); 			
          LCD_WR_DATA(Yhigh);
					LCD_WR_DATA(Ylow);
					LCD_WR_DATA(0X01);
					LCD_WR_DATA(0X3F);
		  }
         if(dat & 0x80)
        {
        	 LCD_WR_REG(0x2C);
	         LCD_WR_DATA(FontColor);
        }
      dat <<= 1;
      }

    if(j%2==1)
	    {   
          LCD_WR_REG(0X2B);
          if((Ylow + j/2) > 0xff)
				    {LCD_WR_DATA(0x01);}
				  else{LCD_WR_DATA(Yhigh);}				
       		LCD_WR_DATA(Ylow+j/2 );
       		LCD_WR_DATA(0X01);
       		LCD_WR_DATA(0X3F);
      }
    }
}

void English_Display(u8*pt, u16 xStart,u16 yStart,u16 FontColor)
{
	  u32 i,j;
    u8 dat;
    u8 Xlow,Xhigh,Ylow,Yhigh;
    u8*p;
	  p=pt;
    Xlow=xStart;
    Xhigh=(xStart >> 8);
    Ylow=yStart;
    Yhigh=(yStart >> 8);
	  for(j = 0;j < 16;j++)
    { dat = *p++;
      for(i = 0; i < 8; i++)
      {
    	  LCD_WR_REG(0X2A);
				if((Xlow + i) > 0xff)
				    {LCD_WR_DATA(0x01);}
				else{LCD_WR_DATA(Xhigh);}
		    LCD_WR_DATA(Xlow + i);
		    LCD_WR_DATA(0X01);
		    LCD_WR_DATA(0XDF);
				
				LCD_WR_REG(0x2B);
				if((Ylow + j) > 0xff)
					{LCD_WR_DATA(0x01);}
				else{LCD_WR_DATA(Yhigh);}
				  LCD_WR_DATA(Ylow + j );
				  LCD_WR_DATA(0X01);
				  LCD_WR_DATA(0X3F); 
        if(dat & 0x80)
        {
        	LCD_WR_REG(0x2C);
	        LCD_WR_DATA(FontColor);
        }
      dat <<= 1;
      }
    }
}
void LCD_Picture_Display(u32 xStart,u32 yStart, u32 xLong,u32 yLong,uc8 * pPicture)
{
	u32 i; 
	u16 pixel;
  LCD_SetBox(xStart,yStart,xLong,yLong);
	LCD_WR_REG(0x2C);
	for (i = 0; i <(xLong*yLong); i++)
	    {
		    pixel = *pPicture++;
		    pixel <<= 8;
	      pixel += *pPicture;
		    pPicture++;
		    LCD_WR_DATA(pixel);
	     }
		 
}

u16 LcdDisStarX = 5;
u16 LcdDisStarY = 5;

void Display_GBK_16X16(u8* cBuffer)
{
	 Chinese_Display(cBuffer,LcdDisStarX,LcdDisStarY,FontC);
	 if((LcdDisStarX += 12) > (DisHoriMax - 23))
	 { 
			 LcdDisStarX = 5;  
			 if((LcdDisStarY += 12) > DisVertMax-23 )
				 {
					LcdDisStarY = 5;}
					else{LcdDisStarY += 12;}
	 }else{
				LcdDisStarX += 12;}
}

void Display_ASC_8X16(u8 *cBuffer)
{
   English_Display(cBuffer,LcdDisStarX,LcdDisStarY,FontC); 
	 if((LcdDisStarX += 12) > (DisHoriMax - 23))
		 {
			 LcdDisStarX = 5;  
			 if((LcdDisStarY += 12) > DisVertMax-23 )
			  {LcdDisStarY = 5;}
			 else{LcdDisStarY += 12;}
     }else{
         LcdDisStarX += 0;}
}
void StringAt(unsigned short StarX,unsigned short StarY,const char *cBuffer)
{
	char asscicount;
	LcdDisStarX = StarX;
  LcdDisStarY = StarY;
	asscicount = strlen(cBuffer)-1;
	do{
			Display_ASC_8X16(Ascii[*cBuffer-32]);
			cBuffer++;
	  }
	while(asscicount--);
}
