#ifndef __RM68042_H
#define __RM68042_H	 
#include "sys.h"

#define DisHoriMax    480
#define DisVertMax    320
#define FontC         0xffff

//LCD地址结构体
typedef struct
{
	u16 LCD_REG;
	u16 LCD_RAM;
} LCD_TypeDef;
//使用NOR/SRAM的 Bank1.sector1,地址位HADDR[27,26]=11 A0作为数据命令区分线 
//注意设置时STM32内部会右移一位对其! 		    
#define LCD_BASE        ((u32)(0x68000000 | 0x00000000))
#define Lcd             ((LCD_TypeDef *) LCD_BASE)
#define LCD_REST        PAout(1)
#define LCD_BL          PAout(0)
void LCD_rm68042_Init(void);
void LCD_WR_DATA(u16 data);
void LCD_WR_REG(u16 reg);
void LCD_rm68042_Pixel_Draw(u32 lX,u32 lY,u32 color);
u32 LCD_rm68042_Pixel_Read(u32 lX,u32 lY);
void LCD_SetBox(u32 xStart,u32 yStart,u32 xLong,u32 yLong);
void LCD_Fill_Rect(u32 xStart,u32 yStart,u32 xLong,u32 yLong,u32 color );
void LCD_Picture_Display(u32 xStart,u32 yStart, u32 xLong,u32 yLong,uc8 * pPicture);
void Display_ASC_8X16(u8 *cBuffer);
void StringAt(unsigned short StarX,unsigned short StarY,const char *cBuffer);
extern u16 LcdDisStarX ;
extern u16 LcdDisStarY ;
void Display_GBK_16X16(u8* cBuffer);
#endif
