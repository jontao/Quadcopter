#ifndef __SYS_H
#define __SYS_H	 
#include "stm32f4xx.h"
#define uchar unsigned char
#define uint unsigned short
#define ulong unsigned long
#define ubit unsigned
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr&0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
//F4XXIO 口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr (GPIOB_BASE+20) //0x40020414
#define GPIOC_ODR_Addr (GPIOC_BASE+20) //0x40020814
#define GPIOD_ODR_Addr (GPIOD_BASE+20) //0x40020C14
#define GPIOE_ODR_Addr (GPIOE_BASE+20) //0x40021014
#define GPIOF_ODR_Addr (GPIOF_BASE+20) //0x40021414
#define GPIOG_ODR_Addr (GPIOG_BASE+20) //0x40021814
#define GPIOH_ODR_Addr (GPIOH_BASE+20) //0x40021C14
#define GPIOI_ODR_Addr (GPIOI_BASE+20) //0x40022014
#define GPIOA_IDR_Addr (GPIOA_BASE+16) //0x40020010
#define GPIOB_IDR_Addr (GPIOB_BASE+16) //0x40020410
#define GPIOC_IDR_Addr (GPIOC_BASE+16) //0x40020810
#define GPIOD_IDR_Addr (GPIOD_BASE+16) //0x40020C10
#define GPIOE_IDR_Addr (GPIOE_BASE+16) //0x40021010
#define GPIOF_IDR_Addr (GPIOF_BASE+16) //0x40021410
#define GPIOG_IDR_Addr (GPIOG_BASE+16) //0x40021810
#define GPIOH_IDR_Addr (GPIOF_BASE+16) //0x40021C10
#define GPIOI_IDR_Addr (GPIOG_BASE+16) //0x40022010
 //F4XXIO 口操作 , 只对单一的 IO 口 ,n 的值范围 [0-15]
 #define PAout(n) BIT_ADDR(GPIOA_ODR_Addr,n) //GPIOA 某一位输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr,n) //GPIOA 某一位输入
#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr,n) //GPIOB 某一位输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr,n) //GPIOB 某一位输入
#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr,n) //GPIOC 某一位输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr,n) //GPIOC 某一位输入
#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr,n) //GPIOD 某一位输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr,n) //GPIOD 某一位输入
#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr,n) //GPIOE 某一位输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr,n) //GPIOE 某一位输入
#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr,n) //GPIOF 某一位输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr,n) //GPIOF 某一位输入
#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr,n) //GPIOG 某一位输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr,n) //GPIOG 某一位输入
#define PHout(n) BIT_ADDR(GPIOH_ODR_Addr,n) //GPIOH 某一位输出
#define PHin(n) BIT_ADDR(GPIOH_IDR_Addr,n) //GPIOH 某一位输入
#define PIout(n) BIT_ADDR(GPIOI_ODR_Addr,n) //GPIOI 某一位输出
#define PIin(n) BIT_ADDR(GPIOI_IDR_Addr,n) //GPIOI 某一位输入
/*************************************************************************/
/////////////////////////////////////////////////////////////////
//Ex_NVIC_Config专用定义
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6 
#define FTIR   1  //下降沿触发
#define RTIR   2  //上升沿触发
								   

//JTAG模式设置定义
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	

/////////////////////////////////////////////////////////////////  
void SystemClockInit(u8 PLL);  //时钟初始化  
#endif











