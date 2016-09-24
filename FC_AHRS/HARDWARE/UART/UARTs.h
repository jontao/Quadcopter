
#ifndef __USARTS_H
#define __USARTS_H	 
#include "sys.h"

#define USART_FLAG_TXE                       ((uint16_t)0x0080)
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
extern u16 USART_RX_STA;
void uart_init(u32 pclk2,u32 bound);
void UART1_Put_Char(unsigned char DataToSend);
void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);
void UART1_ReportMotion(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,
					int16_t hx,int16_t hy,int16_t hz);
void PrintChar(char *s);
#endif
