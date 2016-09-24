#ifndef _SPI_H_
#define _SPI_H_
#include "stm32F4xx.h"

void SPI2_Init(void);
u8 SPI2_ReadWriteByte(u8 TxData);
void SPI2_SetSpeed(u8 SpeedConfig);
#endif
