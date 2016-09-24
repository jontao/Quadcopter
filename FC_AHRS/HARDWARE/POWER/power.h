#ifndef _POWER_H_
#define _POWER_H_
#include "sys.h"
#define  POWER_LED_ON     GPIOC->MODER &= 0xFFFFFFFC
#define  POWER_LED_OFF    GPIOC->MODER &= 0xFFFFFFFD;PCout(0)=1
#define  POWER_ON         PCout(1)=1
#define  POWER_OFF        PCout(1)=0
#define  POWER_SW_DETECT  PCin(2)
void Init_Power_Ctrl(void);
#endif
