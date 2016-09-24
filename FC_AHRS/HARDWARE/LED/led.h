#ifndef _LED_H_
#define _LED_H_
#include "sys.h"
#define LED1_ON   PBout(2)=0
#define LED1_OFF   PBout(2)=1
#define LED2_ON   PBout(1)=0
#define LED2_OFF   PBout(1)=1
#define LED3_ON   PBout(0)=0
#define LED3_OFF   PBout(0)=1
#define LED4_ON   PBout(5)=0
#define LED4_OFF   PBout(5)=1

extern void Init_Led(void);
#endif
