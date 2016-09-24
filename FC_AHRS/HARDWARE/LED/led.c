#include "led.h"

void Init_Led()
{
	 RCC->AHB1ENR  |= 1<<1; //Ê¹ÄÜGPIOBÊ±ÖÓ
   GPIOB->MODER  &= 0xFFFFF3C0;
   GPIOB->MODER  |= 0x00000415;
	 GPIOB->OTYPER &= 0xFFD8;
	 GPIOB->ODR    |= 0x0027;
}
