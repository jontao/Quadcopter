#include "jonta_fc.h"
void Init_Power_Ctrl()
{ 
	 RCC->AHB1ENR  |= 1<<2; //Ê¹ÄÜGPIOCÊ±ÖÓ
   GPIOC->MODER  &= 0xFFFFFFC0;
   GPIOC->MODER  |= 0x00000005;
	 GPIOC->OTYPER &= 0xFFFC;
   GPIOC->PUPDR  &= 0xFFFFFFC0;
	 GPIOC->PUPDR  |= 0x00000005;
	 POWER_ON;

}
