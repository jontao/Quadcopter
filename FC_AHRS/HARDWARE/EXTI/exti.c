#include "jonta_fc.h"

/***********************NVIC_VectTab: ******************************/
/**********specifies if the vector table is in RAM or FLASH memory.*/
void NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset)
{ 
  SCB->VTOR = NVIC_VectTab | (Offset & (uint32_t)0x1FFFFF80);
}
/**
  * @brief  Configures the priority grouping: pre-emption priority and subpriority.
  * @param  NVIC_PriorityGroup: specifies the priority grouping bits length. 
  *   This parameter can be one of the following values:
  *     @arg NVIC_PriorityGroup_0: 0 bits for pre-emption priority
  *                                4 bits for subpriority
  *     @arg NVIC_PriorityGroup_1: 1 bits for pre-emption priority
  *                                3 bits for subpriority
  *     @arg NVIC_PriorityGroup_2: 2 bits for pre-emption priority
  *                                2 bits for subpriority
  *     @arg NVIC_PriorityGroup_3: 3 bits for pre-emption priority
  *                                1 bits for subpriority
  *     @arg NVIC_PriorityGroup_4: 4 bits for pre-emption priority
  *                                0 bits for subpriority
  * @note   When the NVIC_PriorityGroup_0 is selected, IRQ pre-emption is no more possible. 
  *         The pending IRQ priority will be managed only by the subpriority. 
  * @retval None
  */
void NVIC_PriGroConf(u8 NVIC_Group)
{ 
	SCB->AIRCR=((SCB->AIRCR)&0X0000F8FF)|0X05FA0000|(((~NVIC_Group)&0x07)<<8);//设置分组	 
}
/**
  * @brief  Initializes the NVIC peripheral according to the specified
  *         parameters in the NVIC_InitStruct.
  * @note   To configure interrupts priority correctly, the NVIC_PriorityGroupConfig()
  *         function should be called before. 
  * @param  NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure that contains
  *         the configuration information for the specified NVIC peripheral.
  * @retval None
  */
void NVIC_Init(u8 NVIC_PrePri,u8 NVIC_SubPri,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
	NVIC_PriGroConf(NVIC_Group);
	if(NVIC_Channel<32)NVIC->ISER[0]|=1<<NVIC_Channel;
	else NVIC->ISER[1]|=1<<(NVIC_Channel-32);    
	NVIC->IP[NVIC_Channel/4]|=(((NVIC_PrePri<<(4-NVIC_Group))|(NVIC_SubPri&(0x0f>>NVIC_Group)))&0xf)<<((NVIC_Channel%4)*8+4);//设置响应优先级和抢断优先级   	    	  				   
}
void EX_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
{
	u8 EXTADDR;
	u8 EXTOFFSET;
	EXTADDR=BITx/4;//得到中断寄存器组的编号
	EXTOFFSET=(BITx%4)*4;
						   
	RCC->APB2ENR|=1<<14;//使能io复用时钟

	SYSCFG->EXTICR[EXTADDR]&=~(0x000F<<EXTOFFSET);//清除原来设置！！！
  SYSCFG->EXTICR[EXTADDR]|=GPIOx<<EXTOFFSET;//EXTI.BITx映射到GPIOx.BITx
	
	//自动设置
	EXTI->IMR|=1<<BITx;//  开启line BITx上的中断
	//EXTI->EMR|=1<<BITx;//不屏蔽line BITx上的事件 (如果不屏蔽这句,在硬件上是可以的,但是在软件仿真的时候无法进入中断!)
 	if(TRIM&0x01)EXTI->FTSR|=1<<BITx;//line BITx上事件下降沿触发
	if(TRIM&0x02)EXTI->RTSR|=1<<BITx;//line BITx上事件上升降沿触发
}
void EXTI_Init()
{
  RCC->AHB1ENR |= 1<<2; //使能GPIOA时钟
  GPIOC->MODER &= 0xFFFFFFCF; //设置PB0,1 ouput
  GPIOC->PUPDR &= 0xFFFFFFCF; 
	EX_NVIC_Config(GPIO_C,2,FTIR);
  NVIC_Init(0,0,EXTI2_IRQChannel,1);
}
//外部中断0服务程序

void EXTI0_IRQHandler(void)
{
	delay_ms(10);//消抖

	EXTI->PR=1<<0;  //清除LINE0上的中断标志位  
}
//外部中断2服务程序
void EXTI2_IRQHandler(void)
{
	int i=20000000;
	//delay_ms(10);//消抖
	if((EXTI->PR&0x00000004)!=0)
	{			    
    TIM3->CR1&=(uint16_t)~1;		
    while(!POWER_SW_DETECT&&i!=0){if(!(--i)){POWER_LED_OFF;i=50000000;while(i--){}POWER_OFF;}}
  }		
	EXTI->PR=1<<2;  //清除LINE2上的中断标志位  
}
//外部中断3服务程序
void EXTI3_IRQHandler(void)
{	
	if((EXTI->PR&0x00000008)!=0)
	{			    				   				     	    				   
  }	
	EXTI->PR=1<<3;  //清除LINE3上的中断标志位  
}
//外部中断4服务程序
void EXTI4_IRQHandler(void)
{
	delay_ms(10);//消抖
	 
	EXTI->PR=1<<4;  //清除LINE4上的中断标志位  
}		   












