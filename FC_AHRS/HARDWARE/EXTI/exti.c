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
	SCB->AIRCR=((SCB->AIRCR)&0X0000F8FF)|0X05FA0000|(((~NVIC_Group)&0x07)<<8);//���÷���	 
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
	NVIC->IP[NVIC_Channel/4]|=(((NVIC_PrePri<<(4-NVIC_Group))|(NVIC_SubPri&(0x0f>>NVIC_Group)))&0xf)<<((NVIC_Channel%4)*8+4);//������Ӧ���ȼ����������ȼ�   	    	  				   
}
void EX_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
{
	u8 EXTADDR;
	u8 EXTOFFSET;
	EXTADDR=BITx/4;//�õ��жϼĴ�����ı��
	EXTOFFSET=(BITx%4)*4;
						   
	RCC->APB2ENR|=1<<14;//ʹ��io����ʱ��

	SYSCFG->EXTICR[EXTADDR]&=~(0x000F<<EXTOFFSET);//���ԭ�����ã�����
  SYSCFG->EXTICR[EXTADDR]|=GPIOx<<EXTOFFSET;//EXTI.BITxӳ�䵽GPIOx.BITx
	
	//�Զ�����
	EXTI->IMR|=1<<BITx;//  ����line BITx�ϵ��ж�
	//EXTI->EMR|=1<<BITx;//������line BITx�ϵ��¼� (������������,��Ӳ�����ǿ��Ե�,��������������ʱ���޷������ж�!)
 	if(TRIM&0x01)EXTI->FTSR|=1<<BITx;//line BITx���¼��½��ش���
	if(TRIM&0x02)EXTI->RTSR|=1<<BITx;//line BITx���¼��������ش���
}
void EXTI_Init()
{
  RCC->AHB1ENR |= 1<<2; //ʹ��GPIOAʱ��
  GPIOC->MODER &= 0xFFFFFFCF; //����PB0,1 ouput
  GPIOC->PUPDR &= 0xFFFFFFCF; 
	EX_NVIC_Config(GPIO_C,2,FTIR);
  NVIC_Init(0,0,EXTI2_IRQChannel,1);
}
//�ⲿ�ж�0�������

void EXTI0_IRQHandler(void)
{
	delay_ms(10);//����

	EXTI->PR=1<<0;  //���LINE0�ϵ��жϱ�־λ  
}
//�ⲿ�ж�2�������
void EXTI2_IRQHandler(void)
{
	int i=20000000;
	//delay_ms(10);//����
	if((EXTI->PR&0x00000004)!=0)
	{			    
    TIM3->CR1&=(uint16_t)~1;		
    while(!POWER_SW_DETECT&&i!=0){if(!(--i)){POWER_LED_OFF;i=50000000;while(i--){}POWER_OFF;}}
  }		
	EXTI->PR=1<<2;  //���LINE2�ϵ��жϱ�־λ  
}
//�ⲿ�ж�3�������
void EXTI3_IRQHandler(void)
{	
	if((EXTI->PR&0x00000008)!=0)
	{			    				   				     	    				   
  }	
	EXTI->PR=1<<3;  //���LINE3�ϵ��жϱ�־λ  
}
//�ⲿ�ж�4�������
void EXTI4_IRQHandler(void)
{
	delay_ms(10);//����
	 
	EXTI->PR=1<<4;  //���LINE4�ϵ��жϱ�־λ  
}		   












