#include "timer.h"
#include "led.h"  	 
#include "exti.h"
//#include "IMU.h"
//#include "pid.h"
#include "I2C.h"
//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3ʱ��ʹ��    
 	TIM3->ARR=arr-1;  	//�趨�������Զ���װֵ//�պ� 0.1 ms    
	TIM3->PSC=psc;  	//Ԥ��Ƶ��72000000,�õ�10Khz�ļ���ʱ��		  
	TIM3->DIER|=1<<0;   //��������ж�	  
	TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
  NVIC_Init(1,3,TIM3_IRQChannel,2);//��ռ1�������ȼ�3����2									 
}
void TIM2_PWM_Init(u16 arr,u16 psc)
{		 					 
//�˲������ֶ��޸�IO������
	  RCC->APB1ENR|=1; 	    //TIM2ʱ��ʹ��    
   RCC->AHB1ENR  |= 1<<1;   	//ʹ��PORTBʱ��	
	  GPIOB->MODER &= 0xFF0FFFFF;  //����PB0,1 ouput
	  GPIOB->MODER |= 0x00A00000;
	  GPIOB->OSPEEDR |= 0x00F00000;

	
	  GPIOB->AFR[1]&= 0xFFFF00FF;
	  GPIOB->AFR[1]|= 0x00001100;

	  TIM2->ARR=arr-1;		     	//�趨�������Զ���װֵ 
	  TIM2->PSC=psc;			     //Ԥ��Ƶ������Ƶ
	

   TIM2->CCMR2|=7<<4;   //CH3 PWM2ģʽ 
   TIM2->CCMR2|=7<<12;  //CH4 PWM2ģʽ


   TIM2->CCMR2&=~(1<<3);    //CH1Ԥװ��ʹ��	
   TIM2->CCMR2&=~(1<<11);   //CH2Ԥװ��ʹ��	
   
   TIM2->CCER|=0x3300;   //OCx output enable CCxE=1
	  TIM2->CR1|=0x0080;   	//ARPEʹ�� 
	  TIM2->CR1|=0x01;    	//ʹ�ܶ�ʱ��3 											  
}  	 
void TIM4_PWM_Init(u16 arr,u16 psc)
{		 					 
//�˲������ֶ��޸�IO������
	  RCC->APB1ENR|=1<<2; 	    //TIM2ʱ��ʹ��    
   RCC->AHB1ENR  |= 1<<1;   	//ʹ��PORTBʱ��	
	  GPIOB->MODER &= 0xFFFF0FFF;  //����PB0,1 ouput
	  GPIOB->MODER |= 0x0000A000;
	  GPIOB->OSPEEDR |= 0x0000F000;

	
	  GPIOB->AFR[0]&= 0x00FFFFFF;
	  GPIOB->AFR[0]|= 0x22000000;

	  TIM4->ARR=arr-1;		     	//�趨�������Զ���װֵ 
	  TIM4->PSC=psc;			     //Ԥ��Ƶ������Ƶ
	

   TIM4->CCMR1|=7<<4;   //CH1 PWM2ģʽ 
   TIM4->CCMR1|=7<<12;  //CH1 PWM2ģʽ


   TIM4->CCMR1&=~(1<<3);    //CH1Ԥװ��ʹ��	
   TIM4->CCMR1&=~(1<<11);   //CH2Ԥװ��ʹ��	

   TIM4->CCER|=0x0033;   //OCx output enable CCxE=1
	  TIM4->CR1|=0x0080;   	//ARPEʹ�� 
	  TIM4->CR1|=0x01;    	//ʹ�ܶ�ʱ��3 											  
}  	 

//��ʱ��3�жϷ������	 
void TIM3_IRQHandler(void)
{    		  			    
 
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
}

void TIM5_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<3;	//TIM3ʱ��ʹ��    
 TIM5->ARR=arr-1;  	//�趨�������Զ���װֵ//�պ� 0.1 ms    
	TIM5->PSC=psc;  	//Ԥ��Ƶ��72000000,�õ�10Khz�ļ���ʱ��		   
	TIM5->CR1|=0x01;    //ʹ�ܶ�ʱ��3			 
}

float GET_NOWTIME(void)//���ص�ǰsystick������ֵ,32λ
{
	  float temp=0 ;
	  static uint32_t now=0; // �������ڼ��� ��λ us
 	 now = TIM5->CNT;//����16λʱ��
   TIM5->CNT=0;
  	temp = (float)now / 2000000.0f;          //�����ms���ٳ���2�ó��������ڵ�һ��
	return temp;
}

void get_ms(unsigned long *time)
{

}








