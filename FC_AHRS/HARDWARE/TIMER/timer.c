#include "timer.h"
#include "led.h"  	 
#include "exti.h"
//#include "IMU.h"
//#include "pid.h"
#include "I2C.h"
//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<1;	//TIM3时钟使能    
 	TIM3->ARR=arr-1;  	//设定计数器自动重装值//刚好 0.1 ms    
	TIM3->PSC=psc;  	//预分频器72000000,得到10Khz的计数时钟		  
	TIM3->DIER|=1<<0;   //允许更新中断	  
	TIM3->CR1|=0x01;    //使能定时器3
  NVIC_Init(1,3,TIM3_IRQChannel,2);//抢占1，子优先级3，组2									 
}
void TIM2_PWM_Init(u16 arr,u16 psc)
{		 					 
//此部分需手动修改IO口设置
	  RCC->APB1ENR|=1; 	    //TIM2时钟使能    
   RCC->AHB1ENR  |= 1<<1;   	//使能PORTB时钟	
	  GPIOB->MODER &= 0xFF0FFFFF;  //设置PB0,1 ouput
	  GPIOB->MODER |= 0x00A00000;
	  GPIOB->OSPEEDR |= 0x00F00000;

	
	  GPIOB->AFR[1]&= 0xFFFF00FF;
	  GPIOB->AFR[1]|= 0x00001100;

	  TIM2->ARR=arr-1;		     	//设定计数器自动重装值 
	  TIM2->PSC=psc;			     //预分频器不分频
	

   TIM2->CCMR2|=7<<4;   //CH3 PWM2模式 
   TIM2->CCMR2|=7<<12;  //CH4 PWM2模式


   TIM2->CCMR2&=~(1<<3);    //CH1预装载使能	
   TIM2->CCMR2&=~(1<<11);   //CH2预装载使能	
   
   TIM2->CCER|=0x3300;   //OCx output enable CCxE=1
	  TIM2->CR1|=0x0080;   	//ARPE使能 
	  TIM2->CR1|=0x01;    	//使能定时器3 											  
}  	 
void TIM4_PWM_Init(u16 arr,u16 psc)
{		 					 
//此部分需手动修改IO口设置
	  RCC->APB1ENR|=1<<2; 	    //TIM2时钟使能    
   RCC->AHB1ENR  |= 1<<1;   	//使能PORTB时钟	
	  GPIOB->MODER &= 0xFFFF0FFF;  //设置PB0,1 ouput
	  GPIOB->MODER |= 0x0000A000;
	  GPIOB->OSPEEDR |= 0x0000F000;

	
	  GPIOB->AFR[0]&= 0x00FFFFFF;
	  GPIOB->AFR[0]|= 0x22000000;

	  TIM4->ARR=arr-1;		     	//设定计数器自动重装值 
	  TIM4->PSC=psc;			     //预分频器不分频
	

   TIM4->CCMR1|=7<<4;   //CH1 PWM2模式 
   TIM4->CCMR1|=7<<12;  //CH1 PWM2模式


   TIM4->CCMR1&=~(1<<3);    //CH1预装载使能	
   TIM4->CCMR1&=~(1<<11);   //CH2预装载使能	

   TIM4->CCER|=0x0033;   //OCx output enable CCxE=1
	  TIM4->CR1|=0x0080;   	//ARPE使能 
	  TIM4->CR1|=0x01;    	//使能定时器3 											  
}  	 

//定时器3中断服务程序	 
void TIM3_IRQHandler(void)
{    		  			    
 
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}

void TIM5_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<3;	//TIM3时钟使能    
 TIM5->ARR=arr-1;  	//设定计数器自动重装值//刚好 0.1 ms    
	TIM5->PSC=psc;  	//预分频器72000000,得到10Khz的计数时钟		   
	TIM5->CR1|=0x01;    //使能定时器3			 
}

float GET_NOWTIME(void)//返回当前systick计数器值,32位
{
	  float temp=0 ;
	  static uint32_t now=0; // 采样周期计数 单位 us
 	 now = TIM5->CNT;//读高16位时间
   TIM5->CNT=0;
  	temp = (float)now / 2000000.0f;          //换算成ms，再除以2得出采样周期的一半
	return temp;
}

void get_ms(unsigned long *time)
{

}








