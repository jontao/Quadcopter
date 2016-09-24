#ifndef _JONTA_FC_H_
#define _JONTA_FC_H_
#include "stm32f4xx.h"
#include "led/led.h"
#include "sys.h" 
#include "delay.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "led.h" 
#include "exti.h"
#include "delay.h"
#include "core_cm4.h"
#include "power.h"
#include "timer.h"
#include "rm68042.h"
#include <stdio.h>
#include "stdbool.h"


#define MOTOR1_PWM   TIM4->CCR1
#define MOTOR2_PWM   TIM4->CCR2
#define MOTOR3_PWM   TIM2->CCR3
#define MOTOR4_PWM   TIM2->CCR4
#define MOTOR_STOP   false
#define MOTOR_START  true
	
#endif

