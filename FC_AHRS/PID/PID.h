#ifndef _PID_H_
#define _PID_H_

#include "stdbool.h"


#define MOTOR1_PWM   TIM4->CCR1
#define MOTOR2_PWM   TIM4->CCR2
#define MOTOR3_PWM   TIM2->CCR3
#define MOTOR4_PWM   TIM2->CCR4
#define MOTOR_STOP   false
#define MOTOR_START  true
	
typedef struct
{
  float desired;     //< 被调量期望值
  float error;        //< 期望值-实际值
  float prevError;    //< 前一次偏差
  float integ;        //< 积分部分
  float deriv;        //< 微分部分
  float kp;           //< 比例参数
  float ki;           //< 积分参数
  float kd;           //< 微分参数
  float outP;         //< pid比例部分，调试用
  float outI;         //< pid积分部分，调试用
  float outD;         //< pid微分部分，调试用
  float iLimit;      //< 积分限制
} pidstatus;
extern int Motor_Thr;					   //油门
extern float Euler_Yaw,Euler_Pitch,Euler_Roll;
extern float Gyro_FinalX,Gyro_FinalY,Gyro_FinalZ;
void PID_controllerInit(void);
float pidUpdate(pidstatus* pid, const float measured,float expect,float gyro);
void PID_CAL(void);
void motor_on_off(bool m_control);

void Recev_nRF(void);

#endif
