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
  float desired;     //< ����������ֵ
  float error;        //< ����ֵ-ʵ��ֵ
  float prevError;    //< ǰһ��ƫ��
  float integ;        //< ���ֲ���
  float deriv;        //< ΢�ֲ���
  float kp;           //< ��������
  float ki;           //< ���ֲ���
  float kd;           //< ΢�ֲ���
  float outP;         //< pid�������֣�������
  float outI;         //< pid���ֲ��֣�������
  float outD;         //< pid΢�ֲ��֣�������
  float iLimit;      //< ��������
} pidstatus;
extern int Motor_Thr;					   //����
extern float Euler_Yaw,Euler_Pitch,Euler_Roll;
extern float Gyro_FinalX,Gyro_FinalY,Gyro_FinalZ;
void PID_controllerInit(void);
float pidUpdate(pidstatus* pid, const float measured,float expect,float gyro);
void PID_CAL(void);
void motor_on_off(bool m_control);

void Recev_nRF(void);

#endif
