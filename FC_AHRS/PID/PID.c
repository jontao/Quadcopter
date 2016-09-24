#include "pid.h"
#include "mpu6050.h"
#include <math.h>
#include "jonta_fc.h"
#include "nrf24l01.h"
#include "math.h"
#include "stdlib.h"
float PID_ROLL_KP =1.1;//0.4//0.46
float PID_ROLL_KI =0.009;//0//0.0 //2.0	 0.1
float PID_ROLL_KD =30;//0.15//0.1
//0.2	0.0 0.02
//0.18	0.0	0.025
#define PID_ROLL_INTEGRATION_LIMIT    25.0 // 20.0

float PID_PITCH_KP =1.1;//0.4 
float PID_PITCH_KI =0.009;//0.0		  0.1 0.30
float PID_PITCH_KD =30;//0.09  0.15
#define PID_PITCH_INTEGRATION_LIMIT   25.0 // 20.0

float PID_YAW_KP= -0	;				  //0.5~1.0			 2大了
float PID_YAW_KI =0.0;
float PID_YAW_KD =-0.0	;				   //参数前加负号
#define PID_YAW_INTEGRATION_LIMIT   20.0 // 20.0

float Gyro_FinalX=0;
float Gyro_FinalY=0;
float Gyro_FinalZ=0;

int Elemiddle=0;
int	Ailmiddle=0;
int	Rudmiddle=0;

pidstatus pidRoll;					   //横滚角pid
pidstatus pidPitch;					   //俯仰角pid
pidstatus pidYaw;
	 
int Motor_Thr=0;					   //油门
int Motor_Ele=0;					   //俯仰期望
int Motor_Ail=0;					   //横滚期望
int Motor_Rud=0;					   //航向期望

int MOTOR1;
int MOTOR2;
int MOTOR3;	
int MOTOR4;

float pid_roll;
float pid_pitch;
float pid_yaw;

/*------------------------------------------pid结构初始化-----------------------------------------*/
//输入参数：结构体指针，期望值，kp,ki,kd
void pidInit(pidstatus* pid, const float desired, const float kp,
             const float ki, const float kd)
{
  pid->error = 0;
  pid->prevError = 0;
  pid->integ = 0;
  pid->deriv = 0;
  pid->desired = desired;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
	 pid->iLimit=100.0;
}
void PID_controllerInit(void)
{
    pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
    pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
	   pidInit(&pidYaw,0,PID_YAW_KP,PID_YAW_KI,PID_YAW_KD);

}


float pidUpdate(pidstatus* pid, const float measured,float expect,float gyro)
{
  float output;
  static float lastoutput=0;

  pid->desired=expect;			 				//获取期望角度

  pid->error = pid->desired - measured;	 	  //偏差：期望-测量值
  
  pid->integ += pid->error * (float)(1.0/200);	  //偏差积分
 
  if (pid->integ > pid->iLimit)				  //作积分限制
  {
    pid->integ = pid->iLimit;
  }
  else if (pid->integ < -pid->iLimit)
  {
    pid->integ = -pid->iLimit;
  }				 
 // pid->deriv = (pid->error - pid->prevError) / IMU_UPDATE_DT;		//微分	 应该可用陀螺仪角速度代替
  pid->deriv = -gyro;
		
  if(fabs(pid->error)>0)//Piddeadband                                                                        //pid??
  {
//    pid->outP = pid->kp * pid->error;                                                                 //??????
//    pid->outI = pid->ki * pid->integ;
//    pid->outD = pid->kd * pid->deriv;
    output = (pid->kp * pid->error) +
           (pid->ki * pid->integ) +
           (pid->kd * pid->deriv);
   }
   else
   {
                     output=lastoutput;
   }
		
		
  pid->prevError = pid->error;							 		//更新前一次偏差
  lastoutput=output;

  return output;
}


int MOTORLimit(float value)
{
  	  if(value>400)
	    {
		  	value=400;
		}
	  else if(value<200)
		{
			value=200;
		}
	  else 
		{
		   value=value;
		}

	  return value;
}	
#include "UARTs.h"
/*------------------------------------pid控制------------------------------------*/
void PID_CAL(void)		  //PID参数计算
{
	  PID_controllerInit();
  	pid_roll  = pidUpdate(&pidRoll,Euler_Roll,Motor_Ail,Gyro_FinalX);
	  pid_pitch = pidUpdate(&pidPitch,Euler_Pitch,Motor_Ele,Gyro_FinalY);
  	pid_yaw   = pidUpdate(&pidYaw,Euler_Yaw,Motor_Rud,Gyro_FinalZ);

	  MOTOR1=MOTORLimit(Motor_Thr-pid_pitch);//-pid_yaw	 			//A6号电机
   MOTOR2=MOTORLimit(Motor_Thr +pid_roll	);//+pid_yaw			//B1号电机
	
   MOTOR3=MOTORLimit(Motor_Thr +pid_pitch);	//	+-pid_yaw		//B0号电机
   MOTOR4=MOTORLimit(Motor_Thr-pid_roll	);//	+pid_yaw		//A7号电机
	  //UART1_ReportIMU(MOTOR2,MOTOR4, 0,0,0,0,0);
		if(Motor_Thr<=200)
	 {
	  	MOTOR1=200;
		  MOTOR2=200;
	  	MOTOR3=200;
	  	MOTOR4=200;
	  }		 
   MOTOR1_PWM=		MOTOR1;
	  MOTOR2_PWM=		MOTOR2;
		 MOTOR3_PWM=		MOTOR3;
			MOTOR4_PWM=		MOTOR4;
//	  MOTOR1=Motor_Thr+pid_pitch-pid_yaw;	 			//A6号电机
//   MOTOR2=Motor_Thr-pid_pitch-pid_yaw;				//A7号电机

//   MOTOR3=Motor_Thr -pid_roll+pid_yaw;				//B0号电机
//   MOTOR4=Motor_Thr +pid_roll+pid_yaw;				//B1号电机
}
void motor_on_off(bool m_control)
{
	  MOTOR1_PWM = 200;
	  MOTOR2_PWM = 200;
	  MOTOR3_PWM = 200;
	  MOTOR4_PWM = 200;
	if(m_control==true)
	{
			delay_ms(2000);
			delay_ms(2000);
////			delay_ms(2000);
////			delay_ms(1000);
	}
}
//void get_nRF_data()
//{
//	  int tmp;
//	  
//	  if(Recev_nRF(&tmp)==5)
//			{
//				Motor_Thr=100+tmp;
//			}
//}
#define  ADVANCE_ADDR    (u8)0x80
#define  RETREAT_ADDR    (u8)0x81
#define  LEFT_ADDR       (u8)0x82
#define  RIGHT_ADDR      (u8)0x84
#define  ACCLE_ADDR      (u8)0x85


#define  PID_PITCH_ADDR (u8)0x86
#define  PID_ROLL_ADDR  (u8)0x87
#define  PID_YAW_ADDR   (u8)0x88


void Recev_nRF()
{
	 u8 rx_buf[4][8];
	 if(NRF24L01_RxPacket(rx_buf[0])==0)
		{
			rx_buf[3][7]=0;
			 if(rx_buf[0][0]==ADVANCE_ADDR)
				{ 
					if(rx_buf[0][1]){Motor_Ele=-rx_buf[0][1];}
					return ;
				}
				if(rx_buf[0][0]==RETREAT_ADDR)
				{
      if(rx_buf[0][1]){Motor_Ele=rx_buf[0][1];}
						return ;
				}
				if(rx_buf[0][0]==LEFT_ADDR)
				{
      if(rx_buf[0][1]){Motor_Ail=-rx_buf[0][1];}
						return ;
				}
				if(rx_buf[0][0]==RIGHT_ADDR)
				{
      if(rx_buf[0][1]){Motor_Ail=rx_buf[0][1];}
						return ;
				}
				if(rx_buf[0][0]==ACCLE_ADDR)
				{
      Motor_Thr=200+rx_buf[0][1];
					 return ;
				}
				if(rx_buf[0][0]==PID_PITCH_ADDR)
				{
					 PID_PITCH_KP =atof((const char*)rx_buf[1]);//0.4 
      PID_PITCH_KI =atof((const char*)rx_buf[2]);//0.0		  0.1 0.30
      PID_PITCH_KD =atof((const char*)rx_buf[3]);//0.09  0.15
					 return ;
				}
				if(rx_buf[0][0]==PID_ROLL_ADDR)
				{
					 
					 PID_ROLL_KP =atof((const char*)rx_buf[1]);//0.4//0.46
      PID_ROLL_KI =atof((const char*)rx_buf[2]);//0//0.0 //2.0	 0.1
      PID_ROLL_KD =atof((const char*)rx_buf[3]);//0.15//0.1
					 return ;
				}
				if(rx_buf[0][0]==PID_YAW_ADDR)
				{
					 PID_YAW_KP= atof((const char*)rx_buf[1])	;				  //0.5~1.0			 2大了
      PID_YAW_KI =atof((const char*)rx_buf[2]);
      PID_YAW_KD =atof((const char*)rx_buf[3])	;				   //参数前加负号
					 return ;
				}
		}

}

