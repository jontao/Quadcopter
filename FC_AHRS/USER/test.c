
#include "jonta_fc.h"
#include "nrf24l01.h"
#include "pid.h"

//#define UART_ENABLE
#ifdef  UART_ENABLE
#include "UARTs.h"
#endif


#define Gyro_250_Scale_Factor   131.0f
#define Gyro_500_Scale_Factor   65.5f
#define Gyro_1000_Scale_Factor  32.8f
#define Gyro_2000_Scale_Factor  16.4f
#define Accel_2_Scale_Factor    16384.0f
#define Accel_4_Scale_Factor    8192.0f
#define Accel_8_Scale_Factor    4096.0f
#define Accel_16_Scale_Factor   2048.0f
#define FILTER_NUM	            20
#define Accel_Zout_Offset		0//400


void init_quaternion(void);
float invSqrt(float x);
int get_gyro_bias(void);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#define Kp 2.0f      //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f    //integral gain governs rate of convergence of gyroscope biases
float q0, q1, q2, q3;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;
unsigned char data_write[14];
float Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset;

float Euler_Yaw,Euler_Pitch,Euler_Roll;
	
int main(void)
{  

	 Delay_Init(168);     
	 RCC->APB2ENR |= (1<<14);
 	Init_Power_Ctrl();       //启动STM32F4飞控电源管理
  EXTI_Init();
	 Init_Led();
/*84000/420=200khz T=0.005ms pwm频率=tim_frequency/(TIMX_ARR-1)=200/1000=0.200khz*/
  TIM2_PWM_Init(1000,419);
#ifdef UART_ENABLE
	 uart_init(84,115200);
#else
	 TIM4_PWM_Init(1000,419);
#endif
		 
	 TIM5_Int_Init(0xffff,83);
	 
	 motor_on_off(MOTOR_START);//启动电机，处于待机状态
	 NRF24L01_Init();	
  while(NRF24L01_Check()){LED1_ON;delay_ms(500);;LED1_OFF;delay_ms(500);} //检测遥控 
	 LED1_ON;
		InitMPU6050();
  Init_HMC5883L();
		LED2_ON;
	 get_gyro_bias();         //校正陀螺仪 
	 init_quaternion();       //初始化四元素
		LED3_ON;
  PID_controllerInit();    //初始化PID
		LED4_ON;
		delay_ms(500);
		LED1_OFF;LED2_OFF;LED3_OFF;LED4_OFF;
		delay_ms(500);
		LED1_ON;LED2_ON;LED3_ON;LED4_ON;
		delay_ms(500);
		LED1_OFF;LED2_OFF;LED3_OFF;LED4_OFF;
		delay_ms(500);
		RX_Mode();
		LED1_ON;LED2_ON;LED3_ON;LED4_ON;
  while(1)
  {	
    signed short int gyro[3], accel[3], mag[3];
	   static uint8_t filter_cnt=0;
	   float temp[3]={0};
	   uint8_t i;
	   float ACC_X_BUF[FILTER_NUM], ACC_Y_BUF[FILTER_NUM], ACC_Z_BUF[FILTER_NUM];
  	 if(!i2cread(MPU_ADDR, ACCEL_XOUT_H, 14, data_write))
	   {
	    accel[0]=(((signed short int)data_write[0])<<8) | data_write[1];
	    accel[1]=(((signed short int)data_write[2])<<8) | data_write[3];
		   accel[2]=((((signed short int)data_write[4])<<8) | data_write[5])   + Accel_Zout_Offset;
	   	gyro[0] =((((signed short int)data_write[8])<<8) | data_write[9]);
		   gyro[1] =((((signed short int)data_write[10])<<8) | data_write[11]);
		   gyro[2] =((((signed short int)data_write[12])<<8) | data_write[13]);
	     		 
	 	  init_ax=(float)accel[0];   	  
		   init_ay=(float)accel[1];
		   init_az=(float)accel[2];
		   init_gx=((float)gyro[0] - Gyro_Xout_Offset) *0.061*0.0174;//* 0.000266;    //单位转化成：弧度/s，0.000266=1/(Gyro_500_Scale_Factor * 57.295780)
		   init_gy=((float)gyro[1] - Gyro_Yout_Offset) *0.061*0.0174;//* 0.000266;
	   	init_gz=((float)gyro[2] - Gyro_Zout_Offset) *0.061*0.0174;//* 0.000266;
					
					Gyro_FinalX=init_gx;
					Gyro_FinalY=init_gy;
					Gyro_FinalZ=init_gz;
					
					
		  //accel加权平均滤波，部分消除震动影响
		   ACC_X_BUF[filter_cnt] = init_ax;//更新滑动窗口数组
	    ACC_Y_BUF[filter_cnt] = init_ay;
	    ACC_Z_BUF[filter_cnt] = init_az;
	    for(i=0;i<FILTER_NUM;i++)
	    {
		      temp[0] += ACC_X_BUF[i];
		      temp[1] += ACC_Y_BUF[i];
		      temp[2] += ACC_Z_BUF[i];
	    }
	    init_ax = temp[0] * 0.05*0.244*0.0098;     // 0.05相当于除以FILTER_NUM，提高运算速度
	    init_ay = temp[1] * 0.05*0.244*0.0098;     
	    init_az = temp[2] * 0.05*0.244*0.0098;     
	    filter_cnt++;
	    if(filter_cnt==FILTER_NUM)	filter_cnt=0;
//     printf("gx=%f, gy=%f, gz=%f, ax=%f, ay=%f, az=%f \n\r", init_gx,init_gy,init_gz, init_ax,init_ay,init_az);

        mpu_get_compass_reg(mag);  //读取compass数据
        //进行x y轴的校准，未对z轴进行校准，参考ST的校准方法 
        init_mx =((float)mag[0]) - 8;						
        init_my =((float)mag[2] * 1.046632) - 1.569948;
        init_mz =(float)-mag[1];

//		    printf(" mx=%f, my=%f, mz=%f \n\r", init_mx,init_my,init_mz);
        AHRSupdate(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz); 
     } 
				 Recev_nRF();
     PID_CAL();//PID调整
     //UART1_ReportIMU(Euler_Yaw*10, Euler_Pitch*10, Euler_Roll*10,0,0,0,0);
     //delay_ms(10);
  }
}
/*******************************************************************************
* Function Name  : init_quaternion
* Description    : 算出初始化四元数q0 q1 q2 q3.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(void)
{ 
  unsigned long timestamp;
  signed short int accel[3], mag[3];
  float init_Yaw, init_Pitch, init_Roll;
  uint8_t i;

  if(!i2cread(MPU_ADDR, ACCEL_XOUT_H, 6, data_write))
    {
	  accel[0]=(((signed short int)data_write[0])<<8) | data_write[1];
	  accel[1]=(((signed short int)data_write[2])<<8) | data_write[3];
	  accel[2]=((((signed short int)data_write[4])<<8) | data_write[5]) + Accel_Zout_Offset;
	  	    
	  init_ax=((float)accel[0]) *0.244*0.0098 ;// /Accel_4_Scale_Factor;	   //单位转化成重力加速度的单位：g
	  init_ay=((float)accel[1])*0.244*0.0098; // / Accel_4_Scale_Factor;
   init_az=((float)accel[2])*0.244*0.0098;//  / Accel_4_Scale_Factor;
	 // printf("ax=%f, ay=%f, az=%f  \n\r", init_ax, init_ay, init_az);
	  for(i=0;i<5;i++)   //第一次读取的compsaa数据是错的，因此要多读几次保证以后数据正确，芯片bug
	  {
        mpu_get_compass_reg(mag);  //读取compass数据
        //进行x y轴的校准，未对z轴进行校准，参考MEMSense的校准方法 
        init_mx =((float)mag[0]) - 8;						
        init_my =((float)mag[2]) * 1.046632 - 1.569948;
        init_mz =(float)-mag[1];
    //    printf("mx=%f, my=%f, mz=%f \n\r", init_mx, init_my, init_mz);
	  }
//陀螺仪y轴为前进方向    
init_Roll = -atan2(init_ax, init_az);    //算出的单位是弧度，如需要观察则应乘以57.295780转化为角度
init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
                   init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//类似于atan2(my, mx)，其中的init_Roll和init_Pitch是弧度
				            
//将初始化欧拉角转换成初始化四元数，注意sin(a)的位置的不同，可以确定绕xyz轴转动是Pitch还是Roll还是Yaw，按照ZXY顺序旋转,Qzyx=Qz*Qy*Qx，其中的init_YawRollPtich是角度        
q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是pitch
q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是roll
q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw

//陀螺仪x轴为前进方向
//  init_Roll  = atan2(init_ay, init_az);
//  init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
//  init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
//                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);
//  q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
//  q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是roll
//  q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是pitch
//  q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw

init_Roll  = init_Roll * 57.295780;	 //弧度转角度
init_Pitch = init_Pitch * 57.295780;
init_Yaw   = init_Yaw * 57.295780;
if(init_Yaw < 0){init_Yaw = init_Yaw + 360;}      //将Yaw的范围转成0-360
if(init_Yaw > 360){init_Yaw = init_Yaw - 360;} 	
//while(1){
//printf("初始化四元数：Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw, init_Pitch, init_Roll);
//delay_ms(500);
//} 
  }
}

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3需要初始化才能带入到下面的程序中，不能直接使用1 0 0 0进行下面的计算，整个步骤为：
// 1.首先校准accle gyro mag；
// 2.调用init_quaternion，根据1中accle的xyz轴数据，并利用公式计算出初始化欧拉角，
//   其中ACCEL_1G=9.81，单位都是m/s2，而init_Yaw可以用磁力计计算出来；
// 3.根据自己的采样周期，来调整halfT，halfT=采样周期/2，采样周期为执行1次AHRSupdate所用的时间；
// 4.将2中计算出的欧拉角转化为初始化的四元数q0 q1 q2 q3，融合加速度计，陀螺仪，算出更新后的欧拉角pitch和roll，然后使用pitch roll和磁力计的数据进行互补滤波融合得到Yaw，即可使用，但是欧拉角有奇点；
// 5.或直接使用四元数；
// 6.重复4，即可更新姿态;

//总的来说，核心是陀螺仪，加速度计用来修正补偿Pitch和Roll，磁力计用来修正补偿Yaw;
//以下程序中，gx, gy, gz单位为弧度/s，ax, ay, az为加速度计输出的原始16进制数据, mx, my, mz为磁力计输出的原始16进制数据；
//前进方向：mpu9150的加速度计和陀螺仪的x轴为前进方向;
//以下程序采用的参考方向为：mpu9150的加速度计和陀螺仪所指的xyz方向为正方向；

//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
//然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)

//欧拉角单位为弧度radian，乘以57.3以后转换为角度,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
////   float norm, halfT;
////   float hx, hy, hz, bz, by;
////   float vx, vy, vz, wx, wy, wz;
////   float ex, ey, ez;
////   float Pitch, Roll, Yaw;

/////*方便之后的程序使用，减少计算时间*/
////   //auxiliary variables to reduce number of repeated operations，
////   float q0q0 = q0*q0;
////   float q0q1 = q0*q1;
////   float q0q2 = q0*q2;
////   float q0q3 = q0*q3;
////   float q1q1 = q1*q1;
////   float q1q2 = q1*q2;
////   float q1q3 = q1*q3;
////   float q2q2 = q2*q2;   
////   float q2q3 = q2*q3;
////   float q3q3 = q3*q3;
////          
/////*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
////   //normalise the measurements
////   norm = invSqrt(ax*ax + ay*ay + az*az);       
////   ax = ax * norm;
////   ay = ay * norm;
////   az = az * norm;
////   norm = invSqrt(mx*mx + my*my + mz*mz);          
////   mx = mx * norm;
////   my = my * norm;
////   mz = mz * norm;         
////        
/////*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值），下面这个是从飞行器坐标系到世界坐标系的转换公式*/
////   //compute reference direction of flux
////   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
////   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
////   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/////*计算地理坐标系下的磁场矢量bxyz（参考值）。
////因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），我定义by指向正北，所以by=某值，bx=0
////但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
////我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
////磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
////因为bx=0，所以就简化成(by*by)  = ((hx*hx) + (hy*hy))。可算出by。这里修改by和bx指向可以定义哪个轴指向正北*/
//////   bx = sqrtf((hx*hx) + (hy*hy));
////   by = sqrtf((hx*hx) + (hy*hy));
////   bz = hz;        
////    
////   // estimated direction of gravity and flux (v and w)，下面这个是从世界坐标系到飞行器坐标系的转换公式(转置矩阵)
////   vx = 2*(q1q3 - q0q2);
////   vy = 2*(q0q1 + q2q3);
////   vz = q0q0 - q1q1 - q2q2 + q3q3;

/////*我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
////因为bx=0，所以所有涉及到bx的部分都被省略了。同理by=0，所以所有涉及到by的部分也可以被省略，这根据自己定义那个轴指北有关。
////类似上面重力vxyz的推算，因为重力g的az=1，ax=ay=0，所以上面涉及到gxgy的部分也被省略了
////你可以看看两个公式：wxyz的公式，把by换成ay（0），把bz换成az（1），就变成了vxyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
//////   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//////   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//////   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
////   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
////   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
////   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
////           
//////现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
////   // error is sum of cross product between reference direction of fields and direction measured by sensors
////   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
////   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
////   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
////   
//////   // integral error scaled integral gain
//////   exInt = exInt + ex*Ki;		
//////   eyInt = eyInt + ey*Ki;
//////   ezInt = ezInt + ez*Ki;
//////   // adjusted gyroscope measurements
//////   gx = gx + Kp*ex + exInt;
//////   gy = gy + Kp*ey + eyInt;
//////   gz = gz + Kp*ez + ezInt;

////   halfT=GET_NOWTIME();		//得到每次姿态更新的周期的一半
////   
////   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //很关键的一句话，原算法没有
////   {
////      // integral error scaled integral gain
////      exInt = exInt + ex*Ki * halfT;			   //乘以采样周期的一半
////      eyInt = eyInt + ey*Ki * halfT;
////      ezInt = ezInt + ez*Ki * halfT;
////      // adjusted gyroscope measurements
////      gx = gx + Kp*ex + exInt;
////      gy = gy + Kp*ey + eyInt;
////      gz = gz + Kp*ez + ezInt;
////   }         

////   // integrate quaternion rate and normalise，四元数更新算法
////   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
////   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
////   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
////   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
////        
////   // normalise quaternion
////   norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
////   q0 = q0 * norm;       //w
////   q1 = q1 * norm;       //x
////   q2 = q2 * norm;       //y
////   q3 = q3 * norm;       //z
////        
///////*由四元数计算出Pitch  Roll  Yaw，只需在需要PID控制时才将四元数转化为欧拉角
//////乘以57.295780是为了将弧度转化为角度*/
////	Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780;  //偏航角，绕z轴转动	
////    if(Yaw < 0 ){Yaw = Yaw + 360;}
////	if(Yaw > 360 ){Yaw = Yaw - 360;}
////	Pitch = asin(2*q2*q3 + 2*q0*q1) * 57.295780; //俯仰角，绕x轴转动	 
////    Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780; //滚动角，绕y轴转动

/*最初的由四元数计算出Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
乘以57.295780是为了将弧度转化为角度*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.295780; //俯仰角，绕y轴转动	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.295780; //滚动角，绕x轴转动
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.295780;  //偏航角，绕z轴转动

//	printf("halfT=%f \n\r", halfT);
    //printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", Yaw, Pitch, Roll);
	   //UART1_ReportIMU(Yaw*10, Pitch*10, Roll*10,0,0,0,0);
	   //delay_ms(10);
				

float Pitch, Roll, Yaw;
float norm, halfT;
//  float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  // ???????????
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
//  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
//  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc?????
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              ?????????/??
  vx = 2*(q1q3 - q0q2);												//????xyz???
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //???????????????
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								  //???????
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//???PI???????,???????
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							//???gz????????????????,??????????????
 halfT=GET_NOWTIME();
  // integrate quaternion rate and normalise						   //????????
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

	if(Gyro_FinalZ>180)	Gyro_FinalZ-=360;
	if(Gyro_FinalZ<-180)	Gyro_FinalZ+=360;
	  Yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3;
  //Yaw = Gyro_FinalZ;//atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
  Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
	    Euler_Yaw=Yaw;
	    Euler_Pitch=Pitch;
	    Euler_Roll=Roll;
}

/*******************************************************************************
校准陀螺仪零偏	
*******************************************************************************/
int get_gyro_bias(void)
{
  unsigned short int i;
  signed short int gyro[3];
  signed int gyro_x=0, gyro_y=0, gyro_z=0;
  static unsigned short count=0;

  for(i=0;i<5000;i++)
  {
     if(!i2cread(MPU_ADDR, GYRO_XOUT_H, 6, data_write))
	    {
	     gyro[0] = ((((signed short int)data_write[0])<<8) | data_write[1]);
	     gyro[1] = ((((signed short int)data_write[2])<<8) | data_write[3]);
	     gyro[2] = ((((signed short int)data_write[4])<<8) | data_write[5]);
	     gyro_x += gyro[0];
	     gyro_y	+= gyro[1];
	     gyro_z	+= gyro[2];
	     count++;
	     }
  }
  Gyro_Xout_Offset = (float)gyro_x / count;
  Gyro_Yout_Offset = (float)gyro_y / count;
  Gyro_Zout_Offset = (float)gyro_z / count;
//  printf("gyro_x=%d, gyro_y=%d, gyro_z=%d, Gyro_Xout_Offset=%f, Gyro_Yout_Offset=%f, Gyro_Zout_Offset=%f, count=%d \n\r",
//          gyro_x, gyro_y, gyro_z, Gyro_Xout_Offset, Gyro_Yout_Offset, Gyro_Zout_Offset, count);
  count = 0;

  return 0;
}


/*******************************************************************************
快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍 	
*******************************************************************************/
float invSqrt(float x) 
{
	 float halfx = 0.5f * x;
	 float y = x;
	 long i = *(long*)&y;
	 i = 0x5f3759df - (i>>1);
	 y = *(float*)&i;
	 y = y * (1.5f - (halfx * y * y));
	 return y;
}


