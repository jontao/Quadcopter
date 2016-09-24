
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
 	Init_Power_Ctrl();       //����STM32F4�ɿص�Դ����
  EXTI_Init();
	 Init_Led();
/*84000/420=200khz T=0.005ms pwmƵ��=tim_frequency/(TIMX_ARR-1)=200/1000=0.200khz*/
  TIM2_PWM_Init(1000,419);
#ifdef UART_ENABLE
	 uart_init(84,115200);
#else
	 TIM4_PWM_Init(1000,419);
#endif
		 
	 TIM5_Int_Init(0xffff,83);
	 
	 motor_on_off(MOTOR_START);//������������ڴ���״̬
	 NRF24L01_Init();	
  while(NRF24L01_Check()){LED1_ON;delay_ms(500);;LED1_OFF;delay_ms(500);} //���ң�� 
	 LED1_ON;
		InitMPU6050();
  Init_HMC5883L();
		LED2_ON;
	 get_gyro_bias();         //У�������� 
	 init_quaternion();       //��ʼ����Ԫ��
		LED3_ON;
  PID_controllerInit();    //��ʼ��PID
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
		   init_gx=((float)gyro[0] - Gyro_Xout_Offset) *0.061*0.0174;//* 0.000266;    //��λת���ɣ�����/s��0.000266=1/(Gyro_500_Scale_Factor * 57.295780)
		   init_gy=((float)gyro[1] - Gyro_Yout_Offset) *0.061*0.0174;//* 0.000266;
	   	init_gz=((float)gyro[2] - Gyro_Zout_Offset) *0.061*0.0174;//* 0.000266;
					
					Gyro_FinalX=init_gx;
					Gyro_FinalY=init_gy;
					Gyro_FinalZ=init_gz;
					
					
		  //accel��Ȩƽ���˲�������������Ӱ��
		   ACC_X_BUF[filter_cnt] = init_ax;//���»�����������
	    ACC_Y_BUF[filter_cnt] = init_ay;
	    ACC_Z_BUF[filter_cnt] = init_az;
	    for(i=0;i<FILTER_NUM;i++)
	    {
		      temp[0] += ACC_X_BUF[i];
		      temp[1] += ACC_Y_BUF[i];
		      temp[2] += ACC_Z_BUF[i];
	    }
	    init_ax = temp[0] * 0.05*0.244*0.0098;     // 0.05�൱�ڳ���FILTER_NUM����������ٶ�
	    init_ay = temp[1] * 0.05*0.244*0.0098;     
	    init_az = temp[2] * 0.05*0.244*0.0098;     
	    filter_cnt++;
	    if(filter_cnt==FILTER_NUM)	filter_cnt=0;
//     printf("gx=%f, gy=%f, gz=%f, ax=%f, ay=%f, az=%f \n\r", init_gx,init_gy,init_gz, init_ax,init_ay,init_az);

        mpu_get_compass_reg(mag);  //��ȡcompass����
        //����x y���У׼��δ��z�����У׼���ο�ST��У׼���� 
        init_mx =((float)mag[0]) - 8;						
        init_my =((float)mag[2] * 1.046632) - 1.569948;
        init_mz =(float)-mag[1];

//		    printf(" mx=%f, my=%f, mz=%f \n\r", init_mx,init_my,init_mz);
        AHRSupdate(init_gx, init_gy, init_gz, init_ax, init_ay, init_az, init_mx, init_my, init_mz); 
     } 
				 Recev_nRF();
     PID_CAL();//PID����
     //UART1_ReportIMU(Euler_Yaw*10, Euler_Pitch*10, Euler_Roll*10,0,0,0,0);
     //delay_ms(10);
  }
}
/*******************************************************************************
* Function Name  : init_quaternion
* Description    : �����ʼ����Ԫ��q0 q1 q2 q3.
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
	  	    
	  init_ax=((float)accel[0]) *0.244*0.0098 ;// /Accel_4_Scale_Factor;	   //��λת�����������ٶȵĵ�λ��g
	  init_ay=((float)accel[1])*0.244*0.0098; // / Accel_4_Scale_Factor;
   init_az=((float)accel[2])*0.244*0.0098;//  / Accel_4_Scale_Factor;
	 // printf("ax=%f, ay=%f, az=%f  \n\r", init_ax, init_ay, init_az);
	  for(i=0;i<5;i++)   //��һ�ζ�ȡ��compsaa�����Ǵ�ģ����Ҫ������α�֤�Ժ�������ȷ��оƬbug
	  {
        mpu_get_compass_reg(mag);  //��ȡcompass����
        //����x y���У׼��δ��z�����У׼���ο�MEMSense��У׼���� 
        init_mx =((float)mag[0]) - 8;						
        init_my =((float)mag[2]) * 1.046632 - 1.569948;
        init_mz =(float)-mag[1];
    //    printf("mx=%f, my=%f, mz=%f \n\r", init_mx, init_my, init_mz);
	  }
//������y��Ϊǰ������    
init_Roll = -atan2(init_ax, init_az);    //����ĵ�λ�ǻ��ȣ�����Ҫ�۲���Ӧ����57.295780ת��Ϊ�Ƕ�
init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
                   init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//������atan2(my, mx)�����е�init_Roll��init_Pitch�ǻ���
				            
//����ʼ��ŷ����ת���ɳ�ʼ����Ԫ����ע��sin(a)��λ�õĲ�ͬ������ȷ����xyz��ת����Pitch����Roll����Yaw������ZXY˳����ת,Qzyx=Qz*Qy*Qx�����е�init_YawRollPtich�ǽǶ�        
q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��pitch
q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��roll
q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw

//������x��Ϊǰ������
//  init_Roll  = atan2(init_ay, init_az);
//  init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
//  init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
//                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);
//  q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
//  q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��roll
//  q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��pitch
//  q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw

init_Roll  = init_Roll * 57.295780;	 //����ת�Ƕ�
init_Pitch = init_Pitch * 57.295780;
init_Yaw   = init_Yaw * 57.295780;
if(init_Yaw < 0){init_Yaw = init_Yaw + 360;}      //��Yaw�ķ�Χת��0-360
if(init_Yaw > 360){init_Yaw = init_Yaw - 360;} 	
//while(1){
//printf("��ʼ����Ԫ����Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw, init_Pitch, init_Roll);
//delay_ms(500);
//} 
  }
}

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag���ں��㷨��Դ��S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3��Ҫ��ʼ�����ܴ��뵽����ĳ����У�����ֱ��ʹ��1 0 0 0��������ļ��㣬��������Ϊ��
// 1.����У׼accle gyro mag��
// 2.����init_quaternion������1��accle��xyz�����ݣ������ù�ʽ�������ʼ��ŷ���ǣ�
//   ����ACCEL_1G=9.81����λ����m/s2����init_Yaw�����ô����Ƽ��������
// 3.�����Լ��Ĳ������ڣ�������halfT��halfT=��������/2����������Ϊִ��1��AHRSupdate���õ�ʱ�䣻
// 4.��2�м������ŷ����ת��Ϊ��ʼ������Ԫ��q0 q1 q2 q3���ںϼ��ٶȼƣ������ǣ�������º��ŷ����pitch��roll��Ȼ��ʹ��pitch roll�ʹ����Ƶ����ݽ��л����˲��ںϵõ�Yaw������ʹ�ã�����ŷ��������㣻
// 5.��ֱ��ʹ����Ԫ����
// 6.�ظ�4�����ɸ�����̬;

//�ܵ���˵�������������ǣ����ٶȼ�������������Pitch��Roll��������������������Yaw;
//���³����У�gx, gy, gz��λΪ����/s��ax, ay, azΪ���ٶȼ������ԭʼ16��������, mx, my, mzΪ�����������ԭʼ16�������ݣ�
//ǰ������mpu9150�ļ��ٶȼƺ������ǵ�x��Ϊǰ������;
//���³�����õĲο�����Ϊ��mpu9150�ļ��ٶȼƺ���������ָ��xyz����Ϊ������

//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
//Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)

//ŷ���ǵ�λΪ����radian������57.3�Ժ�ת��Ϊ�Ƕ�,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
////   float norm, halfT;
////   float hx, hy, hz, bz, by;
////   float vx, vy, vz, wx, wy, wz;
////   float ex, ey, ez;
////   float Pitch, Roll, Yaw;

/////*����֮��ĳ���ʹ�ã����ټ���ʱ��*/
////   //auxiliary variables to reduce number of repeated operations��
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
/////*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
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
/////*�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ������������Ǵӷ���������ϵ����������ϵ��ת����ʽ*/
////   //compute reference direction of flux
////   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
////   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
////   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/////*�����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
////��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣����Ҷ���byָ������������by=ĳֵ��bx=0
////������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
////�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
////�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
////��Ϊbx=0�����Ծͼ򻯳�(by*by)  = ((hx*hx) + (hy*hy))�������by�������޸�by��bxָ����Զ����ĸ���ָ������*/
//////   bx = sqrtf((hx*hx) + (hy*hy));
////   by = sqrtf((hx*hx) + (hy*hy));
////   bz = hz;        
////    
////   // estimated direction of gravity and flux (v and w)����������Ǵ���������ϵ������������ϵ��ת����ʽ(ת�þ���)
////   vx = 2*(q1q3 - q0q2);
////   vy = 2*(q0q1 + q2q3);
////   vz = q0q0 - q1q1 - q2q2 + q3q3;

/////*���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
////��Ϊbx=0�����������漰��bx�Ĳ��ֶ���ʡ���ˡ�ͬ��by=0�����������漰��by�Ĳ���Ҳ���Ա�ʡ�ԣ�������Լ������Ǹ���ָ���йء�
////������������vxyz�����㣬��Ϊ����g��az=1��ax=ay=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
////����Կ���������ʽ��wxyz�Ĺ�ʽ����by����ay��0������bz����az��1�����ͱ����vxyz�Ĺ�ʽ�ˣ�����q0q0+q1q1+q2q2+q3q3=1����*/
//////   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//////   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//////   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
////   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
////   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
////   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
////           
//////���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
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

////   halfT=GET_NOWTIME();		//�õ�ÿ����̬���µ����ڵ�һ��
////   
////   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //�ܹؼ���һ�仰��ԭ�㷨û��
////   {
////      // integral error scaled integral gain
////      exInt = exInt + ex*Ki * halfT;			   //���Բ������ڵ�һ��
////      eyInt = eyInt + ey*Ki * halfT;
////      ezInt = ezInt + ez*Ki * halfT;
////      // adjusted gyroscope measurements
////      gx = gx + Kp*ex + exInt;
////      gy = gy + Kp*ey + eyInt;
////      gz = gz + Kp*ez + ezInt;
////   }         

////   // integrate quaternion rate and normalise����Ԫ�������㷨
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
///////*����Ԫ�������Pitch  Roll  Yaw��ֻ������ҪPID����ʱ�Ž���Ԫ��ת��Ϊŷ����
//////����57.295780��Ϊ�˽�����ת��Ϊ�Ƕ�*/
////	Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.295780;  //ƫ���ǣ���z��ת��	
////    if(Yaw < 0 ){Yaw = Yaw + 360;}
////	if(Yaw > 360 ){Yaw = Yaw - 360;}
////	Pitch = asin(2*q2*q3 + 2*q0*q1) * 57.295780; //�����ǣ���x��ת��	 
////    Roll  = -atan2(-2*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.295780; //�����ǣ���y��ת��

/*���������Ԫ�������Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
����57.295780��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.295780; //�����ǣ���y��ת��	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.295780; //�����ǣ���x��ת��
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.295780;  //ƫ���ǣ���z��ת��

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
У׼��������ƫ	
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
���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4�� 	
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


