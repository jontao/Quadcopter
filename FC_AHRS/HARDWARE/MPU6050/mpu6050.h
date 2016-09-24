
#ifndef _MPU6050_H
#define _MPU6050_H
#include <math.h>     
#include <stdio.h>
#include "string.h"
#include "sys.h"
#include "I2C.h"

#define	MPU_ADDR      0xD0>>1

#define	SMPLRT_DIV		0x19	
#define	MPUCONFIG			  0x1A	
#define	GYRO_CONFIG		0x1B	
#define	ACCEL_CONFIG	0x1C
#define	BYPASS        0x37
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	USER_CTRL		  0x6A
#define	PWR_MGMT_1		0x6B	
#define	WHO_AM_I			0x75	

typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}S_INT16_XYZ;
extern S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//最新一次读取值
extern S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//零漂
extern u8							GYRO_OFFSET_OK;
extern u8							ACC_OFFSET_OK;

extern void InitMPU6050(void);
//extern int GetData(unsigned char REG_Address);
//extern void READ_MPU6050(short*,short*,short*,short*,short*,short*);
void read_mpu6050(void);
extern volatile float Angle;
short GetData(uchar REG_Address);
char* floatTostr(double floatdat);
				void MPU6050_Dataanl(void);
// extern volatile float Gyro_y;
// extern volatile float Angle;
// extern char BufData[32];
// extern void Init_HMC5883(void);
// extern void Multiple_read_HMC5883(void);
// extern unsigned char BUF[8];
#endif
