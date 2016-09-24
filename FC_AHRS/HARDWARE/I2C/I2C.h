#ifndef _I2C_H
#define _I2C_H
#include "sys.h"
#include "stdbool.h"

void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Write(u8 data);
u8  I2C1_Read(void);

void I2C1SdData(u8 slaveID, u8 Addr, u8 data);
void I2C1SdConf(u8 slaveID, u8 reg);
void I2C1RdSeveralData(u8 slaveID,u8* pbuffer, uint8_t num);
void I2C1RdData(u8 slaveID, u8 Addr, u8* pbuffer);
void I2C1MulRdData(uint8_t slaveID, uint8_t Addr,  uint8_t num,uint8_t* pbuffer);
void I2C1_Init(void);
void I2C2_Init(void);
void I2C2RdData(u8 slaveID, u8 Addr, u8* pbuffer);
void I2C2SdData(u8 slaveID, u8 Addr, u8 data);
bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data);
bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
int8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
int8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
#endif



