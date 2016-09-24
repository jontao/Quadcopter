#ifndef _MS5611_H
#define _MS5611_H
#include "sys.h"

#define MS5611_ADDR  0xEE
#define MS5611_ADC_RD          0x00
#define	MS5611_PROM_RD 	       0xA0
#define MS5611_PROM_CRC        0xAE
#define MS5611_RST             0x1E  //cmd ¸´Î»
#define	MS561101BA_D2_OSR_4096   0x58	// 9.04 mSec conversion time ( 110.62 Hz)
#define	MS561101BA_D1_OSR_4096   0x48
#endif
void MS5611_Init(void);
void MS561101BA_getTemperature(uint8_t OSR_Temp);
