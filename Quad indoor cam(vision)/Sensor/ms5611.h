#ifndef __MS5611_H__
#define __MS5611_H__

#define  MS561101BA_SlaveAddress 0xee

#define  MS561101BA_D1           0x40 
#define  MS561101BA_D2           0x50 
#define  MS561101BA_RST          0x1E 

#define  MS561101BA_D1_OSR_256   0x40 
#define  MS561101BA_D1_OSR_512   0x42 
#define  MS561101BA_D1_OSR_1024  0x44 
#define  MS561101BA_D1_OSR_2048  0x46 
#define  MS561101BA_D1_OSR_4096  0x48 

#define  MS561101BA_D2_OSR_256   0x50 
#define  MS561101BA_D2_OSR_512   0x52 
#define  MS561101BA_D2_OSR_1024  0x54 
#define  MS561101BA_D2_OSR_2048  0x56 
#define  MS561101BA_D2_OSR_4096  0x58 

#define  MS561101BA_ADC_RD       0x00 
#define  MS561101BA_PROM_RD      0xA2
#define  MS561101BA_PROM_CRC     0xAE 




/***********************PRIVATE************************************/
void MS5611_Reset(void);

uint8_t MS5611_ReadPROM(void);

void MS5611_ReadPressureAndTemperatureData(void);

void MS5611_CalculateTemperature(void);

void MS5611_CalculateCompensatedPressure(void);

/***********************PUBLIC************************************/

uint8_t MS5611_Init(void);

float MS5611_GetAltitude(void);

void MS5611_SetGroundOffset(float offset);

#endif /* __MS5611_H__ */
