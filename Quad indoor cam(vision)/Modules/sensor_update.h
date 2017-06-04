#ifndef __SENSOR_UPDATE__
#define __SENSOR_UPDATE__

#include "algorithm.h"

#define G_VALUE (9.8f)

#define GEO_MAG_GAUSS         (0.4f)

typedef struct
{
  VECTOR gyro;
	VECTOR acc;
}InertiaDataType;


typedef struct
{
  VECTOR mag;
}CompassDataType;

void SensorUpdate_Init(void);

void SensorUpdate_SetAccOffset(float x,float y,float z);

void SensorUpdate_SetAccGain(float x,float y,float z);

void SensorUpdate_SetGyroOffset(float x,float y,float z);

void SensorUpdate_SetMagOffset(float x,float y,float z);

uint8_t SensorUpdate_GetGyroAccRaw(VECTOR *g,VECTOR *a);

uint8_t SensorUpdate_GetMagRaw(VECTOR *mag);

void Sensor_Update_GetOffsetData(void);

void Sensor_Update_StoreOffsetData(void);

void SensorUpdate_UpdateAlignRotMat(const EULER euler);

#endif /* __SENSOR_UPDATE__ */
