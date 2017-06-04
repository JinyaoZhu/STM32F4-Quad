#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "algorithm.h"
#include "sys_params.h"

typedef struct
{
	float throttle;
	float mx;
	float my;
  float mz;
}CTRL_DataType;


void CTRL_SetKpAngle(float param);

void CTRL_SetKpRate(float param);

void CTRL_SetThrottle(float t,CTRL_DataType *control_data);

void CTRL_GetTargetRate(const EULER euler, const EULER euler_target,VECTOR *target_rate);

void CTRL_RateController(const VECTOR target_rate,const VECTOR gyro_rate,CTRL_DataType *control_data,sys_params_t sys_params,
                     const float dt,uint8_t en_flag);

#endif /* __CONTROL_H__ */
