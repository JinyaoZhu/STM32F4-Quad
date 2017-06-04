#ifndef __CORIDC_H__

#define __CORDIC_H__

#include "bsp.h"

#ifndef PI
#define PI (3.1415926585f)
#endif

void CORDIC_Init(void);
void CORDIC_SinCos(uint32_t theta, int32_t *sin, int32_t *cos);
float CORDIC_Atan2(float y_val, float x_val);
float fast_sin(float x);
float fast_cos(float x);
void fast_SinCos(float x, float *sin, float *cos);
float fast_atan2(float y,float x);

#endif /* __CORDIC_H__ */
