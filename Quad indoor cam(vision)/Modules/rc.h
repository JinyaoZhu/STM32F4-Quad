#ifndef __RC_H__
#define __RC_H__

#include "algorithm.h"

typedef struct
{
	float ch1; 
	float ch2; 
	float ch3; 
	float ch4;
	float ch5;
	float ch6;
	float ch7;
	float ch8;
	
	float roll;
	float pitch;
	float throttle;
	float yaw;
	float aux1;
	float aux2;
	float aux3;
	float aux4;
	
}RC_DataType;

void  rc_GetTarget(RC_DataType * p, EULER *target_e,float dt);


#endif /* __RC_H__ */

