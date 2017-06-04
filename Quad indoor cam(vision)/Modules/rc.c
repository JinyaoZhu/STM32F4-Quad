#include "bsp.h"
#include "rc.h"
#include "GlobalVariable.h"

#define MAX_PITCH_RAD  (20*DEG2RAD)
#define MAX_ROLL_RAD   (20*DEG2RAD)

/*
******************************************************************
                      rc_GetTarget()
  Description : Convert rc channel data to target quaternion and euler
******************************************************************
*/
void  rc_GetTarget(RC_DataType * p, EULER *target_e,float dt)
{
  float roll_in, pitch_in, yaw_in;
  float throttle_in;
	
	p->throttle = p->ch3;
  p->yaw = p->ch4 - 0.5f;
  p->pitch = (p->ch2 - 0.5f);
  p->roll = (p->ch1 - 0.5f);
  p->aux1 = p->ch5;
  p->aux2 = p->ch6;
  p->aux3 = p->ch7;
	p->aux4 = p->ch8;
	
  throttle_in = p->throttle;
  yaw_in = p->yaw;

  /* get target roll and pitch */
  pitch_in = (p->pitch) * MAX_PITCH_RAD * 2 ;
  roll_in  = (p->roll) * MAX_ROLL_RAD * 2  ;

  if ((fabs(yaw_in) > 0.05f) && (throttle_in > 0.20f)) {
    target_e->yaw = WarpToPI(target_e->yaw - yaw_in * dt * PI);/* 180 degree/s */
  }

  target_e->pitch = pitch_in;
  target_e->roll  = roll_in;
}

