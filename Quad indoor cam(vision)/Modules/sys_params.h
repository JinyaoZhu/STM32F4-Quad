#ifndef __SYS_PARAMS_H__
#define __SYS_PARAMS_H__


typedef struct{
  float length;
	float mass;
	float gravity;
	float inertia_xx;
	float inertia_yy;
	float inertia_zz;
	float max_F;
	float min_F;
	float factor_pwm2f;
}sys_params_t;

extern sys_params_t g_sys_params;



#endif /*__SYS_PARAMS_H__*/
