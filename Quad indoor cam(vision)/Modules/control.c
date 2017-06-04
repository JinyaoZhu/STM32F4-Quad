#include "bsp.h"
#include "control.h"
#include "motor.h"
#include "GlobalVariable.h"

#define ERROR_RATE_I_LIMIT   (100.0f)

void CTRL_SetKpAngle(float param)
{

}

void CTRL_SetKpRate(float param)
{

}
/*
***********************************************************************
                       CTRL_SetThrottle()
***********************************************************************
*/
void CTRL_SetThrottle(float t, CTRL_DataType *control_data)
{
  control_data->throttle = Constraint_f(t, 0, 0.8f*g_sys_params.max_F); /* avoid throttle saturate */
}

/*
***********************************************************************
                       CTRL_GetTargetRate()
***********************************************************************
*/
void CTRL_GetTargetRate(const EULER euler_target, const EULER euler, VECTOR *target_rate)
{
  const VECTOR P = {14.0, 14.0, 26};

  VECTOR error;
  VECTOR error_tmp;
  VECTOR rate;

  error.x = WarpToPI(euler_target.roll - euler.roll);
  error.y = WarpToPI(euler_target.pitch - euler.pitch);
  error.z = WarpToPI(euler_target.yaw - euler.yaw);

  error_tmp.x = P.x * error.x;
  error_tmp.y = P.y * error.y;
  error_tmp.z = P.z * error.z;

  rate.x =  error_tmp.x - arm_sin_f32(euler.pitch);
  rate.y =  error_tmp.y * arm_cos_f32(euler.roll) + error_tmp.z * arm_cos_f32(euler.pitch) * arm_sin_f32(euler.roll);
  rate.z = -error_tmp.y * arm_sin_f32(euler.roll) + error_tmp.z * arm_cos_f32(euler.pitch) * arm_cos_f32(euler.roll);

  target_rate->x = Constraint_f(rate.x,-6.28,6.28);
  target_rate->y = Constraint_f(rate.y,-6.28,6.28);
  target_rate->z = Constraint_f(rate.z,-6.28,6.28);
}


/*
***********************************************************************
                       CTRL_RateController()
***********************************************************************
*/
void CTRL_RateController(const VECTOR target_rate, const VECTOR gyro_rate,
                         CTRL_DataType *control_data,sys_params_t sys_params,const float dt, uint8_t en_flag)
{
//   const VECTOR P  = {0.025, 0.025, 0.10};
//   const VECTOR I  = {0.010, 0.010, 0.004};
//   const VECTOR D  = {0.0018, 0.0018, 0.0032};
	const VECTOR P  = {65.0, 65.0, 250.0};
  const VECTOR I  = {23.00, 23.00,13.13};
  const VECTOR D  = {4.8, 4.8, 10.50};

  static VECTOR last_error = {0};
  static VECTOR error = {0};
  static VECTOR d_error = {0};
  static VECTOR i_error = {0};

  static VECTOR M; /* torque */

  error.x =  DLPF(target_rate.x - gyro_rate.x, error.x, 30.0f * HZ2RAD, dt);
  error.y =  DLPF(target_rate.y - gyro_rate.y, error.y, 30.0f * HZ2RAD, dt);
  error.z =  DLPF(target_rate.z - gyro_rate.z, error.z, 30.0f * HZ2RAD, dt);

  d_error.x = DLPF((error.x - last_error.x) / dt, d_error.x, 15.0f * HZ2RAD, dt);
  d_error.y = DLPF((error.y - last_error.y) / dt, d_error.y, 15.0f * HZ2RAD, dt);
  d_error.z = DLPF((error.z - last_error.z) / dt, d_error.z, 15.0f * HZ2RAD, dt);
  last_error.x = error.x;
  last_error.y = error.y;
  last_error.z = error.z;

  i_error.x = Constraint_f(i_error.x + dt * error.x, -ERROR_RATE_I_LIMIT, ERROR_RATE_I_LIMIT);
  i_error.y = Constraint_f(i_error.y + dt * error.y, -ERROR_RATE_I_LIMIT, ERROR_RATE_I_LIMIT);
  i_error.z = Constraint_f(i_error.z + dt * error.z, -ERROR_RATE_I_LIMIT, ERROR_RATE_I_LIMIT);

  /* EN flag */
  if (en_flag == SET) {
    M.x = sys_params.inertia_xx*(P.x * error.x + I.x * i_error.x + D.x * d_error.x);
    M.y = sys_params.inertia_yy*(P.y * error.y + I.y * i_error.y + D.y * d_error.y);
    M.z = sys_params.inertia_zz*(P.z * error.z + I.z * i_error.z + D.z * d_error.z);
  }
  else {
    M.x = M.y = M.z = 0;
    i_error.x = i_error.y = i_error.z = 0;
		d_error.x = d_error.y = d_error.z = 0;
		error.x = error.y = error.z = 0;
  }

  control_data->mx = M.x;
  control_data->my = M.y;
  control_data->mz = M.z;
}
