#include "includes.h"
#include "GlobalVariable.h"
#include "sys_params.h"

__align(8) static  CPU_STK  ControlTaskStk[CONTROL_TASK_STK_SIZE];
OS_TCB  ControlTaskTCB;


void ControlTask(void *p_arg)
{
  CPU_FP32 dt = 0;
  CPU_TS32 timestamp_old;

  EULER euler_att;

  /* attitude control */
  CTRL_DataType control_data;
  MOTOR_DataType motor_data;
  EULER target_euler;
  float throttle;
  VECTOR gyro_rate;
  VECTOR target_rate;

  (void)p_arg;

  timestamp_old = CPU_TS_Get32();

  while (1)
  {
    BSP_OS_SemWait(&Sem_AttControlUpdate, 0);
    //LOGIC_DBG_PIN1_H;

    dt = Get_dt(&timestamp_old);

    /* copy data */
    euler_att.yaw = g_AttEuler.yaw;
    euler_att.pitch = g_AttEuler.pitch;
    euler_att.roll = g_AttEuler.roll;
    VectCopy(&g_GyroRaw, &gyro_rate);

    /* Get target Euler */
    target_euler.yaw   = g_RCEuler.yaw + g_PosEuler.yaw;
    target_euler.pitch = g_RCEuler.pitch + g_PosEuler.pitch;
    target_euler.roll  = g_RCEuler.roll + g_PosEuler.roll;

    /* update throttle */
    if (g_FlagPosHoldZ == SET)
      throttle = Constraint_f(g_PosThrottle, 0, g_sys_params.max_F*0.8f);
    else
      throttle = Constraint_f(g_RCThrottle*g_sys_params.max_F, 0, g_sys_params.max_F*0.8f);

    /* Start contorl */
    if (g_FlagArmed == RESET) {

      if (g_FlagTakeOff == SET) {				
        CTRL_GetTargetRate(target_euler, euler_att, &target_rate); /* angle contorl */
        CTRL_RateController(target_rate, gyro_rate, &control_data,g_sys_params,dt, SET); /* rate contorl */
        CTRL_SetThrottle(throttle, &control_data);
      }
      /* still not take off */
      else {
        CTRL_RateController(target_rate, gyro_rate, &control_data,g_sys_params,dt, RESET);
        CTRL_SetThrottle(throttle, &control_data);
      }
      //control_data.throttle = g_RCThrottle*g_sys_params.max_F;
			//control_data.mx = control_data.my = control_data.mz = 0;
      Motor_SetPWM(&control_data, &motor_data,g_sys_params);
    }
    else
      Motor_MinThrust(&motor_data);
		
    /* End control */

    //LOGIC_DBG_PIN1_L;
  }
}

void ControlTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&ControlTaskTCB,
               (CPU_CHAR   *)"Control Task",
               (OS_TASK_PTR )ControlTask,
               (void       *)0,
               (OS_PRIO     )CONTROL_TASK_PRIO,
               (CPU_STK    *)&ControlTaskStk[0],
               (CPU_STK_SIZE)CONTROL_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)CONTROL_TASK_STK_SIZE,
               (OS_MSG_QTY  )5u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate Control Task...\n\r"));
  }
  else
    for (;;);
}
