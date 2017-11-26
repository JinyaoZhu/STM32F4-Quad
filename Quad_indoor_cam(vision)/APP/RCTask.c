#include "includes.h"
#include "GlobalVariable.h"
#include "rc.h"

__align(8) static  CPU_STK  RCTaskStk[RC_TASK_STK_SIZE];
OS_TCB  RCTaskTCB;



void RCTask(void *p_arg)
{
  float dt = 0;
  CPU_TS32 timestamp_old = 0;
  PWM_IN_DataType pwm_in_data;
  RC_DataType rc_data;
  /* arm */
  float rc_arm_wait_time = 0;

  /* set position */
  float last_throttle = 0;
  float set_height_wait_time = 0;

  (void)p_arg;

  timestamp_old = CPU_TS_Get32();

  while (1)
  {
    dt = Get_dt(&timestamp_old);

    if (BSP_PWM_IN_GetWidth(&pwm_in_data) == DEF_OK) {
      rc_data.ch1 = Constraint_f((pwm_in_data.ch1_width - 0.001f) / 0.001f, 0, 1);
      rc_data.ch2 = Constraint_f((pwm_in_data.ch2_width - 0.001f) / 0.001f, 0, 1);
      rc_data.ch3 = Constraint_f((pwm_in_data.ch3_width - 0.001f) / 0.001f, 0, 1);
      rc_data.ch4 = Constraint_f((pwm_in_data.ch4_width - 0.001f) / 0.001f, 0, 1);
      rc_data.ch5 = Constraint_f((pwm_in_data.ch5_width - 0.001f) / 0.001f, 0, 1);
      rc_data.ch6 = Constraint_f((pwm_in_data.ch6_width - 0.001f) / 0.001f, 0, 1);
      rc_data.ch7 = Constraint_f((pwm_in_data.ch7_width - 0.001f) / 0.001f, 0, 1);
      rc_data.ch8 = Constraint_f((pwm_in_data.ch8_width - 0.001f) / 0.001f, 0, 1);
      rc_GetTarget(&rc_data, &g_RCEuler, dt);
      g_RCThrottle = rc_data.throttle;
    }

    /* arm or disarm */
    if ((rc_data.ch3 < 0.10f) && (rc_data.ch4 < 0.10f) && (rc_data.ch2 < 0.10f)
        && (rc_data.ch1 > 0.80f) && (g_FlagAttInit == SET)) {
      /* wait 2 seconds */
      if (rc_arm_wait_time < 2.0f) {
        rc_arm_wait_time += dt;
      }
      else {
        rc_arm_wait_time = 0;
        /* toggle g_FlagArmed */
        if (g_FlagArmed == 0)
          g_FlagArmed = SET;
        else {
          g_RCEuler.yaw = g_AttEuler.yaw;
          g_FlagArmed = RESET;
        }
      }
    }
    else
      rc_arm_wait_time = 0;


    /* Set Height Mode */
    if ((fabs(rc_data.throttle - last_throttle) < 0.005f) && (g_FlagTakeOff == SET) && (rc_data.aux1 < 0.8f)) {
      if (set_height_wait_time < 2) {
        set_height_wait_time += dt;
      }
      else if (g_FlagPosHoldZ == RESET) {
        g_TargetPos.x = g_PosRef.x;
        g_TargetPos.y = g_PosRef.y;
        g_TargetPos.z = g_PosRef.z;
// 				g_TargetPos.x = 0;
//         g_TargetPos.y = 0;
//         g_TargetPos.z = 0.3;
				
        g_FlagPosHoldZ = SET;
        if (rc_data.aux1 < 0.2f)
          g_FlagPosHoldXY = SET;
      }
    }
    else {
      g_FlagPosHoldZ = RESET;
      g_FlagPosHoldXY = RESET;
      set_height_wait_time = 0;
    }
    last_throttle = rc_data.throttle;
		
		//if((rc_data.aux2>0.5f)&&(g_FlagTriggerStick == RESET)) g_FlagTriggerStick = SET;
		
		//g_TargetPos.x = DLPF(2.0f*(rc_data.aux2 - 0.5f),g_TargetPos.x,3*HZ2RAD,dt);
// 		g_TargetPos.z = DLPF(rc_data.aux2 *1.2f,g_TargetPos.z,2*HZ2RAD,dt);
// 		{
//       static float last_target_pos;
// 			g_TargetVel.z = Constraint_f(DLPF((g_TargetPos.z-last_target_pos)/dt,g_TargetVel.z,2*HZ2RAD,dt),-1,1);
// 			last_target_pos = g_TargetPos.z;
//     }
//	  g_TargetPos.x = DLPF(2.0f*(rc_data.aux2 - 0.5f),g_TargetPos.x,2*HZ2RAD,dt);
// 		{
//       static float last_target_pos;
// 			g_TargetVel.x = Constraint_f(DLPF((g_TargetPos.x-last_target_pos)/dt,g_TargetVel.x,2*HZ2RAD,dt),-1,1);
// 			last_target_pos = g_TargetPos.x;
//     }
		
// 		if(rc_data.aux2 < 0.5f)
// 			g_TargetPos.z = 0.2;
// 		else
// 			g_TargetPos.z = 0.4;
	
				
    /* takeoff flag */
    if ((g_RCThrottle > 0.15f)&&(g_FlagArmed == RESET))
      g_FlagTakeOff = SET;
    else
      g_FlagTakeOff = RESET;
  }
}


void RCTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&RCTaskTCB,
               (CPU_CHAR   *)"RCTask",
               (OS_TASK_PTR )RCTask,
               (void       *)0,
               (OS_PRIO     )RC_TASK_PRIO,
               (CPU_STK    *)&RCTaskStk[0],
               (CPU_STK_SIZE)RC_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)RC_TASK_STK_SIZE,
               (OS_MSG_QTY  )5u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate RC Task...\n\r"));
  }
  else
    for (;;);
}
