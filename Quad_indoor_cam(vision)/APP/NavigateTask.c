#include "includes.h"
#include "GlobalVariable.h"
#include "trajectory.h"

__align(8) static  CPU_STK  NavigateTaskStk[NAVIGATE_TASK_STK_SIZE];
OS_TCB  NavigateTaskTCB;


/*******************************************************************************************************/
void NavigateTask(void *p_arg)
{
	uint32_t i;
	VECTOR last_target_pos;
	VECTOR last_target_vel;
	const float dt = 0.02;
	(void)p_arg;

	while(g_FlagAttInit == RESET) BSP_OS_TimeDlyMs(10);
	
	while(g_FlagPosHoldXY == RESET)  BSP_OS_TimeDlyMs(10);
	
	g_TargetVel.x = 0;
	g_TargetVel.y = 0;
	g_TargetVel.z = 0;
	
	g_TargetAcc.x = 0;
	g_TargetAcc.y = 0;
	g_TargetAcc.z = 0;
	
	last_target_pos.x = Traj_X[0];
	last_target_pos.y = Traj_Y[0];
	last_target_pos.z = Traj_Z[0];
	
	g_PosControlMode = soft; /* position control in soft mode */
	
	last_target_vel.x = last_target_vel.y = last_target_vel.z = 0;
		
	BSP_OS_TimeDlyMs(2000);
	g_TargetPos.x = Traj_X[0];
  g_TargetPos.y = Traj_Y[0];
  g_TargetPos.z = Traj_Z[0];
	BSP_OS_TimeDlyMs(2000);
	g_PosControlMode = stiff; /* shift to stiff mode */
	BSP_OS_TimeDlyMs(2000);	
	
	for(i=0;i<TRAJ_LEN;i++){
		
	  /* target pos */
    g_TargetPos.x = Traj_X[i];
    g_TargetPos.y = Traj_Y[i];
    g_TargetPos.z = Traj_Z[i];
		
		/* target vel */
		g_TargetVel.x = Constraint_f((g_TargetPos.x - last_target_pos.x)/dt,-5.0f,5.0f);
		g_TargetVel.y = Constraint_f((g_TargetPos.y - last_target_pos.y)/dt,-5.0f,5.0f);
		g_TargetVel.z = Constraint_f((g_TargetPos.z - last_target_pos.z)/dt,-5.0f,5.0f);
		/* target acc */
		g_TargetAcc.x = Constraint_f((g_TargetVel.x - last_target_vel.x)/dt,-7.0f,7.0f);
		g_TargetAcc.y = Constraint_f((g_TargetVel.y - last_target_vel.y)/dt,-7.0f,7.0f);
		g_TargetAcc.z = Constraint_f((g_TargetVel.z - last_target_vel.z)/dt,-7.0f,7.0f);
		
		last_target_vel.x = g_TargetVel.x;
		last_target_vel.y = g_TargetVel.y;
		last_target_vel.z = g_TargetVel.z;
		
	  last_target_pos.x = g_TargetPos.x;
	  last_target_pos.y = g_TargetPos.y;
	  last_target_pos.z = g_TargetPos.z;
		
		if((i==515)||(i==580)||(i==645)||(i==710)||(i==775)||(i==840)||(i==905)||(i==970)||(i==1035)||(i==1100)||(i==1165)||(i==1230)||(i==1295)||(i==1360)||(i==1425)||(i==1490)||(i==1555)||(i==1620)||(i==1685)||(i==1750)||(i==1815)||(i==1880)||(i==1945)||(i==2010)||(i==2075)||(i==2140)||(i==2205)||(i==2270)||(i==2335)||(i==2400)||(i==2465)||(i==2530)||(i==2595)||(i==2660)||(i==2725)||(i==2790)||(i==2855)||(i==2920)||(i==2985)||(i==3050)||(i==3115)||(i==3180))
		  g_FlagTriggerStick = SET;
		
		BSP_OS_TimeDlyMs(dt*1000);
	}
	g_TargetAcc.x = 0;
	g_TargetAcc.y = 0;
	g_TargetAcc.z = 0;
  g_TargetVel.x = 0;
	g_TargetVel.y = 0;
	g_TargetVel.z = 0;
	BSP_OS_TimeDlyMs(1000);
	g_PosControlMode = soft; /* return to soft mode */

  while (1)
  {
		BSP_OS_TimeDlyMs(50);
  }
}


/*******************************************************************************************************/
void NavigateTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&NavigateTaskTCB,
               (CPU_CHAR   *)"Navigate Task",
               (OS_TASK_PTR )NavigateTask,
               (void       *)0,
               (OS_PRIO     )NAVIGATE_TASK_PRIO,
               (CPU_STK    *)&NavigateTaskStk[0],
               (CPU_STK_SIZE)NAVIGATE_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)NAVIGATE_TASK_STK_SIZE,
               (OS_MSG_QTY  )0u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate Navigate Task...\n\r"));
  }
}
