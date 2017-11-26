#include "includes.h"
#include "GlobalVariable.h"
#include "sensor_update.h"
#include "algorithm.h"

__align(8) static  CPU_STK  SonarTaskStk[SONAR_TASK_STK_SIZE];
OS_TCB  SonarTaskTCB;



void SonarTask(void *p_arg)
{
  float dt;
  CPU_TS32 timestamp_old = 0;
	float sonar_raw;
	float last_sonar_raw;
	
	(void)p_arg;

	
	
  while (1)
  {
		dt = Get_dt(&timestamp_old);
		if(Sonar_GetDistance(&sonar_raw) == DEF_OK){
			if((sonar_raw > 0.0f)&&(sonar_raw < 3.0f)&&((sonar_raw-last_sonar_raw)/dt)<0.5f){
			  g_SonarDist = DLPF(sonar_raw,g_SonarDist,5*HZ2RAD,dt);
				if((fabs(g_RCEuler.roll) < 10.0f*DEG2RAD)&&(fabs(g_RCEuler.pitch) < 10.0f*DEG2RAD))
					g_FlagSonarUpdate = SET;
			}
    }
		last_sonar_raw = sonar_raw;
    BSP_OS_TimeDlyMs(50);
  }
}






void SonarTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&SonarTaskTCB,
               (CPU_CHAR   *)"SonarTask",
               (OS_TASK_PTR )SonarTask,
               (void       *)0,
               (OS_PRIO     )SONAR_TASK_PRIO,
               (CPU_STK    *)&SonarTaskStk[0],
               (CPU_STK_SIZE)SONAR_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)SONAR_TASK_STK_SIZE,
               (OS_MSG_QTY  )5u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate Sonar Task...\n\r"));
  }
	else
		for(;;);
}
