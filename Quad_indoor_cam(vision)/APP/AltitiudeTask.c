#include "includes.h"
#include "GlobalVariable.h"


__align(8) static  CPU_STK  AltTaskStk[ALT_TASK_STK_SIZE];
OS_TCB  AltTaskTCB;


/*******************************************************************************************************/
void AltTask(void *p_arg)
{
	CPU_FP32 dt = 0;
	CPU_TS32 timestamp_old = 0;
	CPU_FP32 baro_last_alt = 0;
	CPU_FP32 baro_vel = 0;
	CPU_FP32 baro_alt = 0;
	CPU_INT16U alt_offset_cnt = 0;
	CPU_FP32 alt_offset_accumulate = 0;
	CPU_FP32 sonar_alt = 0;
	CPU_FP32 sonar_temp = 0;
	CPU_FP32 sonar_alt_last = 0;
	CPU_FP32 sonar_vel = 0;
	CPU_FP32 est_vel;
	CPU_INT32U loop_cnt = 0;
	
  (void)p_arg;
	
	/* wait sensors init */
	while(g_AllSensors_InitFlag == 0) BSP_OS_TimeDlyMs(10);
	
	
	alt_offset_cnt = 0;
	while(g_AttitudeTask_InitFlag == 0){
		alt_offset_accumulate += MS5611_GetAltitude();
		alt_offset_cnt++;
	}
	
	MS5611_SetGroundOffset(alt_offset_accumulate/alt_offset_cnt);
			
  while (1)
  {
    dt = Get_dt(&timestamp_old);
		
		/* Lowpass filter */
		baro_alt = DLPF(MS5611_GetAltitude(),baro_alt,2.0f*HZ2RAD,dt);
		
		if(loop_cnt >= 2){
			loop_cnt = 0;
			if(Sonar_GetDistance(&sonar_temp) == DEF_FAIL)
				sonar_alt = -1;
			else{
				sonar_alt = DLPF(sonar_temp,sonar_alt,2.0f*HZ2RAD,3.0f*dt);
				sonar_vel = DLPF((sonar_alt - sonar_alt_last)/(3.0f*dt),sonar_vel,2.0f*HZ2RAD,3.0f*dt);
				sonar_alt_last = sonar_alt;
				g_Sonar_AltEst = sonar_alt;
			}
		}
		else
			loop_cnt++;
	
		baro_vel = DLPF((baro_alt - baro_last_alt)/dt,baro_vel,0.5f*HZ2RAD,dt);
		
		baro_last_alt = baro_alt;
		
		/* if sonar is in range */
		if((sonar_alt > 0.01f)&&(sonar_alt<2.5f)&&(max(fabs(g_ATT_E.pitch),fabs(g_ATT_E.roll))<15.0f*DEG2RAD)){
	    g_AltEst = sonar_alt;
			g_SonarInRange_Flag = 1;
		}
		else{
      g_SonarInRange_Flag = 0;
    }

	  est_vel = baro_vel;
		
	  g_VelEst = est_vel;
  }
}

/*******************************************************************************************************/
void AltTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&AltTaskTCB,
               (CPU_CHAR   *)"Alt Task",
               (OS_TASK_PTR )AltTask,
               (void       *)0,
               (OS_PRIO     )ALT_TASK_PRIO,
               (CPU_STK    *)&AltTaskStk[0],
               (CPU_STK_SIZE)ALT_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)ALT_TASK_STK_SIZE,
               (OS_MSG_QTY  )0u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate AltTask Task...\n\r"));
  }
}
