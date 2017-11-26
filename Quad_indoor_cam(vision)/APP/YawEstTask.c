#include "includes.h"
#include "GlobalVariable.h"
#include "sensor_update.h"

__align(8) static  CPU_STK  YawEstTaskStk[YAW_EST_TASK_STK_SIZE];
OS_TCB  YawEstTaskTCB;


void YawEstTask(void *p_arg)
{
	CPU_FP32 dt = 0;
	CPU_TS32 timestamp_old = 0;
	
	while(g_FlagAttInit == RESET) BSP_OS_TimeDlyMs(10);
	
  while (1)
  {
		dt = Get_dt(&timestamp_old);

		BSP_OS_TimeDlyMs(5);
  }
}






void YawEstTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&YawEstTaskTCB,
               (CPU_CHAR   *)"YawEstTask",
               (OS_TASK_PTR )YawEstTask,
               (void       *)0,
               (OS_PRIO     )YAW_EST_TASK_PRIO,
               (CPU_STK    *)&YawEstTaskStk[0],
               (CPU_STK_SIZE)YAW_EST_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)YAW_EST_TASK_STK_SIZE,
               (OS_MSG_QTY  )0u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate Yaw estimation Task...\n\r"));
  }
	else
		for(;;);
}
