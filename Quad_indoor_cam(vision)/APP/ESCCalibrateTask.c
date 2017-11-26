#include "includes.h"
#include "GlobalVariable.h"

__align(8) static  CPU_STK  ESCCalibrateTaskStk[ESC_CAL_TASK_STK_SIZE];
OS_TCB  ESCCalibrateTaskTCB;


/*******************************************************************************************************/
void ESCCalibrateTask(void *p_arg)
{
	OS_ERR err;
	MOTOR_DataType motor_data;
	(void)p_arg;
  
	Motor_MaxThrust(&motor_data);
	BSP_OS_TimeDlyMs(3000);
	Motor_MinThrust(&motor_data);
	BSP_OS_TimeDlyMs(3000);

	g_Flag_ESC_Calibrated = SET;
	
	OSTaskDel(&ESCCalibrateTaskTCB,&err);      

  while (1)
  {
    BSP_OS_TimeDlyMs(100);
  }
}


/*******************************************************************************************************/
void ESCCalibrateTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&ESCCalibrateTaskTCB,
               (CPU_CHAR   *)"ESCCalibrateTask",
               (OS_TASK_PTR )ESCCalibrateTask,
               (void       *)0,
               (OS_PRIO     )ESC_CAL_TASK_STK_PRIO,
               (CPU_STK    *)&ESCCalibrateTaskStk[0],
               (CPU_STK_SIZE)ESC_CAL_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)ESC_CAL_TASK_STK_SIZE,
               (OS_MSG_QTY  )0u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate ESCCalibrateTask Task...\n\r"));
  }
}
