#include "includes.h"
#include "GlobalVariable.h"
#include "sensor_update.h"

OS_TCB  AccCalibrateTaskTCB;
__align(8) static  CPU_STK  AccCalibrateTaskStk[ACC_CALIBRATE_TASK_STK_SIZE];



/*******************************************************************************************************/
void AccCalibrateTask(void *p_arg)
{
	VECTOR acc_max;
	VECTOR acc_min;
	CPU_FP32 acc_sum;
  VECTOR acc_gain;
  VECTOR acc_offset;
	EULER imu_align_euler_tmp = {0,0,0};
	
	CPU_INT32U i = 0;
	(void)p_arg; 
	
	while(g_FlagAttInit == 0) BSP_OS_TimeDlyMs(100);
	
	BSP_Ser_Printf("Start acc calibration...\r\n");
	
	BSP_OS_TimeDlyMs(5000);

  /* cancel the align rotaion */
  SensorUpdate_UpdateAlignRotMat(imu_align_euler_tmp);
	
	SensorUpdate_SetAccGain(1,1,1);

  SensorUpdate_SetAccOffset(0,0,0);
	
	/* x max */
	acc_sum = 0;
	g_LEDBlinkPeriod_ms = 500;
	BSP_OS_TimeDlyMs(10000);
	g_LEDBlinkPeriod_ms = 10;
  for(i = 0; i<2500; i++){
		acc_sum += g_AccRaw.x;
		BSP_OS_TimeDlyMs(2);
	}
	acc_max.x = acc_sum/2500.0f;

	/* x min */
	acc_sum = 0;
	g_LEDBlinkPeriod_ms = 500;	
	BSP_OS_TimeDlyMs(10000);
	g_LEDBlinkPeriod_ms = 10;
  for(i = 0; i<2500; i++){
		acc_sum += g_AccRaw.x;
		BSP_OS_TimeDlyMs(2);
	}
	acc_min.x = acc_sum/2500.0f;
	
	/* y max */
	acc_sum = 0;
	g_LEDBlinkPeriod_ms = 500;
	BSP_OS_TimeDlyMs(10000);
	g_LEDBlinkPeriod_ms = 10;
  for(i = 0; i<2500; i++){
		acc_sum += g_AccRaw.y;
		BSP_OS_TimeDlyMs(2);
	}
	acc_max.y = acc_sum/2500.0f;

	/* y min */
	acc_sum = 0;
	g_LEDBlinkPeriod_ms = 500;	
	BSP_OS_TimeDlyMs(10000);
	g_LEDBlinkPeriod_ms = 10;
  for(i = 0; i<2500; i++){
		acc_sum += g_AccRaw.y;
		BSP_OS_TimeDlyMs(2);
	}
	acc_min.y = acc_sum/2500.0f;

	/* z max */
	acc_sum = 0;
	g_LEDBlinkPeriod_ms = 500;
	BSP_OS_TimeDlyMs(10000);
	g_LEDBlinkPeriod_ms = 10;
  for(i = 0; i<2500; i++){
		acc_sum += g_AccRaw.z;
		BSP_OS_TimeDlyMs(2);
	}
	acc_max.z = acc_sum/2500.0f;

	/* z min */
	acc_sum = 0;
	g_LEDBlinkPeriod_ms = 500;	
	BSP_OS_TimeDlyMs(10000);
	g_LEDBlinkPeriod_ms = 10;
  for(i = 0; i<2500; i++){
		acc_sum += g_AccRaw.z;
		BSP_OS_TimeDlyMs(2);
	}
	acc_min.z = acc_sum/2500.0f;	
	

  BSP_Ser_Printf("Sensor Init\n\r");
	
	if(acc_min.x > acc_max.x){
		acc_sum = acc_min.x;
		acc_min.x = acc_max.x;
		acc_max.x = acc_sum;
	}
	
	if(acc_min.y > acc_max.y){
		acc_sum = acc_min.y;
		acc_min.y = acc_max.y;
		acc_max.y = acc_sum;
	}
	
	if(acc_min.z > acc_max.z){
		acc_sum = acc_min.z;
		acc_min.z = acc_max.z;
		acc_max.z = acc_sum;
	}

  acc_gain.x = (2.0f*G_VALUE)/(acc_max.x - acc_min.x);
  acc_gain.y = (2.0f*G_VALUE)/(acc_max.y - acc_min.y);
  acc_gain.z = (2.0f*G_VALUE)/(acc_max.z - acc_min.z);

  acc_offset.x = 0.5f*(acc_max.x + acc_min.x)*acc_gain.x;
  acc_offset.y = 0.5f*(acc_max.y + acc_min.y)*acc_gain.y;
  acc_offset.z = 0.5f*(acc_max.z + acc_min.z)*acc_gain.z;
	
	if((fabs(acc_gain.x - 1.0f) > 0.5f)||(fabs(acc_gain.y - 1.0f) > 0.5f)||(fabs(acc_gain.z - 1.0f) > 0.5f)){
    BSP_Ser_Printf("Acc calibrate error!\r\n");
    for(;;);
  }

  SensorUpdate_SetAccGain(acc_gain.x,acc_gain.y,acc_gain.z);

  SensorUpdate_SetAccOffset(acc_offset.x,acc_offset.y,acc_offset.z);
	
	BSP_Ser_Printf("x max:%f,x min: %f\r\n",acc_max.x,acc_min.x);
	BSP_Ser_Printf("y max:%f,y min: %f\r\n",acc_max.y,acc_min.y);
	BSP_Ser_Printf("z max:%f,z min: %f\r\n",acc_max.z,acc_min.z);
	
	BSP_Ser_Printf("Gain:%f,%f,%f\r\n",acc_gain.x,acc_gain.y,acc_gain.z);
	BSP_Ser_Printf("Offset:%f,%f,%f\r\n",acc_offset.x,acc_offset.y,acc_offset.z);
	
	g_LEDBlinkPeriod_ms = 200;	
	BSP_OS_TimeDlyMs(10000);
	g_FlagAccCalFinished = SET;
	while(1)
  {
    BSP_OS_TimeDlyMs(50);
  }
}


void AccCalibrateTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&AccCalibrateTaskTCB,
               (CPU_CHAR   *)"AccCalibrate Task",
               (OS_TASK_PTR )AccCalibrateTask,
               (void       *)0,
               (OS_PRIO     )ACC_CALIBRATE_TASK_PRIO,
               (CPU_STK    *)&AccCalibrateTaskStk[0],
               (CPU_STK_SIZE)ACC_CALIBRATE_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)ACC_CALIBRATE_TASK_STK_SIZE,
               (OS_MSG_QTY  )5u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate AccCalibrate Task...\n\r"));
  }
}
