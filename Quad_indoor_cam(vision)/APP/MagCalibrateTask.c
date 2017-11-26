#include "includes.h"
#include "GlobalVariable.h"
#include "sensor_update.h"

OS_TCB  MagCalibrateTaskTCB;
__align(8) static  CPU_STK  MagCalibrateTaskStk[MAG_CALIBRATE_TASK_STK_SIZE];



/*******************************************************************************************************/
void MagCalibrateTask(void *p_arg)
{
  CPU_TS32 timestamp_old = 0;
	CPU_FP32 mag_x_max,mag_x_min;
	CPU_FP32 mag_y_max,mag_y_min;
	CPU_FP32 mag_z_max,mag_z_min;
	VECTOR mag;
	VECTOR mag_offset;
	CPU_FP32 rotate_angle_x;
	CPU_FP32 rotate_angle_y;
	CPU_FP32 rotate_angle_z;
	CPU_FP32 dt = 0;
	CPU_INT08U CalSuccessFlag;
	(void)p_arg;
	
	mag_x_max = mag_x_min = 0;
	mag_y_max = mag_y_min = 0;
	mag_z_max = mag_z_min = 0;

	while(g_AttitudeTask_InitFlag == 0)/* wait until Attitude Init success */
	  BSP_OS_TimeDlyMs(10);
	
	BSP_Ser_Printf("Calibrate Magnetic sensor...\r\n");
	
	SensorUpdate_SetMagOffset(0,0,0);
	
	CalSuccessFlag = 1;
	g_LEDBlinkPeriod_ms = 25;
	rotate_angle_x = rotate_angle_y = rotate_angle_z = 0.0f;
	
	timestamp_old = CPU_TS_Get32();
	do
	{
		dt = Get_dt(&timestamp_old);
		
		mag.x = g_ATT_MagRaw.x;
		mag.y = g_ATT_MagRaw.y;
		
		rotate_angle_x += g_ATT_GyroRaw.x * dt;
		rotate_angle_y += g_ATT_GyroRaw.y * dt;
		rotate_angle_z += g_ATT_GyroRaw.z * dt;

    if((fabs(rotate_angle_x) > 30*DEG2RAD)||(fabs(rotate_angle_y) > 30*DEG2RAD))
			CalSuccessFlag = 0;
		
		/* mag x axis */
		if(mag.x > mag_x_max)
      mag_x_max = mag.x;
		else if(mag.x < mag_x_min)
      mag_x_min = mag.x;
		
		/* mag y axis */
	  if(mag.y > mag_y_max)
      mag_y_max = mag.y;
		else if(mag.y < mag_y_min)
      mag_y_min = mag.y;
		
		BSP_OS_TimeDlyMs(20);
  }while(fabs(rotate_angle_z) < 2*PI);
	
	g_LEDBlinkPeriod_ms = 500;
	BSP_OS_TimeDlyMs(5000);
	
	g_LEDBlinkPeriod_ms = 50;
	rotate_angle_x = rotate_angle_y = rotate_angle_z = 0.0f;
	timestamp_old = CPU_TS_Get32();
	do/* x-axis point up */
	{
		dt = Get_dt(&timestamp_old);
		mag.z = g_ATT_MagRaw.z;
		rotate_angle_x += g_ATT_GyroRaw.x * dt;
		rotate_angle_y += g_ATT_GyroRaw.y * dt;
		rotate_angle_z += g_ATT_GyroRaw.z * dt;

    if((fabs(rotate_angle_z) > 30*DEG2RAD)||(fabs(rotate_angle_y) > 30*DEG2RAD))
			CalSuccessFlag = 0;		

		/* mag z axis */
		if(mag.z > mag_z_max)
		{
      mag_z_max = mag.z;
    }
		else if(mag.z < mag_z_min)
		{
      mag_z_min = mag.z;
    }
		BSP_OS_TimeDlyMs(20);
  }while(fabs(rotate_angle_x) < 2*PI);

	/* calibrate success */
	if(CalSuccessFlag == 1){
		
	g_LEDBlinkPeriod_ms = 100u;
	
	mag_offset.x = (mag_x_max + mag_x_min) * 0.5f;
	mag_offset.y = (mag_y_max + mag_y_min) * 0.5f;
	mag_offset.z = (mag_z_max + mag_z_min) * 0.5f;	
		
	SensorUpdate_SetMagOffset(mag_offset.x,mag_offset.y,mag_offset.z);
	
	BSP_Ser_Printf("Mag compensate complete!\r\n");
	BSP_Ser_Printf("\n\rMAG_OFFSET_x:%4.1f,MAG_OFFSET_y:%4.1f,MAG_OFFSET_z:%4.1f \r\n",mag_offset.x,mag_offset.y,mag_offset.z);
  g_MagCalFinishedFlag = 1;
		
  }
	/* calibrate fail */
	else{
		g_LEDBlinkPeriod_ms = 25u;
    BSP_Ser_Printf("Mag compensate FAIL!\r\n");
  }
		
	BSP_OS_TimeDlyMs(5000);
	g_LEDBlinkPeriod_ms = 500u;
	
	while(1)
  {
    BSP_OS_TimeDlyMs(50);
  }
}


void MagCalibrateTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&MagCalibrateTaskTCB,
               (CPU_CHAR   *)"MagCalibrate Task",
               (OS_TASK_PTR )MagCalibrateTask,
               (void       *)0,
               (OS_PRIO     )MAG_CALIBRATE_TASK_PRIO,
               (CPU_STK    *)&MagCalibrateTaskStk[0],
               (CPU_STK_SIZE)MAG_CALIBRATE_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)MAG_CALIBRATE_TASK_STK_SIZE,
               (OS_MSG_QTY  )5u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate MagCalibrate Task...\n\r"));
  }
}
