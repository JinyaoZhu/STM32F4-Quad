#include "includes.h"
#include "GlobalVariable.h"


__align(8) static  CPU_STK  PrintfTaskStk[PRINTF_TASK_STK_SIZE];
OS_TCB  PrintfTaskTCB;


/*******************************************************************************************************/
void PrintfTask(void *p_arg)
{
	(void)p_arg;
	
	while(g_FlagAttInit == RESET) BSP_OS_TimeDlyMs(10);
  while(g_FlagAccCalFinished == RESET) BSP_OS_TimeDlyMs(0);
	
  while (1)
  {
		BSP_OS_SemWait(&Sem_SerPrintUpdate,0);
		
		//BSP_I2C2_SingleWriteNoMemAddr(0x00,0x42);
		//BSP_Ser_Printf("%d,\r\n",(int32_t)BSP_I2C2_SingleRead(0x00,0x84));
		//BSP_Ser_Printf("%f\r\n",g_InertialData_dt);
		//BSP_Ser_Printf("%5.3f,%5.3f,%3d,\r\n",g_ATT_VelRef.x,g_OptFlow_Vx,g_OptFlowQuality);
		//BSP_Ser_Printf("%8.4f,%8.4f,%8.4f,%3d,P\r\n",g_ATT_PosRef.x,g_ATT_PosRef.y,g_ATT_PosRef.z,g_OptFlowQuality);
		//BSP_Ser_Printf("%f,%f,%f,\r\n",g_VelRef.x,g_VelRef.y,g_VelRef.z);
		//BSP_Ser_Printf("%f,%f,%f,\r\n",g_ATT_AccRef.x,g_ATT_AccRef.y,g_ATT_AccRef.z);
		//BSP_Ser_Printf("%f,\r\n",VectNorm(&g_ATT_AccRef));
		//BSP_Ser_Printf("%f,%f,\r\n",g_OptFlow_Vx,g_ATT_VelRef.x);
		
		//BSP_Ser_Printf("%f,%f,%3d,\r\n",g_OptFlow_Vx,g_OptFlow_Vy,g_OptFlowQuality);
		
		//BSP_Ser_Printf("%d\r\n",g_OptFlowQuality);
		//BSP_Ser_Printf("%f,%f,\r\n",g_OptFlow_Px,g_OptFlow_Py);
		//BSP_Ser_Printf("%8.4f,%8.4f,%3d\r\n",g_OptFlow_Vx,g_OptFlow_Vy,g_OptFlowQuality);
		//BSP_Ser_Printf("%8.4f,%8.4f,%8.4f,%8.4f,P\r\n",g_SonarDist,g_PosRef.z,g_VelRef.z,g_AccRef.z);
		//BSP_Ser_Printf("%f,\r\n",g_Sonar_AltEst);
		//BSP_Ser_Printf("%f,\r\n",g_ATT_AccRef.z);
		//BSP_Ser_Printf("%f,%f,\r\n",g_ATT_PosRef.z,g_Baro_AltEst);
		//BSP_Ser_Printf("%6.3f,%6.3f,%6.3f,\r\n", g_VelRef.x,g_VelRef.y,g_VelRef.z);
		//BSP_Ser_Printf("%6.3f,%6.3f,%6.3f, MR\r\n", g_ATT_MagRef.x,g_ATT_MagRef.y,g_ATT_MagRef.z);
		//BSP_Ser_Printf("%f\r\n",g_AccCurrentNorm);
    
    //BSP_Ser_Printf("%8.4f,%8.4f,%8.4f,%6.2f,%6.2f, %s\r\n", RAD2DEG*g_ATT_E.yaw, RAD2DEG*g_ATT_E.pitch, RAD2DEG*g_ATT_E.roll,g_ATT_PosRef.z,g_AttTaskRunningTime,g_ArmFlag?"Armed":"Disarmed");
	  //BSP_Ser_Printf("%8.4f,%8.4f,%8.4f, E_ATT\r\n", RAD2DEG*g_AttEuler.yaw, RAD2DEG*g_AttEuler.pitch, RAD2DEG*g_AttEuler.roll);
		//BSP_Ser_Printf("%8.4f,%8.4f,%8.4f,%3.2f E_TRG\r\n", RAD2DEG*g_ATT_TargetE.yaw, RAD2DEG*g_ATT_TargetE.pitch, RAD2DEG*g_ATT_TargetE.roll,g_Thrust_RC);
    //BSP_Ser_Printf("%8.4f,%8.4f,%8.4f,G \r\n", g_AccRaw.x,g_AccRaw.y,g_AccRaw.z);
		//BSP_Ser_Printf("%8.4f,%8.4f,%8.4f,G \r\n", g_ATT_Gyro.x,g_ATT_Gyro.y,g_ATT_Gyro.z);
		
		//BSP_Ser_Printf("%8.4f,%8.4f, E_ATT_CMP\r\n", RAD2DEG*g_ATT_E.pitch, RAD2DEG*g_ATT_TargetE.pitch);
		//BSP_Ser_Printf("%8.4f,%8.4f, E_ATT_CMP\r\n", RAD2DEG*g_ATT_E.roll, RAD2DEG*g_ATT_TargetE.roll);
		//BSP_Ser_Printf("%8.4f,%8.4f, E_ATT_CMP\r\n", RAD2DEG*g_ATT_E.yaw, RAD2DEG*g_ATT_TargetE.yaw);
		//BSP_Ser_Printf("%8.4f,%8.4f,%8.4f,\r\n", g_TargetPos.z,g_PosRef.z,g_AttEuler.yaw);
		//BSP_Ser_Printf("%8.4f,%8.4f,%8.4f, CAM\r\n", g_CamPos.x,g_CamPos.y,g_CamPos.z);
		//BSP_Ser_Printf("%8.4f,%8.4f,%8.4f,%8.4f, E_ATT_CMP\r\n", g_Thrust_RC,g_Altitude,g_ATT_Mag.x,g_ATT_Mag.y,g_ATT_Mag.z);
		
		//BSP_Ser_Printf("%8.5f,%8.5f,%8.5f,A \r\n", g_GyroRaw.x,g_GyroRaw.y,g_GyroRaw.z);
		//BSP_Ser_Printf("%8.3f,%8.3f,%8.3f,A \r\n", g_AccRaw.x,g_AccRaw.y,g_AccRaw.z);
		//BSP_Ser_Printf("%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,E_ATT_CMP \r\n", EST_G_body.x,EST_G_body.y,EST_G_body.z,g_ATT_AccRaw.x,g_ATT_AccRaw.y,g_ATT_AccRaw.z);
		//BSP_Ser_Printf("%8.5f,%8.5f,%8.5f,M \r\n", g_ATT_Mag.x,g_ATT_Mag.y,g_ATT_Mag.z);
		//BSP_Ser_Printf("%f,%f,%f,%f,Q_ATT\r\n",g_ATT_Q.w,g_ATT_Q.x,g_ATT_Q.y,g_ATT_Q.z);
		//BSP_Ser_Printf("%f,%f,%f,%f,Q_TRG\r\n",g_ATT_TargetQ.w,g_ATT_TargetQ.x,g_ATT_TargetQ.y,g_ATT_Q.z);
		//BSP_Ser_Printf("%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,\r\n",g_TestTmpData1,g_TestTmpData2,g_TestTmpData3,g_TestTmpData4,g_TestTmpData5,g_TestTmpData6, \
		g_TestTmpData7,g_TestTmpData8,g_TestTmpData9);
		//BSP_Ser_Printf("%8.5f,%8.5f,%8.5f,\r\n",g_TestTmpData1,g_TestTmpData2,g_TestTmpData3);
		BSP_Ser_Printf("%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,%8.3f,\r\n",g_TestTmpData1,g_TestTmpData2,g_TestTmpData3,g_TestTmpData4,g_TestTmpData5,g_TestTmpData6);
		//BSP_Ser_Printf("%f,\r\n",g_AttTaskRunningTime);
		//BSP_Ser_Printf("%f,%f,%f,%f,Q\r\n",g_ControlData.Thrust,g_ControlData.Mx,g_ControlData.My,g_ControlData.Mz);
		
		//BSP_Ser_Printf("%f,%f,%f,%f,\r\n",g_RCThrottle,g_RCEuler.yaw,g_RCEuler.pitch,g_RCEuler.roll);
		//BSP_Ser_Printf("%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,RC\r\n",g_RC_Data.ch1,g_RC_Data.ch2,g_RC_Data.ch3,g_RC_Data.ch4,g_RC_Data.ch5,g_RC_Data.ch6);
		//BSP_Ser_Printf("%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,%5.4f,RC\r\n",g_RC_Data.yaw,g_RC_Data.pitch,g_RC_Data.roll,g_RC_Data.thrust,g_RC_Data.aux1,g_RC_Data.aux2);
    //BSP_OS_TimeDlyMs(16);
// 		{
// 			VECTOR a;
// 			float yaw_dot;
//       EulerRoateVectXY(&a,&g_AccRaw,&g_AttEuler);
// 			yaw_dot = g_GyroRaw.y*sinf(g_AttEuler.roll)/cosf(g_AttEuler.pitch) + g_GyroRaw.z*cos(g_AttEuler.roll)/cos(g_AttEuler.pitch);
// 		  BSP_Ser_Printf("%f,%f,%f,%f,%f,$$\r\n",-a.x,-a.y,yaw_dot,g_CamPos.x,g_CamPos.y);
// 		}
  }
}


/*******************************************************************************************************/
void PrintfTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&PrintfTaskTCB,
               (CPU_CHAR   *)"Printf Task",
               (OS_TASK_PTR )PrintfTask,
               (void       *)0,
               (OS_PRIO     )PRINTF_TASK_PRIO,
               (CPU_STK    *)&PrintfTaskStk[0],
               (CPU_STK_SIZE)PRINTF_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)PRINTF_TASK_STK_SIZE,
               (OS_MSG_QTY  )0u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate Printf Task...\n\r"));
  }
}
