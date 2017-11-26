#include "GlobalVariable.h"

/*
**********************************
          App Object
**********************************
*/
BSP_OS_SEM Sem_AttControlUpdate;/* update control data */

BSP_OS_SEM Sem_PosControlUpdate;

BSP_OS_SEM Sem_SerPrintUpdate;

/*
**********************************
          RC Task
**********************************
*/
EULER g_RCEuler = {0,0,0};
float g_RCThrottle = 0;
uint8_t g_FlagTakeOff = RESET;
VECTOR g_TargetPos = {0,0,0};
VECTOR g_TargetVel = {0,0,0};
VECTOR g_TargetAcc = {0,0,0};

uint8_t g_FlagPosHoldZ = RESET;
uint8_t g_FlagPosHoldXY = RESET;
uint8_t g_FlagArmed = SET;

uint8_t g_FlagTriggerStick = RESET;


/*
**********************************
          Attitude Task
**********************************
*/
EULER g_AttEuler = {0,0,0};
VECTOR g_AccRaw = {0,0,0};
VECTOR g_GyroRaw = {0,0,0};
VECTOR g_AccRef = {0,0,0};
VECTOR g_AccRefRaw = {0,0,0};
VECTOR g_VelRef = {0,0,0};
VECTOR g_PosRef = {0,0,0};
CPU_INT08U g_FlagAttInit = RESET;


/*
**********************************
          Sonar Task
**********************************
*/
float g_SonarDist = 0;
CPU_INT08U g_FlagSonarUpdate = RESET;
/*
**********************************
          Baro Task
**********************************
*/
float g_BaroAlt = 0;
float g_BaroAltOffset = 0;

CPU_INT08U g_FlagBaroUpdate = RESET;
/*
**********************************
          Comm Task
**********************************
*/
VECTOR g_CamPos = {0,0,0};
VECTOR g_CamVel = {0,0,0};
EULER  g_CamEuler = {0,0,0};
VECTOR g_CamTargetPos = {0,0,0};
uint8_t g_FlagCamUpdate = RESET;
/*
**********************************
          Pos Control Task
**********************************
*/
EULER g_PosEuler = {0,0,0};
float g_PosThrottle = 0;
PID_MODE g_PosControlMode = stiff;

/*
**********************************
            DEBUG
**********************************
*/
float g_TestTmpData1 = 0;
float g_TestTmpData2 = 0;
float g_TestTmpData3 = 0;
float g_TestTmpData4 = 0;
float g_TestTmpData5 = 0;
float g_TestTmpData6 = 0;
float g_TestTmpData7 = 0;
float g_TestTmpData8 = 0;
float g_TestTmpData9 = 0;
float g_TestTmpData10 = 0;
float g_TestTmpData11 = 0;
float g_TestTmpData12 = 0;

CPU_INT32U g_LEDBlinkPeriod_ms = 1000;

/*
**********************************
        Calibrate Task
**********************************
*/
CPU_INT08U g_Flag_ESC_Calibrated = SET;
CPU_INT08U g_FlagAccCalFinished = SET;




/*
******************************************************************
*                          Get_dt()
* Description : A hight resoluion counter used cpu timestamp to get 
*               the period of a loop.
*
* Parameter   : CPU_TS32 *timestamp_old --- used to save old stamp
*               each task whitch used this function should have an individual
*               variable to store the old timestamp.
*
* Caller      : Application.
******************************************************************
*/
CPU_FP32 Get_dt(CPU_TS32 *timestamp_old)
{
	CPU_FP32 dt;
  CPU_TS32 timestamp_new = 0;
	
	timestamp_new = CPU_TS_Get32();
	
	if(timestamp_new < *timestamp_old)
	{
		dt = (CPU_FP32)(timestamp_new +(4294967296u - *timestamp_old))/SYS_CLOCK_FREQ; /* sec */
	}
	else
	{
		dt = (CPU_FP32)(timestamp_new - *timestamp_old)/SYS_CLOCK_FREQ;
	}
	*timestamp_old = timestamp_new;
	return dt;
}
