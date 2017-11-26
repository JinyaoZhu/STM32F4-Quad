#ifndef __GLOBAL_VARIABLE_H__
#define __GLOBAL_VARIABLE_H__

#include "includes.h"


/*
**********************************
          App Object
**********************************
*/
extern BSP_OS_SEM Sem_AttControlUpdate;/* update control data */

extern BSP_OS_SEM Sem_PosControlUpdate;

extern BSP_OS_SEM Sem_SerPrintUpdate;

/*
**********************************
          RC Task
**********************************
*/
extern EULER g_RCEuler;
extern float g_RCThrottle;
extern VECTOR g_TargetPos;
extern VECTOR g_TargetVel;
extern VECTOR g_TargetAcc;

extern uint8_t g_FlagTakeOff;
extern uint8_t g_FlagPosHoldZ;
extern uint8_t g_FlagPosHoldXY;
extern uint8_t g_FlagArmed;

extern uint8_t g_FlagTriggerStick;


/*
**********************************
          Attitude Task
**********************************
*/
extern EULER g_AttEuler;
extern VECTOR g_AccRaw;
extern VECTOR g_AccRefRaw;
extern VECTOR g_GyroRaw;
extern VECTOR g_AccRef;
extern VECTOR g_VelRef;
extern VECTOR g_PosRef;
extern CPU_INT08U g_FlagAttInit;
/*
**********************************
          Sonar Task
**********************************
*/
extern float g_SonarDist;
extern CPU_INT08U g_FlagSonarUpdate;
/*
**********************************
          Baro Task
**********************************
*/
extern float g_BaroAlt;
extern float g_BaroAltOffset;

extern CPU_INT08U g_FlagBaroUpdate;
/*
**********************************
          Comm Task
**********************************
*/
extern VECTOR g_CamPos;
extern VECTOR g_CamVel;
extern EULER  g_CamEuler;
extern VECTOR g_CamTargetPos;
extern uint8_t g_FlagCamUpdate;
/*
**********************************
          Pos Control Task
**********************************
*/
extern EULER g_PosEuler;
extern float g_PosThrottle;
typedef enum {stiff=0,soft}PID_MODE;
extern PID_MODE g_PosControlMode;
/*
**********************************
            DEBUG
**********************************
*/
extern float g_TestTmpData1;
extern float g_TestTmpData2;
extern float g_TestTmpData3;
extern float g_TestTmpData4;
extern float g_TestTmpData5;
extern float g_TestTmpData6;
extern float g_TestTmpData7;
extern float g_TestTmpData8;
extern float g_TestTmpData9;
extern float g_TestTmpData10;
extern float g_TestTmpData11;
extern float g_TestTmpData12;

extern CPU_INT32U g_LEDBlinkPeriod_ms;

/*
**********************************
        Calibrate Task
**********************************
*/
extern CPU_INT08U g_Flag_ESC_Calibrated;
extern CPU_INT08U g_FlagAccCalFinished;


CPU_FP32 Get_dt(CPU_TS32 *timestamp_old);


#endif /*__GLOBAL_VARIABLE_H__*/
