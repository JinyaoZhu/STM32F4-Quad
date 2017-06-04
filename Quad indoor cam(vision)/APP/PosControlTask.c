#include "includes.h"
#include "GlobalVariable.h"
#include "sys_params.h"

__align(8) static  CPU_STK  PosControlTaskStk[POS_CONTROL_TASK_STK_SIZE];
OS_TCB  PosControlTaskTCB;

// static const VECTOR KpPos = {20.0, 20.0, 1.0};
// static const VECTOR KdPos = {10.0, 10.0, 0.45};
// static const VECTOR KiPos = {0.0, 0.0, 0.30};

//static const VECTOR KpPos = {20.0, 20.0, 25.00};
//static const VECTOR KdPos = {10.0, 10.0, 20.00};

VECTOR KpPos = {8.0, 8.0, 12.00};
VECTOR KdPos = {6.0, 6.0, 14.00};
VECTOR KiPos = {12.0,12.0, 12.00};

void PosControlTask(void *p_arg)
{
  CPU_FP32 dt = 0;
  CPU_TS32 timestamp_old;
  EULER euler_att;
	
	/* measurement */
	VECTOR pos_ref = {0};
  VECTOR vel_ref = {0};

  /* target */
  VECTOR target_pos = {0};
  VECTOR target_vel = {0};
  VECTOR target_acc = {0};

  /* position PID */
  float sin_yaw, cos_yaw;
  EULER euler_pos;
  float pos_throttle;

  /* position error */
  VECTOR error_pos = {0, 0, 0};
  VECTOR i_error_pos = {0, 0, 0};
  /* velocity error */
  VECTOR error_vel = {0, 0, 0};

  VECTOR des_acc_r = {0,0,0};
  VECTOR des_acc_b1 = {0,0,0};

  (void)p_arg;

  timestamp_old = CPU_TS_Get32();

  while (1)
  {
		BSP_OS_SemWait(&Sem_PosControlUpdate, 0);
		
		//LOGIC_DBG_PIN1_H;
    dt = Get_dt(&timestamp_old);
		
		/* update measurement */
		euler_att.yaw = g_AttEuler.yaw;
		euler_att.pitch = g_AttEuler.pitch;
		euler_att.roll = g_AttEuler.roll;
		
		VectCopy(&g_PosRef,&pos_ref);
		VectCopy(&g_VelRef,&vel_ref);
		VectCopy(&g_TargetPos,&target_pos);
		VectCopy(&g_TargetVel,&target_vel);
		VectCopy(&g_TargetAcc,&target_acc);
		
		sin_yaw = arm_sin_f32(euler_att.yaw);
    cos_yaw = arm_cos_f32(euler_att.yaw);
		
		/* update Position PID Mode */
		g_PosControlMode = stiff;
		/* stiff control */
		if(g_PosControlMode == stiff){
     KpPos.x = KpPos.y = 15.0;
		 KpPos.z = 18.00;
			
     KdPos.x = KdPos.y = 10.0;
		 KdPos.z = 15.0;
    }/* soft contorl */
		else{
     KpPos.x = KpPos.y = 8;
		 KpPos.z = 16;
			
     KdPos.x = KdPos.y = 6;
		 KdPos.z = 14;
    }
		
    /* Start Position PID */
		/* Z axis */
    if (g_FlagPosHoldZ == SET) {
      error_pos.z = DLPF(Constraint_f(target_pos.z - pos_ref.z, -0.3, 0.3), error_pos.z, 2.0f * HZ2RAD, dt);
      error_vel.z =  target_vel.z - 1.2f*vel_ref.z;
      i_error_pos.z = Constraint_f(i_error_pos.z + error_pos.z * dt, -2, 2);
			
			/* desire acc in referrent frame */
			des_acc_r.z = target_acc.z + KpPos.z*error_pos.z + KdPos.z * error_vel.z + KiPos.z * i_error_pos.z;
			
      pos_throttle = g_sys_params.mass*(des_acc_r.z + g_sys_params.gravity)/Constraint_f(arm_cos_f32(euler_att.pitch) * arm_cos_f32(euler_att.roll),0.5,1);
    }
    else {
      i_error_pos.z = 0;
      pos_throttle = 0;
    }
		
    /* XY plane */
    if (g_FlagPosHoldXY == SET) {
      /* X */
      error_pos.x   = DLPF(Constraint_f(target_pos.x - pos_ref.x, -0.3, 0.3), error_pos.x , 5.0f * HZ2RAD, dt);
      error_vel.x = target_vel.x - 1.2f*vel_ref.x;
      i_error_pos.x = Constraint_f(i_error_pos.x + error_pos.x * dt, -2, 2);

      /* Y */
      error_pos.y   = DLPF(Constraint_f(target_pos.y - pos_ref.y, -0.3, 0.3), error_pos.y , 5.0f * HZ2RAD, dt);
      error_vel.y =  target_vel.y - 1.2f*vel_ref.y;
      i_error_pos.y = Constraint_f(i_error_pos.y + error_pos.y * dt, -2, 2);
			
      
			/* desire acc in referrent frame */
      des_acc_r.x = target_acc.x + KpPos.x*error_pos.x + KdPos.x * error_vel.x + KiPos.x * i_error_pos.x;
      des_acc_r.y = target_acc.y + KpPos.y*error_pos.y + KdPos.y * error_vel.y + KiPos.y * i_error_pos.y;
      
			/* desire acc in b1 frame */
      des_acc_b1.x =  cos_yaw * des_acc_r.x + sin_yaw * des_acc_r.y;
      des_acc_b1.y = -sin_yaw * des_acc_r.x + cos_yaw * des_acc_r.y;

      euler_pos.pitch = Constraint_f(asinf( Constraint_f(des_acc_b1.x / (arm_cos_f32(euler_att.roll) * g_sys_params.gravity), -1, 1)), -50 * DEG2RAD, 50 * DEG2RAD);
      euler_pos.roll  = Constraint_f(asinf(-Constraint_f(des_acc_b1.y / g_sys_params.gravity, -1, 1)), -50 * DEG2RAD, 50 * DEG2RAD);
			//euler_pos.pitch =  Constraint_f(des_acc_b1.x/g_sys_params.gravity, -30.0f* DEG2RAD, 30.0f* DEG2RAD) ;
      //euler_pos.roll  = -Constraint_f(des_acc_b1.y/g_sys_params.gravity, -30.0f* DEG2RAD, 30.0f* DEG2RAD);
    }
    else {
      euler_pos.pitch = euler_pos.roll = 0;
      i_error_pos.x = i_error_pos.y = 0;
    }
		/* End position PID */
		
		g_TestTmpData1 = euler_att.yaw;
 		g_TestTmpData2 = euler_att.pitch;
 		g_TestTmpData3 = euler_att.roll;
		g_TestTmpData4 = pos_ref.x;
 		g_TestTmpData5 = pos_ref.y;
 		g_TestTmpData6 = pos_ref.z;
		
    /* update */
    g_PosEuler.yaw = 0;
    g_PosEuler.pitch = euler_pos.pitch;
    g_PosEuler.roll = euler_pos.roll;
    g_PosThrottle = pos_throttle;
    //LOGIC_DBG_PIN1_L;
		
    BSP_OS_SemPost(&Sem_AttControlUpdate);/* trigger attitude control */
		//BSP_OS_TimeDlyMs(5);
  }
}






void PosControlTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&PosControlTaskTCB,
               (CPU_CHAR   *)"PosControlTask",
               (OS_TASK_PTR )PosControlTask,
               (void       *)0,
               (OS_PRIO     )POS_CONTROL_TASK_PRIO,
               (CPU_STK    *)&PosControlTaskStk[0],
               (CPU_STK_SIZE)POS_CONTROL_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)POS_CONTROL_TASK_STK_SIZE,
               (OS_MSG_QTY  )5u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);

  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate Position Control Task...\n\r"));
  }
  else
    for (;;);
}
