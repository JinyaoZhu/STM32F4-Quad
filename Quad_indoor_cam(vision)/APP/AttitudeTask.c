#include "includes.h"
#include "GlobalVariable.h"
#include "sensor_update.h"


#define ATT_INIT_TIMES  (7.00f) /*s*/

OS_TCB  AttitudeTaskTCB;
__align(8) static  CPU_STK  AttitudeTaskStk[ATTITUDE_TASK_STK_SIZE];

#define KALMAN_POS_K1_X   (0.150f)
#define KALMAN_POS_K2_X   (0.620f)
#define KALMAN_POS_K3_X   (-0.01f)

#define KALMAN_POS_K1_Y   (0.150f)
#define KALMAN_POS_K2_Y   (0.620f)
#define KALMAN_POS_K3_Y   (-0.01f)

#define KALMAN_POS_K1_Z   (0.150f)
#define KALMAN_POS_K2_Z   (0.620f)
#define KALMAN_POS_K3_Z   (-0.01f)

static float acc_ref_x_hist[40] = {0};
static float acc_ref_y_hist[40] = {0};
static float acc_ref_z_hist[40] = {0};
#define MEAS_DELAY  (20)



/*******************************************************************************************************/
void AttitudeTask(void *p_arg)
{
  float dt;
  CPU_TS32 timestamp_old = 0; /* used to calculate task loop interval */

  /* raw data */
  VECTOR gyro_raw;
  VECTOR acc_raw;
  VECTOR last_gyro_raw = {0, 0, 0};

  /* attitude estimate */
  EULER att_euler;
  EULER euler_delta;
  EULER euler_tmp;
  VECTOR acc_g_body;
  VECTOR acc_g_body_meas;
  VECTOR acc_g_body_meas_cam;
  VECTOR mag_g_body = {1, 0, 0};
  const VECTOR mag_g_const = {1, 0, 0};
  const VECTOR acc_g_const = {0, 0, -9.8};
  VECTOR mag_g_body_meas;
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;
  float Xh, Yh;

  /* position estimate */
	int32_t acc_ref_hist_index = 0;
	int32_t i,j;
	float tmp1,tmp2,tmp3,u;
  VECTOR acc_ref = {0, 0, 0};
  VECTOR acc_ref_filtered = {0, 0, 0};
  VECTOR vel_ref = {0, 0, 0};
  VECTOR pos_ref = {0, 0, 0};
  VECTOR acc_ref_offset = {0, 0, 0};
  float tmp;
  uint32_t att_loop_cnt = 0;

  uint32_t control_update_cnt = 0;

  (void)p_arg;

  g_LEDBlinkPeriod_ms = 50;

  /* Initialize all snesor */
  SensorUpdate_Init();

  g_LEDBlinkPeriod_ms = 200;

  SensorUpdate_GetGyroAccRaw(&gyro_raw, &acc_raw);
  acc_g_body.x = acc_raw.x;
  acc_g_body.y = acc_raw.y;
  acc_g_body.z = acc_raw.z;
	
  timestamp_old = CPU_TS_Get32();

  while (1)
  {
    SensorUpdate_GetGyroAccRaw(&gyro_raw, &acc_raw);
    //LOGIC_DBG_PIN1_H;
		
    dt = Get_dt(&timestamp_old);

    VectCopy(&gyro_raw, &g_GyroRaw);
    VectCopy(&acc_raw, &g_AccRaw);

    /* Start attitude estimate */
    euler_delta.yaw = -0.5f * (g_GyroRaw.z + last_gyro_raw.z) * dt;
    euler_delta.pitch = -0.5f * (g_GyroRaw.y + last_gyro_raw.y) * dt;
    euler_delta.roll = -0.5f * (g_GyroRaw.x + last_gyro_raw.x) * dt;

    EulerRoateVect(&acc_g_body, &acc_g_body, &euler_delta); /* prior estimate */
    EulerRoateVect(&mag_g_body, &mag_g_body, &euler_delta);

    VectCopy(&acc_raw, &acc_g_body_meas);
    acc_g_body.x += (acc_g_body_meas.x - acc_g_body.x) * 0.0003f;
    acc_g_body.y += (acc_g_body_meas.y - acc_g_body.y) * 0.0003f;
    acc_g_body.z += (acc_g_body_meas.z - acc_g_body.z) * 0.0003f;

    /* get pitch and roll */
    att_euler.pitch = atan2f(acc_g_body.x, sqrtf(acc_g_body.y * acc_g_body.y + acc_g_body.z * acc_g_body.z));
    att_euler.roll  = atan2f(-acc_g_body.y, -acc_g_body.z);

    sinRoll = sinf(g_AttEuler.roll);
    cosRoll = cosf(g_AttEuler.roll);
    sinPitch = sinf(g_AttEuler.pitch);
    cosPitch = cosf(g_AttEuler.pitch);
    Xh = mag_g_body.x * cosPitch + mag_g_body.y * sinRoll * sinPitch + mag_g_body.z * sinPitch * cosRoll;
    Yh = mag_g_body.y * cosRoll  - mag_g_body.z * sinRoll;
    //VectNormalize(&mag_g_body);
    /* get yaw */
    att_euler.yaw = -atan2f(Yh, Xh);

    /* update attitude */
    g_AttEuler.yaw = att_euler.yaw;
    g_AttEuler.pitch = att_euler.pitch;
    g_AttEuler.roll = att_euler.roll;

    VectCopy(&gyro_raw, &last_gyro_raw);
    /* End attitude estimate */

    /* Start position estimate */

    EulerRoateVect(&acc_ref, &acc_raw, &g_AttEuler);/* Get reference acceleration */
    VectScale(&acc_ref, -1);

    if (att_loop_cnt < 1000) {
      if (att_loop_cnt >= 500)
        VectAdd(&acc_ref_offset, &acc_ref_offset, &acc_ref);
      VectScale(&acc_ref, 0);
      att_loop_cnt++;
    }
    else if (att_loop_cnt == 1000) {
      VectScale(&acc_ref_offset, 1.0f / 500);
      VectScale(&acc_ref, 0);
      att_loop_cnt++;
      g_FlagAttInit = SET; /* Indicate attitude init is ok */
    }
    else {
      //VectSub(&acc_ref, &acc_ref, &acc_ref_offset);
      VectCopy(&acc_ref, &g_AccRefRaw);
      acc_ref_filtered.x = DLPF(acc_ref.x, acc_ref_filtered.x, 40 * HZ2RAD, dt);
      acc_ref_filtered.y = DLPF(acc_ref.y, acc_ref_filtered.y, 40 * HZ2RAD, dt);
      acc_ref_filtered.z = DLPF(acc_ref.z, acc_ref_filtered.z, 40 * HZ2RAD, dt);
    }

    if (g_FlagAttInit == SET) {

      /* a prior estimate */
      vel_ref.x = Constraint_f(vel_ref.x + (acc_ref_filtered.x - acc_ref_offset.x) * dt, -10, 10);
      vel_ref.y = Constraint_f(vel_ref.y + (acc_ref_filtered.y - acc_ref_offset.y) * dt, -10, 10);
      vel_ref.z = Constraint_f(vel_ref.z + (acc_ref_filtered.z - acc_ref_offset.z)* dt, -10, 10);

      pos_ref.x = Constraint_f(pos_ref.x + vel_ref.x * dt + (acc_ref_filtered.x - acc_ref_offset.x) * 0.5f * dt * dt, -10, 10);
      pos_ref.y = Constraint_f(pos_ref.y + vel_ref.y * dt + (acc_ref_filtered.y - acc_ref_offset.y) * 0.5f * dt * dt, -10, 10);
      pos_ref.z = Constraint_f(pos_ref.z + vel_ref.z * dt + (acc_ref_filtered.z - acc_ref_offset.z) * 0.5f * dt * dt, -10, 10);
			
			acc_ref_hist_index = (acc_ref_hist_index+1)%MEAS_DELAY;
			acc_ref_x_hist[acc_ref_hist_index] = acc_ref_filtered.x;
			acc_ref_y_hist[acc_ref_hist_index] = acc_ref_filtered.y;
			acc_ref_z_hist[acc_ref_hist_index] = acc_ref_filtered.z;
			
      /* Camera update */
      if (g_FlagCamUpdate == SET) {
				/* trace back to 40ms ago */
				j = acc_ref_hist_index;
				for(i=0;i<MEAS_DELAY;i++){
					/*X*/
					u = acc_ref_x_hist[j];
					tmp1 = pos_ref.x - 0.5f*dt*dt*u;
					tmp2 = vel_ref.x - dt*u;
					tmp3 = acc_ref_offset.x;
          pos_ref.x = tmp1 - dt*tmp2 - 0.5f*dt*dt*tmp3;
					vel_ref.x = tmp2 + dt*tmp3;

					/*Y*/
					u = acc_ref_y_hist[j];
					tmp1 = pos_ref.y - 0.5f*dt*dt*u;
					tmp2 = vel_ref.y - dt*u;
					tmp3 = acc_ref_offset.y;
          pos_ref.y = tmp1 - dt*tmp2 - 0.5f*dt*dt*tmp3;
					vel_ref.y = tmp2 + dt*tmp3;

					/*Z*/
					u = acc_ref_z_hist[j];
					tmp1 = pos_ref.z - 0.5f*dt*dt*u;
					tmp2 = vel_ref.z - dt*u;
					tmp3 = acc_ref_offset.z;
          pos_ref.z = tmp1 - dt*tmp2 - 0.5f*dt*dt*tmp3;
					vel_ref.z = tmp2 + dt*tmp3;
					
					j--;
					if(j < 0)
						j = MEAS_DELAY-1;
        }
				
        tmp = g_CamPos.x - pos_ref.x; /* X update */
        pos_ref.x += KALMAN_POS_K1_X * tmp;
        vel_ref.x += KALMAN_POS_K2_X * tmp;
        acc_ref_offset.x += KALMAN_POS_K3_X * tmp;
				
				tmp = g_CamPos.y - pos_ref.y;  /* Y update */
        pos_ref.y += KALMAN_POS_K1_Y * tmp;
        vel_ref.y += KALMAN_POS_K2_Y * tmp;
        acc_ref_offset.y += KALMAN_POS_K3_Y * tmp;

        tmp = g_CamPos.z - pos_ref.z; /* Z update */
        pos_ref.z += KALMAN_POS_K1_Z * tmp;
        vel_ref.z += KALMAN_POS_K2_Z * tmp;
        acc_ref_offset.z += KALMAN_POS_K3_Z * tmp;
				
				/* recover */
				for(i=0;i<MEAS_DELAY;i++){
					/*X*/
					u = acc_ref_x_hist[j] - acc_ref_offset.x;
          pos_ref.x = pos_ref.x + dt*vel_ref.x + 0.5f*dt*dt*u;
					vel_ref.x = vel_ref.x + dt*u;
					/*Y*/
					u = acc_ref_y_hist[j] - acc_ref_offset.y;
          pos_ref.y = pos_ref.y + dt*vel_ref.y + 0.5f*dt*dt*u;
					vel_ref.y = vel_ref.y + dt*u;
					/*Z*/
					u = acc_ref_z_hist[j] - acc_ref_offset.z;
          pos_ref.z = pos_ref.z + dt*vel_ref.z + 0.5f*dt*dt*u;
					vel_ref.z = vel_ref.z + dt*u;
					j = (j+1)%MEAS_DELAY;
       }
		
        /* cam euler */
        euler_tmp.yaw = -g_CamEuler.yaw;
        euler_tmp.pitch = -g_CamEuler.pitch;
        euler_tmp.roll = -g_CamEuler.roll;
				
        EulerRoateVect(&mag_g_body_meas, (VECTOR*)&mag_g_const, &euler_tmp);
        EulerRoateVect(&acc_g_body_meas_cam, (VECTOR*)&acc_g_const, &euler_tmp);

        mag_g_body.x += (mag_g_body_meas.x - mag_g_body.x) * 0.01f;
        mag_g_body.y += (mag_g_body_meas.y - mag_g_body.y) * 0.01f;
        mag_g_body.z += (mag_g_body_meas.z - mag_g_body.z) * 0.01f;

        acc_g_body.x += (acc_g_body_meas_cam.x - acc_g_body.x) * 0.0f;
        acc_g_body.y += (acc_g_body_meas_cam.y - acc_g_body.y) * 0.0f;
        acc_g_body.z += (acc_g_body_meas_cam.z - acc_g_body.z) * 0.0f;

        g_FlagCamUpdate = RESET;
      }
    }
    VectCopy(&acc_ref, &g_AccRef);
    VectCopy(&vel_ref, &g_VelRef);
    VectCopy(&pos_ref, &g_PosRef);
    /* End position estimate */
		
// 		g_TestTmpData1 = vel_ref.x;
// 		g_TestTmpData2 = vel_ref.y;
// 		g_TestTmpData3 = vel_ref.z;

    /* post semaphore to control task */
    if (control_update_cnt >= 1) {
      BSP_OS_SemPost(&Sem_PosControlUpdate);/* trigger position control */
      control_update_cnt = 0;
    }
    else
      control_update_cnt++;
	  //LOGIC_DBG_PIN1_L;
  }
}




/*******************************************************************************************************/
void AttitudeTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&AttitudeTaskTCB,
               (CPU_CHAR   *)"Attitude Task",
               (OS_TASK_PTR )AttitudeTask,
               (void       *)0,
               (OS_PRIO     )ATTITUDE_TASK_PRIO,
               (CPU_STK    *)&AttitudeTaskStk[0],
               (CPU_STK_SIZE)ATTITUDE_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)ATTITUDE_TASK_STK_SIZE,
               (OS_MSG_QTY  )0u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate Attitude Task...\n\r"));
  }
}

