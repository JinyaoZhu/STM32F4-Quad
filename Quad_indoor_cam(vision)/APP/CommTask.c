#include "includes.h"
#include "GlobalVariable.h"

__align(8) static  CPU_STK  CommTaskStk[COMM_TASK_STK_SIZE];
OS_TCB  CommTaskTCB;

extern BSP_OS_SEM Sem_USART2_RxWait;
extern volatile uint8_t g_packet_buf[];


/*******************************************************************************************************/
void CommTask(void *p_arg)
{
  CPU_FP32 dt = 0;
  CPU_TS32 timestamp_old = 0;
	/* Get packet */
  int32_t packet_error_cnt = 0;
	int32_t packet_total_cnt = 0;
	float packet_error_rate;
  sw2b_t packet_x, packet_y,packet_z,packet_yaw,packet_pitch,packet_roll;
	sw2b_t packet_target_x,packet_target_y,packet_target_z;
  uint8_t buf[COMM_PACKET_BUF_LEN];
	
	/* calibrate */
	VECTOR marker_in_body = {-0.002,-0.00,0.055};
  VECTOR marker_in_b1;
  VECTOR marker_in_ref;
  EULER euler_att;

  VECTOR cam_pos = {0,0,0};
  VECTOR cam_vel = {0,0,0};
  VECTOR last_cam_pos = {0,0,0};
	
  (void)p_arg;

  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  while (1)
  {
    dt = Get_dt(&timestamp_old);
		
		euler_att.yaw = g_AttEuler.yaw;
		euler_att.pitch = g_AttEuler.pitch;
		euler_att.roll = g_AttEuler.roll;
		
    if (BSP_OS_SemWait(&Sem_USART2_RxWait, 50) == DEF_OK) {
      
      /* get packet Camera position update */
      memcpy((void*)buf, (void*)g_packet_buf, COMM_PACKET_BUF_LEN);

      packet_x.b[0] = buf[1];
      packet_x.b[1] = buf[2];

      packet_y.b[0] = buf[3];
      packet_y.b[1] = buf[4];
			
			packet_z.b[0] = buf[5];
      packet_z.b[1] = buf[6];
			
			packet_yaw.b[0] = buf[7];
      packet_yaw.b[1] = buf[8];
			
			packet_pitch.b[0] = buf[9];
      packet_pitch.b[1] = buf[10];
			
			packet_roll.b[0] = buf[11];
      packet_roll.b[1] = buf[12];
			
			packet_target_x.b[0] = buf[13];
			packet_target_x.b[1] = buf[14];
			
			packet_target_y.b[0] = buf[15];
			packet_target_y.b[1] = buf[16];
			
			packet_target_z.b[0] = buf[17];
			packet_target_z.b[1] = buf[18];
			
// 			g_TestTmpData1 = marker_in_ref.x;
// 			g_TestTmpData2 = marker_in_ref.y;
// 			g_TestTmpData3 = marker_in_ref.z;
			
			/* check sum */
      if (checksum(&buf[1], COMM_PACKET_BUF_LEN-3) == buf[COMM_PACKET_BUF_LEN-2]) {
				
					/* get marker in b1 frame */
					EulerRoateVect(&marker_in_b1, &marker_in_body, &euler_att);
					
					marker_in_ref.x = packet_x.w/1000.0f;
          marker_in_ref.y = packet_y.w/1000.0f;
					marker_in_ref.z = packet_z.w/1000.0f;
										
					cam_pos.x = DLPF(marker_in_ref.x - marker_in_b1.x,cam_pos.x,10*HZ2RAD,dt);
          cam_pos.y = DLPF(marker_in_ref.y - marker_in_b1.y,cam_pos.y,10*HZ2RAD,dt);
					cam_pos.z = DLPF(marker_in_ref.z - marker_in_b1.z,cam_pos.z,10*HZ2RAD,dt);
					
					g_CamEuler.yaw = DLPF(packet_yaw.w/1000.0f,g_CamEuler.yaw,10*HZ2RAD,dt);
				  g_CamEuler.pitch = DLPF(packet_pitch.w/1000.0f,g_CamEuler.pitch,10*HZ2RAD,dt);
				  g_CamEuler.roll = DLPF(packet_roll.w/1000.0f,g_CamEuler.roll,10*HZ2RAD,dt);
				
				  cam_vel.x = (cam_pos.x - last_cam_pos.x)/dt;
          cam_vel.y = (cam_pos.y - last_cam_pos.y)/dt;
          cam_vel.z = (cam_pos.z - last_cam_pos.z)/dt;
	
	        last_cam_pos.x = cam_pos.x;
          last_cam_pos.y = cam_pos.y;
	        last_cam_pos.z = cam_pos.z;
					
					g_CamPos.x = cam_pos.x;
					g_CamPos.y = cam_pos.y;
					g_CamPos.z = cam_pos.z;
					
					g_CamVel.x = cam_vel.x;
					g_CamVel.y = cam_vel.y;
					g_CamVel.z = cam_vel.z;
					
					g_CamTargetPos.x = DLPF(packet_target_x.w/1000.0f,g_CamTargetPos.x,1.5f*HZ2RAD,dt);
					g_CamTargetPos.y = DLPF(packet_target_y.w/1000.0f,g_CamTargetPos.y,1.5f*HZ2RAD,dt);
					g_CamTargetPos.z = DLPF(packet_target_z.w/1000.0f,g_CamTargetPos.z,1.5f*HZ2RAD,dt);
					
// 					g_TargetPos.x = g_CamTargetPos.x;
// 					g_TargetPos.y = g_CamTargetPos.y;
// 					g_TargetPos.z = g_CamTargetPos.z;
// 					
// 					g_TestTmpData1 = g_CamTargetPos.x;
// 					g_TestTmpData2 = g_CamTargetPos.y;
// 					g_TestTmpData3 = g_CamTargetPos.z;
					
          g_FlagCamUpdate = SET; /* set update cam */
      }
      else
        packet_error_cnt++;
			
			packet_total_cnt++;
			
			packet_error_rate = 100.0f*packet_error_cnt/packet_total_cnt;
			
			packet_error_rate = packet_error_rate;
			
			//g_TestTmpData1 = 1/dt;
		
			//BSP_Ser_Printf("%d,%d,%f%%,\r\n",packet_error_cnt,packet_total_cnt,100.0f*packet_error_cnt/packet_total_cnt);
    }
  }
}

/*******************************************************************************************************/
void CommTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&CommTaskTCB,
               (CPU_CHAR   *)"Comm Task",
               (OS_TASK_PTR )CommTask,
               (void       *)0,
               (OS_PRIO     )COMM_TASK_PRIO,
               (CPU_STK    *)&CommTaskStk[0],
               (CPU_STK_SIZE)COMM_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)COMM_TASK_STK_SIZE,
               (OS_MSG_QTY  )0u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate Communication Task...\n\r"));
  }
}
