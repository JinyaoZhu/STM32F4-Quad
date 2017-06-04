#include "includes.h"
#include "GlobalVariable.h"

__align(8) static  CPU_STK  StateCheckTaskStk[STATE_CHECK_TASK_STK_SIZE];
OS_TCB  StateCheckTaskTCB;


static void DisplaySystemInfo(void);

void StateCheckTask(void *p_arg)
{
	CPU_TS32 timestamp_old = 0;
	CPU_FP32 led_blink_time_accumulate = 0;
	CPU_INT16U ser_update_cnt = 0;
	CPU_INT16U stick_cnt = 0;
  (void)p_arg;
	
	DisplaySystemInfo();
	
	timestamp_old = CPU_TS_Get32();
	
  while (1)
  {
		led_blink_time_accumulate += 1000.0f*Get_dt(&timestamp_old);
		
		if(led_blink_time_accumulate > g_LEDBlinkPeriod_ms){
			led_blink_time_accumulate = 0;
		  BSP_LED_Toggle(2u);
		}
	 
	 		/* armed LED */
		if(g_FlagArmed == SET)
			BSP_LED_On(1u);
		else
			BSP_LED_Off(1u);
		
		/* post to printf task */
		if(ser_update_cnt >= 2){
			ser_update_cnt = 0;
		  BSP_OS_SemPost(&Sem_SerPrintUpdate); 
		}
		else
			ser_update_cnt++;
		
		/* trigger stick */
		if((g_FlagTriggerStick == SET)&&(g_FlagPosHoldXY==SET)){
			if(stick_cnt < 5){
        stick_cnt++;
				BSP_PWM2_Set(TIM4_PWM_LOWEST + TIM3_PWM_WIDTH*0.5);
			}
			else if(stick_cnt<20){
        stick_cnt++;
				BSP_PWM2_Set(TIM4_PWM_HIGHEST);
      }
			else{
        stick_cnt=0;
				g_FlagTriggerStick = RESET;
      }
    }
		
		
    BSP_OS_TimeDlyMs(10);
  }
}






void StateCheckTaskCreate(void)
{
  OS_ERR err;
  OSTaskCreate((OS_TCB     *)&StateCheckTaskTCB,
               (CPU_CHAR   *)"StateCheckTask",
               (OS_TASK_PTR )StateCheckTask,
               (void       *)0,
               (OS_PRIO     )STATE_CHECK_TASK_PRIO,
               (CPU_STK    *)&StateCheckTaskStk[0],
               (CPU_STK_SIZE)STATE_CHECK_TASK_STK_SIZE / 10,
               (CPU_STK_SIZE)STATE_CHECK_TASK_STK_SIZE,
               (OS_MSG_QTY  )5u,
               (OS_TICK     )0u,
               (void       *)0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR     *)&err);
  if (err == OS_ERR_NONE)
  {
    APP_TRACE_INFO(("\n\rCreate StateCheck Task...\n\r"));
  }
	else
		for(;;);
}


/*
******************************************************************
*                        DisplaySystemInfo()
* Description :
*
* Parameter   : none.
*
* Caller      : Application.
******************************************************************
*/
static void DisplaySystemInfo(void)
{
  RCC_ClocksTypeDef  rcc_clocks;
  RCC_GetClocksFreq(&rcc_clocks);

  BSP_Ser_Printf("\n\rSystem Info: \n\r");
  BSP_Ser_Printf("SYSCLK_Frequency = %ldHz \r\n", rcc_clocks.SYSCLK_Frequency);
  BSP_Ser_Printf("HCLK_Frequency   = %ldHz \r\n", rcc_clocks.HCLK_Frequency);
  BSP_Ser_Printf("PCLK1_Frequency  = %ldHz \r\n", rcc_clocks.PCLK1_Frequency);
  BSP_Ser_Printf("PCLK2_Frequency  = %ldHz \r\n", rcc_clocks.PCLK2_Frequency);
}

