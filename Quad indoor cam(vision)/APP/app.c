/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <stm32f4xx.h>
#include  <includes.h>
#include "sensor_update.h"
#include "GlobalVariable.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

/* ----------------- APPLICATION GLOBALS -------------- */
static  OS_TCB   AppTaskStartTCB;
__align(8)  CPU_STK  AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
/* ------------ FLOATING POINT TEST TASK -------------- */

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart          (void     *p_arg);
static  void  AppTaskCreate         (void);
static  void  AppObjCreate          (void);


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

int main(void)
{
  OS_ERR  err;


  BSP_IntDisAll();                                            /* Disable all interrupts.                              */

  CPU_Init();                                                 /* Initialize the uC/CPU Services                       */
  Mem_Init();                                                 /* Initialize Memory Management Module                  */
  Math_Init();                                                /* Initialize Mathematical Module                       */

  OSInit(&err);                                               /* Init uC/OS-III.                                      */

  OSTaskCreate((OS_TCB       *)&AppTaskStartTCB,              /* Create the start task                                */
               (CPU_CHAR     *)"App Task Start",
               (OS_TASK_PTR   )AppTaskStart,
               (void         *)0u,
               (OS_PRIO       )APP_CFG_TASK_START_PRIO,
               (CPU_STK      *)&AppTaskStartStk[0u],
               (CPU_STK_SIZE  )AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE / 10u],
               (CPU_STK_SIZE  )APP_CFG_TASK_START_STK_SIZE,
               (OS_MSG_QTY    )0u,
               (OS_TICK       )0u,
               (void         *)0u,
               (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               (OS_ERR       *)&err);

  OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

  (void)&err;

  return (0u);
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
  OS_ERR  err;

  (void)p_arg;

  App_OS_SetAllHooks();
	
  CORDIC_Init();
	
  BSP_Init();                                                 /* Initialize BSP functions                             */
  BSP_Tick_Init();                                            /* Initialize Tick Services.                            */

#if OS_CFG_STAT_TASK_EN > 0u
  OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  CPU_IntDisMeasMaxCurReset();
#endif

	BSP_PWM_Init(TIM3_PWM_LOWEST);
	
	BSP_PWM2_Init(TIM4_PWM_HIGHEST);
	
  BSP_PWM_IN_InitTimer();

  BSP_Ser_Init(115200);
	
	BSP_USART2_Init();
	
	BSP_I2C3_Init(); /* Conneted to HMC5883 */

  BSP_LED_Off(0u);                                            /* Turn Off LEDs after initialization                   */

  APP_TRACE_DBG(("Creating Application Kernel Objects...\n\r"));
  AppObjCreate();                                             /* Create Applicaiton kernel objects                    */

  APP_TRACE_DBG(("Creating Application Tasks...\n\r"));
  AppTaskCreate();                                            /* Create Application tasks                             */

  //OSTaskDel(&AppTaskStartTCB,&err);                           /* Kill itself */

  while (DEF_TRUE) {                                          /* Task body, always written as an infinite loop.       */

    BSP_OS_TimeDlyMs(500);
  }
}


/*
*********************************************************************************************************
*                                          AppTaskCreate()
*
* Description : Create application tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{
  if (g_Flag_ESC_Calibrated == RESET) {
    ESCCalibrateTaskCreate();
		while(g_Flag_ESC_Calibrated == RESET) 
		  BSP_OS_TimeDlyMs(10u);
  }
	
	StateCheckTaskCreate();
	
	RCTaskCreate();

	ControlTaskCreate();

	PosControlTaskCreate();
	
  AttitudeTaskCreate();
	
	if(g_FlagAccCalFinished == RESET)
		AccCalibrateTaskCreate();	
	
	CommTaskCreate();
	
	PrintfTaskCreate();
	
	NavigateTaskCreate();
}


/*
*********************************************************************************************************
*                                          AppObjCreate()
*
* Description : Create application kernel objects tasks.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : AppTaskStart()
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  AppObjCreate (void)
{
  uint8_t status;
	
	status = BSP_OS_SemCreate(&Sem_SerPrintUpdate, 0, "Sem_SerPrintUpdate");
	if (status == DEF_FAIL){
		BSP_Ser_Printf("AppObjCreate error!");
    for (;;);
	}
	
  status = BSP_OS_SemCreate(&Sem_PosControlUpdate, 0, "Sem_PosControlUpdate");
  if (status == DEF_FAIL){
		BSP_Ser_Printf("AppObjCreate error!");
    for (;;);
	}
	
	status = BSP_OS_SemCreate(&Sem_AttControlUpdate, 0, "Sem_AttControlUpdate");
  if (status == DEF_FAIL){
		BSP_Ser_Printf("AppObjCreate error!");
    for (;;);
	}
	
}



