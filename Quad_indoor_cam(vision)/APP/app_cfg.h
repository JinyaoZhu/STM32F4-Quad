/*
*********************************************************************************************************
*                                              uC/OS-II
*                                        The Real-Time Kernel
*
*                             (c) Copyright 2012; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               uC/OS-II is provided in source form for FREE evaluation, for educational
*               use or peaceful research.  If you plan on using uC/OS-II in a commercial
*               product you need to contact Micrium to properly license its use in your
*               product.  We provide ALL the source code for your convenience and to
*               help you experience uC/OS-II.  The fact that the source code is provided
*               does NOT mean that you can use it without paying a licensing fee.
*
*               Knowledge of the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                       APPLICATION CONFIGURATION
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/

#ifndef  APP_CFG_MODULE_PRESENT
#define  APP_CFG_MODULE_PRESENT


/*
*********************************************************************************************************
*                                       ADDITIONAL uC/MODULE ENABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                    BSP CONFIGURATION: RS-232
*********************************************************************************************************
*/

#define  BSP_CFG_SER_COMM_SEL             			BSP_SER_COMM_UART_01
#define  BSP_CFG_TS_TMR_SEL                             2

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  APP_CFG_TASK_START_PRIO                           3u
#define  ESC_CAL_TASK_STK_PRIO                             4u
#define  CONTROL_TASK_PRIO                                 5u
#define  POS_CONTROL_TASK_PRIO                             6u
#define  ATTITUDE_TASK_PRIO                                7u
#define  NAVIGATE_TASK_PRIO                                7u
#define  RC_TASK_PRIO                                      8u
#define  COMM_TASK_PRIO                                    8u
#define  SONAR_TASK_PRIO                                   9u
#define  ACC_CALIBRATE_TASK_PRIO                           9u
#define  STATE_CHECK_TASK_PRIO                             11u
#define  PRINTF_TASK_PRIO                                  12u

/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*********************************************************************************************************
*/

#define  APP_CFG_TASK_START_STK_SIZE                     512u
#define  STATE_CHECK_TASK_STK_SIZE                       512u
#define  ATTITUDE_TASK_STK_SIZE                          512u
#define  PRINTF_TASK_STK_SIZE                            512u
#define  CONTROL_TASK_STK_SIZE                           512u
#define  RC_TASK_STK_SIZE                                512u
#define  ESC_CAL_TASK_STK_SIZE                           512u
#define  SONAR_TASK_STK_SIZE                             256u
#define  ACC_CALIBRATE_TASK_STK_SIZE                     256u
#define  COMM_TASK_STK_SIZE                              512u
#define  POS_CONTROL_TASK_STK_SIZE                       512u
#define  NAVIGATE_TASK_STK_SIZE                          512u

/*
*********************************************************************************************************
*                                            TASK STACK SIZES LIMIT
*********************************************************************************************************
*/

#define  APP_CFG_TASK_START_STK_SIZE_PCT_FULL             90u
#define  APP_CFG_TASK_START_STK_SIZE_LIMIT       (APP_CFG_TASK_START_STK_SIZE     * (100u - APP_CFG_TASK_START_STK_SIZE_PCT_FULL))    / 100u
#define  APP_CFG_TASK_BLINKY_STK_SIZE_LIMIT      (APP_CFG_TASK_BLINKY_STK_SIZE    * (100u - APP_CFG_TASK_START_STK_SIZE_PCT_FULL))    / 100u


/*
*********************************************************************************************************
*                                       TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/

#ifndef  TRACE_LEVEL_OFF
#define  TRACE_LEVEL_OFF                0
#endif

#ifndef  TRACE_LEVEL_INFO
#define  TRACE_LEVEL_INFO               1
#endif

#ifndef  TRACE_LEVEL_DBG
#define  TRACE_LEVEL_DBG                2
#endif

#define  APP_CFG_TRACE_LEVEL             TRACE_LEVEL_DBG
#define  APP_CFG_TRACE                   BSP_Ser_Printf

#define  BSP_CFG_TRACE_LEVEL             TRACE_LEVEL_OFF
#define  BSP_CFG_TRACE                   BSP_Ser_Printf

#define  APP_TRACE_INFO(x)               ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_CFG_TRACE x) : (void)0)
#define  APP_TRACE_DBG(x)                ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_DBG)   ? (void)(APP_CFG_TRACE x) : (void)0)

#define  BSP_TRACE_INFO(x)               ((BSP_CFG_TRACE_LEVEL  >= TRACE_LEVEL_INFO) ? (void)(BSP_CFG_TRACE x) : (void)0)
#define  BSP_TRACE_DBG(x)                ((BSP_CFG_TRACE_LEVEL  >= TRACE_LEVEL_DBG)  ? (void)(BSP_CFG_TRACE x) : (void)0)

#endif
