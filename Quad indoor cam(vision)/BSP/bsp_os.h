/*
*********************************************************************************************************
*                                     MICIRUM BOARD SUPPORT PACKAGE
*
*                             (c) Copyright 2009; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*                                               OS LAYER
*
* Filename      : bsp_os.h
* Version       : V1.00
* Programmer(s) : FT
*                 EHS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
*********************************************************************************************************
*/

#ifndef  BSP_OS_PRESENT
#define  BSP_OS_PRESENT


/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/

#include  <os.h>


/*
*********************************************************************************************************
*                                                 EXTERNS
*********************************************************************************************************
*/

#ifdef   BSP_OS_MODULE
#define  BSP_OS_EXT
#else
#define  BSP_OS_EXT  extern
#endif


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             DATA TYPES
*********************************************************************************************************
*/

typedef   OS_SEM       BSP_OS_SEM;
typedef   OS_SEM_CTR   BSP_OS_SEM_VAL;
typedef   OS_MUTEX     BSP_OS_MUTEX;
typedef   OS_SEM_CTR   BSP_OS_SEM_CTR;


/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/

CPU_BOOLEAN  BSP_OS_SemCreate          (BSP_OS_SEM     *p_sem,
                                        BSP_OS_SEM_VAL  sem_val,
                                        CPU_CHAR       *p_sem_name);

CPU_BOOLEAN  BSP_OS_SemWait            (BSP_OS_SEM     *p_sem,
                                        CPU_INT32U      dly_ms);

CPU_BOOLEAN  BSP_OS_SemPost            (BSP_OS_SEM     *p_sem);

CPU_BOOLEAN  BSP_OS_SemSet (BSP_OS_SEM *p_sem , BSP_OS_SEM_CTR cnt);

CPU_BOOLEAN  BSP_OS_MutexCreate        (BSP_OS_MUTEX  *p_mutex,
                                        CPU_CHAR      *p_mutex_name);
																				
CPU_BOOLEAN  BSP_OS_MutexWait          (BSP_OS_MUTEX  *p_mutex,
                                        CPU_INT32U     dly_ms);
																				
CPU_BOOLEAN  BSP_OS_MutexPost          (BSP_OS_MUTEX *p_mutex);																				
																				
void         BSP_OS_TmrTickInit        (CPU_INT32U      tick_rate);
void         BSP_OS_TimeDlyMs          (CPU_INT32U      dly_ms);


/*
*********************************************************************************************************
*                                          CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif                                                          /* End of module include.                               */
