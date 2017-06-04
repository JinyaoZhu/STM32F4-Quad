/**
  ******************************************************************************
  * @file    Project/STM32_CPAL_Template/cpal_usercallback.c
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    21-December-2012
  * @brief   This file provides all the CPAL UserCallback functions .
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cpal_i2c.h"
#include "stdio.h"
#include "includes.h"

extern CPAL_InitTypeDef I2C2_DevStructure; 
extern BSP_OS_SEM    Sem_I2C2_TxWait;
extern BSP_OS_SEM    Sem_I2C2_RxWait;

extern CPAL_InitTypeDef I2C3_DevStructure; 
extern BSP_OS_SEM    Sem_I2C3_TxWait;
extern BSP_OS_SEM    Sem_I2C3_RxWait;

extern I2C_TypeDef* CPAL_I2C_DEVICE[];
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



/*------------------------------------------------------------------------------
                     CPAL User Callbacks implementations 
------------------------------------------------------------------------------*/


/*=========== Timeout UserCallback ===========*/

extern CPAL_TransferTypeDef I2C3_TxStruct;
extern CPAL_TransferTypeDef I2C3_RxStruct;

extern CPAL_TransferTypeDef I2C2_TxStruct;
extern CPAL_TransferTypeDef I2C2_RxStruct;
/**
  * @brief  User callback that manages the Timeout error.
  * @param  pDevInitStruct .
  * @retval None.
  */
uint32_t CPAL_TIMEOUT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

	if((pDevInitStruct->CPAL_Dev) == CPAL_I2C3)
	{
		/* Generate STOP */
    __CPAL_I2C_HAL_STOP(CPAL_I2C3);
				
 		//BSP_Ser_Printf("\n\r<CPAL_TIMEOUT_UserCallback> I2C3\r\n");  
		//CPAL_I2C_DeInit(&I2C1_DevStructure);
		CPAL_I2C_StructInit(&I2C3_DevStructure);
		I2C3_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C3_SPEED;
		I2C3_DevStructure.CPAL_Direction     = CPAL_DIRECTION_TXRX;               /* Transmitter and Receiver direction selected */
		I2C3_DevStructure.CPAL_Mode          = CPAL_MODE_MASTER;                  /* Mode Master selected */
		I2C3_DevStructure.CPAL_ProgModel     = CPAL_PROGMODEL_INTERRUPT;          /* Interrupt Programming Model selected */
		I2C3_DevStructure.pCPAL_TransferTx   = &I2C3_TxStruct;                    /* Point pCPAL_TransferTx to a Null pointer */
		I2C3_DevStructure.pCPAL_TransferRx   = &I2C3_RxStruct;                    /* Point pCPAL_TransferRx to a Null pointer */ 
		I2C3_DevStructure.CPAL_State         = CPAL_STATE_READY;                  /* Device Disabled */
		I2C3_DevStructure.wCPAL_DevError     = CPAL_I2C_ERR_NONE;                 /* No Device Error */
		I2C3_DevStructure.wCPAL_Options      = 0;             
		I2C3_DevStructure.wCPAL_Timeout      = I2C_TIMEOUT;                      /* Set timeout value to CPAL_I2C_TIMEOUT_DEFAULT */
		CPAL_I2C_Init(&I2C3_DevStructure);
 	}
	
	if((pDevInitStruct->CPAL_Dev) == CPAL_I2C2)
	{
		/* Generate STOP */
    __CPAL_I2C_HAL_STOP(CPAL_I2C2);
				
 		//BSP_Ser_Printf("\n\r<CPAL_TIMEOUT_UserCallback> I2C3\r\n");  
		//CPAL_I2C_DeInit(&I2C1_DevStructure);
		CPAL_I2C_StructInit(&I2C2_DevStructure);
		I2C2_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C2_SPEED;
		I2C2_DevStructure.CPAL_Direction     = CPAL_DIRECTION_TXRX;               /* Transmitter and Receiver direction selected */
		I2C2_DevStructure.CPAL_Mode          = CPAL_MODE_MASTER;                  /* Mode Master selected */
		I2C2_DevStructure.CPAL_ProgModel     = CPAL_PROGMODEL_INTERRUPT;          /* Interrupt Programming Model selected */
		I2C2_DevStructure.pCPAL_TransferTx   = &I2C2_TxStruct;                    /* Point pCPAL_TransferTx to a Null pointer */
		I2C2_DevStructure.pCPAL_TransferRx   = &I2C2_RxStruct;                    /* Point pCPAL_TransferRx to a Null pointer */ 
		I2C2_DevStructure.CPAL_State         = CPAL_STATE_READY;                  /* Device Disabled */
		I2C2_DevStructure.wCPAL_DevError     = CPAL_I2C_ERR_NONE;                 /* No Device Error */
		I2C2_DevStructure.wCPAL_Options      = 0;             
		I2C2_DevStructure.wCPAL_Timeout      = I2C_TIMEOUT;                      /* Set timeout value to CPAL_I2C_TIMEOUT_DEFAULT */
		CPAL_I2C_Init(&I2C2_DevStructure);
 	}
  
  return CPAL_PASS;
}


/*=========== Transfer UserCallback ===========*/


/**
  * @brief  Manages the End of Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
void CPAL_I2C_TXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
	if(pDevInitStruct->CPAL_Dev == CPAL_I2C3)
    BSP_OS_SemPost(&Sem_I2C3_TxWait);
	
	if(pDevInitStruct->CPAL_Dev == CPAL_I2C2)
    BSP_OS_SemPost(&Sem_I2C2_TxWait);
}


/**
  * @brief  Manages the End of Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */ 
void CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
	if(pDevInitStruct->CPAL_Dev == CPAL_I2C3)
    BSP_OS_SemPost(&Sem_I2C3_RxWait);
	
	if(pDevInitStruct->CPAL_Dev == CPAL_I2C2)
    BSP_OS_SemPost(&Sem_I2C2_RxWait);
}


/**
  * @brief  Manages Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*void CPAL_I2C_TX_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
 
}*/


/**
  * @brief  Manages Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */ 
/*void CPAL_I2C_RX_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
 
}*/


/**
  * @brief  Manages the End of DMA Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*void CPAL_I2C_DMATXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}
*/

/**
  * @brief  Manages the Half of DMA Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*void CPAL_I2C_DMATXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/


/**
  * @brief  Manages Error of DMA Tx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*void CPAL_I2C_DMATXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/


/**
  * @brief  Manages the End of DMA Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */
/*
void CPAL_I2C_DMARXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{

}
*/


/**
  * @brief  Manages the Half of DMA Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */ 
/*void CPAL_I2C_DMARXHT_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/


/**
  * @brief  Manages Error of DMA Rx transfer event.
  * @param  pDevInitStruct 
  * @retval None
  */ 
/*void CPAL_I2C_DMARXTE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/


/*=========== Error UserCallback ===========*/


/**
  * @brief  User callback that manages the I2C device errors.
  * @note   Make sure that the define USE_SINGLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInitStruct. 
  * @param  DeviceError.
  * @retval None
  */ 
void CPAL_I2C_ERR_UserCallback(CPAL_DevTypeDef pDevInstance, uint32_t DeviceError)
{	
  if (pDevInstance == CPAL_I2C3)
  {
		/* Generate STOP */
    __CPAL_I2C_HAL_STOP(CPAL_I2C3);
				
 		//BSP_Ser_Printf("\n\r<CPAL_I2C_ERR_UserCallback> I2C1\r\n");  
		//CPAL_I2C_DeInit(&I2C1_DevStructure);
		CPAL_I2C_StructInit(&I2C3_DevStructure);
		I2C3_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C3_SPEED;
		I2C3_DevStructure.CPAL_Direction     = CPAL_DIRECTION_TXRX;               /* Transmitter and Receiver direction selected */
		I2C3_DevStructure.CPAL_Mode          = CPAL_MODE_MASTER;                  /* Mode Master selected */
		I2C3_DevStructure.CPAL_ProgModel     = CPAL_PROGMODEL_INTERRUPT;          /* Interrupt Programming Model selected */
		I2C3_DevStructure.pCPAL_TransferTx   = &I2C3_TxStruct;                    /* Point pCPAL_TransferTx to a Null pointer */
		I2C3_DevStructure.pCPAL_TransferRx   = &I2C3_RxStruct;                    /* Point pCPAL_TransferRx to a Null pointer */ 
		I2C3_DevStructure.CPAL_State         = CPAL_STATE_READY;                  /* Device Disabled */
		I2C3_DevStructure.wCPAL_DevError     = CPAL_I2C_ERR_NONE;                 /* No Device Error */
		I2C3_DevStructure.wCPAL_Options      = 0;             
		I2C3_DevStructure.wCPAL_Timeout      = I2C_TIMEOUT;                      /* Set timeout value to CPAL_I2C_TIMEOUT_DEFAULT */
		CPAL_I2C_Init(&I2C3_DevStructure);
  }
	
  if (pDevInstance == CPAL_I2C2)
  {
		/* Generate STOP */
    __CPAL_I2C_HAL_STOP(CPAL_I2C2);
				
 		//BSP_Ser_Printf("\n\r<CPAL_I2C_ERR_UserCallback> I2C1\r\n");  
		//CPAL_I2C_DeInit(&I2C1_DevStructure);
		CPAL_I2C_StructInit(&I2C2_DevStructure);
		I2C2_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = I2C2_SPEED;
		I2C2_DevStructure.CPAL_Direction     = CPAL_DIRECTION_TXRX;               /* Transmitter and Receiver direction selected */
		I2C2_DevStructure.CPAL_Mode          = CPAL_MODE_MASTER;                  /* Mode Master selected */
		I2C2_DevStructure.CPAL_ProgModel     = CPAL_PROGMODEL_INTERRUPT;          /* Interrupt Programming Model selected */
		I2C2_DevStructure.pCPAL_TransferTx   = &I2C2_TxStruct;                    /* Point pCPAL_TransferTx to a Null pointer */
		I2C2_DevStructure.pCPAL_TransferRx   = &I2C2_RxStruct;                    /* Point pCPAL_TransferRx to a Null pointer */ 
		I2C2_DevStructure.CPAL_State         = CPAL_STATE_READY;                  /* Device Disabled */
		I2C2_DevStructure.wCPAL_DevError     = CPAL_I2C_ERR_NONE;                 /* No Device Error */
		I2C2_DevStructure.wCPAL_Options      = 0;             
		I2C2_DevStructure.wCPAL_Timeout      = I2C_TIMEOUT;                      /* Set timeout value to CPAL_I2C_TIMEOUT_DEFAULT */
		CPAL_I2C_Init(&I2C2_DevStructure);
  }
    
}

/**
  * @brief  User callback that manages BERR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */  
/*void CPAL_I2C_BERR_UserCallback(CPAL_DevTypeDef pDevInstance)
{
   
}*/


/**
  * @brief  User callback that manages ARLO I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_ARLO_UserCallback(CPAL_DevTypeDef pDevInstance)
{
   
}*/


/**
  * @brief  User callback that manages OVR I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */
/*void CPAL_I2C_OVR_UserCallback(CPAL_DevTypeDef pDevInstance)
{
   
}*/


/**
  * @brief  User callback that manages AF I2C device errors.
  * @note   Make sure that the define USE_MULTIPLE_ERROR_CALLBACK is uncommented in
  *         the cpal_conf.h file, otherwise this callback will not be functional.
  * @param  pDevInstance.
  * @retval None
  */ 
/*void CPAL_I2C_AF_UserCallback(CPAL_DevTypeDef pDevInstance)
{
   
}*/


/*=========== Addressing Mode UserCallback ===========*/


/**
  * @brief  User callback that manage General Call Addressing mode.
  * @param  pDevInitStruct 
  * @retval None
  */ 
/*void CPAL_I2C_GENCALL_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/


/**
  * @brief  User callback that manage Dual Address Addressing mode.
  * @param  pDevInitStruct 
  * @retval None
  */  
/*void CPAL_I2C_DUALF_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
   
}*/
 
 
/*=========== Listen Mode UserCallback ===========*/


/**
  * @brief  User callback that manage slave read operation.
  * @param  pDevInitStruct 
  * @retval None
  */ 
/*void CPAL_I2C_SLAVE_READ_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
}*/


/**
  * @brief  User callback that manage slave write operation.
  * @param  pDevInitStruct 
  * @retval None
  */  
/*void CPAL_I2C_SLAVE_WRITE_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{  
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
