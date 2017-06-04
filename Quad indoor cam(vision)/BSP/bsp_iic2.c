#include "bsp.h"
#include "cpal_i2c.h"

CPAL_TransferTypeDef I2C2_TxStruct = {pNULL,0,0,0};
CPAL_TransferTypeDef I2C2_RxStruct = {pNULL,0,0,0};
extern CPAL_InitTypeDef I2C2_DevStructure; 

BSP_OS_MUTEX  Mutex_I2C2_Lock;
BSP_OS_SEM    Sem_I2C2_TxWait;
BSP_OS_SEM    Sem_I2C2_RxWait;

/************************************
 *@brief : BSP_I2C2_Init
 *@param : none
 *************************************/
CPU_BOOLEAN BSP_I2C2_Init(void)
{	   

  /*Create OS Semaphore */
  BSP_OS_MutexCreate(&Mutex_I2C2_Lock,"I2C2 Lock"); 
  BSP_OS_SemCreate(&Sem_I2C2_TxWait,(BSP_OS_SEM_VAL) 0,"I2C2 TxWait"); 
	BSP_OS_SemCreate(&Sem_I2C2_RxWait,(BSP_OS_SEM_VAL) 0,"I2C2 RxWait"); 
	
	BSP_IntVectSet(BSP_INT_ID_I2C2_EV,  BSP_I2C2_EV_ISR);
	BSP_IntVectSet(BSP_INT_ID_I2C2_ER,  BSP_I2C2_ER_ISR);
	BSP_IntEn(BSP_INT_ID_I2C2_EV);
	BSP_IntEn(BSP_INT_ID_I2C2_ER);

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

	if(CPAL_I2C_IsDeviceReady(&I2C2_DevStructure) == CPAL_PASS)
    return DEF_OK;
	else
		return DEF_FAIL;
}  


/**********************************************
 *@brief : 写入单个字节
 *@param :   -RegAddr     寄存器地址
 * 					 -DataToWrite 要写入的数据
 * 					 -SlaveAddr   设备地址
 **********************************************/
CPU_BOOLEAN  BSP_I2C2_SingleWrite(CPU_INT08U RegAddr,CPU_INT08U DataToWrite,CPU_INT08U SlaveAddr)
{
	  BSP_OS_MutexWait(&Mutex_I2C2_Lock,0);
		I2C2_TxStruct.pbBuffer = &DataToWrite;
		I2C2_TxStruct.wNumData = 1;
		I2C2_TxStruct.wAddr1   = SlaveAddr;
		I2C2_TxStruct.wAddr2   = RegAddr;
	  I2C2_DevStructure.wCPAL_Options = 0;  
	
		if(CPAL_I2C_Write(&I2C2_DevStructure) == CPAL_PASS)
		{
			if(BSP_OS_SemWait(&Sem_I2C2_TxWait,I2C_TIMEOUT) == DEF_OK)
			{
				BSP_OS_MutexPost(&Mutex_I2C2_Lock);
				return DEF_OK;
			}
			else
			{
				BSP_OS_MutexPost(&Mutex_I2C2_Lock);
				return DEF_FAIL;
			}
		}
		else
		{
      BSP_OS_MutexPost(&Mutex_I2C2_Lock);
			return DEF_FAIL;
    }
}

/**********************************************
 *@brief : Write single byte without register address
 *@param :  
 * 					 -DataToWrite 要写入的数据
 * 					 -SlaveAddr   设备地址
 **********************************************/
CPU_BOOLEAN  BSP_I2C2_SingleWriteNoMemAddr(CPU_INT08U DataToWrite,CPU_INT08U SlaveAddr)
{
	  BSP_OS_MutexWait(&Mutex_I2C2_Lock,0);
		I2C2_TxStruct.pbBuffer = &DataToWrite;
		I2C2_TxStruct.wNumData = 1;
		I2C2_TxStruct.wAddr1   = SlaveAddr;
	  I2C2_DevStructure.wCPAL_Options = CPAL_OPT_NO_MEM_ADDR;  
	
		if(CPAL_I2C_Write(&I2C2_DevStructure) == CPAL_PASS)
		{
			if(BSP_OS_SemWait(&Sem_I2C2_TxWait,I2C_TIMEOUT) == DEF_OK)
			{
				BSP_OS_MutexPost(&Mutex_I2C2_Lock);
				return DEF_OK;
			}
			else
			{
				BSP_OS_MutexPost(&Mutex_I2C2_Lock);
				return DEF_FAIL;
			}
		}
		else
		{
      BSP_OS_MutexPost(&Mutex_I2C2_Lock);
			return DEF_FAIL;
    }
}

/**********************************************
 *@brief : 读取一个字节
 *@param : -RegAddr 寄存器地址
 * 				 -SlaveAddr 设备地址
 ***********************************************/
CPU_INT08U  BSP_I2C2_SingleRead(CPU_INT08U RegAddr,CPU_INT08U SlaveAddr)
{
	uint8_t temp;
	BSP_OS_MutexWait(&Mutex_I2C2_Lock,0);
	I2C2_RxStruct.pbBuffer = &temp;
	I2C2_RxStruct.wNumData = 1;
	I2C2_RxStruct.wAddr1   = SlaveAddr;
	I2C2_RxStruct.wAddr2   = RegAddr;
	I2C2_DevStructure.wCPAL_Options = 0;  
	
	if(CPAL_I2C_Read(&I2C2_DevStructure)==CPAL_PASS)
	{
		if(BSP_OS_SemWait(&Sem_I2C2_RxWait,I2C_TIMEOUT) == DEF_OK)
		{
			BSP_OS_MutexPost(&Mutex_I2C2_Lock);
			return temp;
		}
		else
		{
			BSP_OS_MutexPost(&Mutex_I2C2_Lock);
			return DEF_FAIL;
		}
	}
	else
	{
		BSP_OS_MutexPost(&Mutex_I2C2_Lock);
		return DEF_FAIL;
	}

}

CPU_INT08U  BSP_I2C2_SingleReadNoMemAddr(CPU_INT08U SlaveAddr)
{
	uint8_t temp;
	BSP_OS_MutexWait(&Mutex_I2C2_Lock,0);
	I2C2_RxStruct.pbBuffer = &temp;
	I2C2_RxStruct.wNumData = 1;
	I2C2_RxStruct.wAddr1   = SlaveAddr;
	I2C2_DevStructure.wCPAL_Options = CPAL_OPT_NO_MEM_ADDR;  
	
	if(CPAL_I2C_Read(&I2C2_DevStructure)==CPAL_PASS)
	{
		if(BSP_OS_SemWait(&Sem_I2C2_RxWait,I2C_TIMEOUT) == DEF_OK)
		{
			BSP_OS_MutexPost(&Mutex_I2C2_Lock);
			return temp;
		}
		else
		{
			BSP_OS_MutexPost(&Mutex_I2C2_Lock);
			return DEF_FAIL;
		}
	}
	else
	{
		BSP_OS_MutexPost(&Mutex_I2C2_Lock);
		return DEF_FAIL;
	}

}

/**********************************************
 *@brief : Read two byte(No register address)
 *@param : 
 * 				 -SlaveAddr 设备地址
 ***********************************************/
CPU_INT16U  BSP_I2C2_SingleRead16NoMemAddr(CPU_INT08U SlaveAddr)
{
	  uint8_t temp[2];
	  BSP_OS_MutexWait(&Mutex_I2C2_Lock,0);
		I2C2_RxStruct.pbBuffer = &temp[0];
		I2C2_RxStruct.wNumData = 2;
		I2C2_RxStruct.wAddr1   = SlaveAddr;
	  I2C2_DevStructure.wCPAL_Options = CPAL_OPT_NO_MEM_ADDR; 
	
		if(CPAL_I2C_Read(&I2C2_DevStructure)==CPAL_PASS)
		{
			if(BSP_OS_SemWait(&Sem_I2C2_RxWait,I2C_TIMEOUT) == DEF_OK)
			{
				BSP_OS_MutexPost(&Mutex_I2C2_Lock);
				return ((temp[0]<<8)|temp[1]);
			}
			else
			{
        BSP_OS_MutexPost(&Mutex_I2C2_Lock);
		  	return DEF_FAIL;
      }
		}
		else
		{
      BSP_OS_MutexPost(&Mutex_I2C2_Lock);
			return DEF_FAIL;
    }

}

/**********************************************
 *@brief : Read three byte(No register address)
 *@param : 
 * 				 -SlaveAddr 设备地址
 ***********************************************/
CPU_INT32U  BSP_I2C2_SingleRead24NoMemAddr(CPU_INT08U SlaveAddr)
{
	  uint8_t temp[3];
	  BSP_OS_MutexWait(&Mutex_I2C2_Lock,0);
		I2C2_RxStruct.pbBuffer = &temp[0];
		I2C2_RxStruct.wNumData = 3;
		I2C2_RxStruct.wAddr1   = SlaveAddr;
	  I2C2_DevStructure.wCPAL_Options = CPAL_OPT_NO_MEM_ADDR;  
	
		if(CPAL_I2C_Read(&I2C2_DevStructure)==CPAL_PASS)
		{
			if(BSP_OS_SemWait(&Sem_I2C2_RxWait,I2C_TIMEOUT) == DEF_OK)
			{
				BSP_OS_MutexPost(&Mutex_I2C2_Lock);
				return (((temp[0]<<16)|(temp[1]<<8))|temp[0]);
			}
			else
			{
        BSP_OS_MutexPost(&Mutex_I2C2_Lock);
		  	return DEF_FAIL;
      }
		}
		else
		{
      BSP_OS_MutexPost(&Mutex_I2C2_Lock);
			return DEF_FAIL;
    }

}
/*
**************************************************
 *              BSP_I2C_MultRead()
 * Description : Read n data from i2c bus
 * Argument    : *buf_ptr
 * Note        : None
 **************************************************
 */
CPU_BOOLEAN  BSP_I2C2_MultRead(CPU_INT08U RegAddr,CPU_INT08U SlaveAddr,CPU_INT08U *buf_ptr,CPU_INT16U nbr_of_byte)
{
	BSP_OS_MutexWait(&Mutex_I2C2_Lock,0);
	I2C2_RxStruct.pbBuffer = buf_ptr;
	I2C2_RxStruct.wNumData = nbr_of_byte;
	I2C2_RxStruct.wAddr1   = SlaveAddr;
	I2C2_RxStruct.wAddr2   = RegAddr;
	I2C2_DevStructure.wCPAL_Options = 0;  
	
	if(CPAL_I2C_Read(&I2C2_DevStructure) == CPAL_PASS)
	{
		if(BSP_OS_SemWait(&Sem_I2C2_RxWait,I2C_TIMEOUT) == DEF_OK)
		{
		  BSP_OS_MutexPost(&Mutex_I2C2_Lock);
		  return DEF_OK;
		}
		else/* Timeout */
		{
      BSP_OS_MutexPost(&Mutex_I2C2_Lock);
		  return DEF_FAIL;
    }
	}
	else
	{
		BSP_OS_MutexPost(&Mutex_I2C2_Lock);
		return DEF_FAIL;
	}
}

/*
*********************************************************************
*                            I2C2 ISR
*********************************************************************
*/
void BSP_I2C2_EV_ISR(void)
{
  I2C2_EV_IRQHandler();
}

void BSP_I2C2_ER_ISR(void)
{
  I2C2_ER_IRQHandler();
}
