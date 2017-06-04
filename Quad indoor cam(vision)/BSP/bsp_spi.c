#include "bsp.h"

__IO CPU_INT08U SPI2_Buffer_Rx[SPI2_BUFFERSIZE_RX] = {0};
__IO CPU_INT08U SPI2_Buffer_Tx[SPI2_BUFFERSIZE_TX] = {0};

BSP_OS_SEM    Sem_SPI2RxWait;
BSP_OS_SEM    Sem_SPI2TxWait;
BSP_OS_MUTEX  Mutex_SPI2Lock;

/*
*******************************************************
*                  BSP_SPI2_Init()
*******************************************************
*/
void BSP_SPI2_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
 /* ------------------ INIT OS OBJECTS ----------------- */
  BSP_OS_SemCreate(&Sem_SPI2TxWait, 0 , "SPI2 Wait");
	BSP_OS_SemCreate(&Sem_SPI2RxWait, 0 , "SPI2 Wait");
  BSP_OS_MutexCreate(&Mutex_SPI2Lock, "SPI2 Lock");     

  /* Enable SPI and GPIO clocks */
  BSP_PeriphEn(BSP_PERIPH_ID_SPI2);
	BSP_PeriphEn(BSP_PERIPH_ID_IOPB);	
	
	BSP_SPI2_DMA_Configuration();

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN | SPI2_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI2_PORT, &GPIO_InitStructure);

  /* Configure MISO */
  GPIO_InitStructure.GPIO_Pin = SPI2_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SPI2_PORT, &GPIO_InitStructure);
	
  /* SPI configuration */
	SPI_I2S_DeInit(SPI2);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;  
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2, &SPI_InitStructure);
	/* Disable SPI2 CRC calculation */
  SPI_CalculateCRC(SPI2, DISABLE);
	
	
	BSP_IntVectSet(BSP_INT_ID_DMA1_CH4, BSP_SPI1_DMA_CH4_ISR_Handler);
	BSP_IntPrioSet(BSP_INT_ID_DMA1_CH4,BSP_INT_DMA1_CH4_PRIO);
  BSP_IntEn(BSP_INT_ID_DMA1_CH4);
	BSP_IntVectSet(BSP_INT_ID_DMA1_CH5, BSP_SPI1_DMA_CH5_ISR_Handler);
	BSP_IntPrioSet(BSP_INT_ID_DMA1_CH5,BSP_INT_DMA1_CH5_PRIO);
  BSP_IntEn(BSP_INT_ID_DMA1_CH5);
	
  /* Enable the SPI  */
  SPI_Cmd(SPI2, ENABLE);
	
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
   
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
}


/*
*******************************************************
*            BSP_SPI2_DMA_Configuration()
*******************************************************
*/
void BSP_SPI2_DMA_Configuration(void)
{
    DMA_InitTypeDef  DMA_InitStructure;
  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
     
    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI2_DR_Addr;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI2_Buffer_Rx;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = SPI2_BUFFERSIZE_RX;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	  /* Enable DMA1 Channel 4 Translate Complete interrupt */
	  DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);

   
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI2_DR_Addr;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)SPI2_Buffer_Tx;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = SPI2_BUFFERSIZE_TX;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	  /* Enable DMA1 Channel 4 Translate Complete interrupt */
	  DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
}


/*
*******************************************************
*                    BSP_SPI2_WR()
*******************************************************
*/
CPU_INT08U BSP_SPI2_WR(CPU_INT08U* data,CPU_INT08U bytesNumber,CPU_INT08U return_data)
{
    CPU_INT08U byte = 0;
		CPU_INT08U status;
    for(byte = 0; byte < bytesNumber; byte++)
    {
      SPI2_Buffer_Tx[byte] = data[byte];
    }

    status = BSP_SPI2_ReceiveSendByte(bytesNumber);
		
		if(return_data == 1)
		{
			for(byte = 0; byte < bytesNumber; byte++)
			{
				data[byte]=SPI2_Buffer_Rx[byte];
			}
		}
		return status;
}

/*
*******************************************************
*                  BSP_SPI2_WriteByte()
*******************************************************
*/
uint8_t BSP_SPI2_WriteByte(CPU_INT08U data)
{	
   BSP_SPI2_WR(&data,1,1);
   return  data;
}

/*
*******************************************************
*                  BSP_SPI2_ReceiveSendByte()
*******************************************************
*/
CPU_INT08U BSP_SPI2_ReceiveSendByte(CPU_INT16U  num )
{
	CPU_INT08U status = DEF_OK;
    DMA_SetCurrDataCounter(DMA1_Channel5, num);
    DMA_SetCurrDataCounter(DMA1_Channel4, num);

    DMA_Cmd(DMA1_Channel4, ENABLE);
    DMA_Cmd(DMA1_Channel5, ENABLE);   
    
	  if(BSP_OS_SemWait(&Sem_SPI2TxWait, SPI2_TIMEOUT_LIMIT) != DEF_OK)
		{
			SPI2_TxTimeoutCallback();
			status = DEF_FAIL;
		}
		if(BSP_OS_SemWait(&Sem_SPI2RxWait, SPI2_TIMEOUT_LIMIT) != DEF_OK)
		{
			SPI2_RxTimeoutCallback();
			status = DEF_FAIL;
		}
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_Cmd(DMA1_Channel4, DISABLE);
		
		return status;
}

/*
*********************************************************************************************************
*                                            SPI Timeout Callback
*********************************************************************************************************
*/
void SPI2_TxTimeoutCallback(void)
{
  BSP_Ser_Printf("SPI2_TxTimeoutCallback\r\n");
}

void SPI2_RxTimeoutCallback(void)
{
  BSP_Ser_Printf("SPI2_RxTimeoutCallback\r\n");
}

/*
*********************************************************************************************************
*                                         BSP_SPI1_DMA_CH4_ISR_Handler()
*
* Description : DMA1Channel4 ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_SPI1_DMA_CH4_ISR_Handler (void)
{
    if(DMA_GetITStatus(DMA1_IT_TC4) == SET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        BSP_OS_SemPost(&Sem_SPI2RxWait);                         /* Post to the sempahore                              */
    }
}

/*
*********************************************************************************************************
*                                         BSP_SPI1_DMA_CH5_ISR_Handler()
*
* Description : DMA1Channel5 ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_SPI1_DMA_CH5_ISR_Handler (void)
{
    if(DMA_GetITStatus(DMA1_IT_TC5) == SET)
    {
       DMA_ClearITPendingBit(DMA1_IT_TC5);
       BSP_OS_SemPost(&Sem_SPI2TxWait);                         /* Post to the sempahore                              */
    }
}

