#include "bsp.h"

BSP_OS_SEM    Sem_SPI1RxWait;
BSP_OS_SEM    Sem_SPI1TxWait;
BSP_OS_MUTEX  Mutex_SPI1Lock;

/*
*******************************************************
*                  BSP_SPI1_Init()
*******************************************************
*/
void BSP_SPI1_Init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	
 /* ------------------ INIT OS OBJECTS ----------------- */
  BSP_OS_SemCreate(&Sem_SPI1TxWait, 0 , "SPI1 Wait");
	BSP_OS_SemCreate(&Sem_SPI1RxWait, 0 , "SPI1 Wait");
  BSP_OS_MutexCreate(&Mutex_SPI1Lock, "SPI1 Lock");     

	SPI_I2S_DeInit(SPI1);
	
  /* Enable SPI and GPIO clocks */
  BSP_PeriphEn(BSP_PERIPH_ID_SPI1);
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOA);	

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_MISO_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
  GPIO_Init(SPI1_PORT, &GPIO_InitStructure);
	
  /* SPI configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
	/* Disable SPI1 CRC calculation */
  SPI_CalculateCRC(SPI1, DISABLE);
	
  /* Enable the SPI  */
  SPI_Cmd(SPI1, ENABLE);
}

/*
*******************************************************
*                  BSP_SPI1_SendReceiveByte()
*******************************************************
*/
CPU_INT08U BSP_SPI1_SendReceiveByte(CPU_INT08U data)
{
	uint32_t time_out = 0;
	/* Tx */
  while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)==RESET)
	{
    time_out++;
		if(time_out>SPI1_TIMEOUT_LIMIT){
			SPI1_TxTimeoutCallback();
			break;
		}
  }
	time_out = 0;
  SPI_SendData(SPI1,data);

	/* Rx */
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET)
	{
    time_out++;
		if(time_out>SPI1_TIMEOUT_LIMIT){
			SPI1_RxTimeoutCallback();
			break;
		}
  }
  return SPI_ReceiveData(SPI1);
}

/*
*********************************************************************************************************
*                                            SPI Timeout Callback
*********************************************************************************************************
*/
void SPI1_TxTimeoutCallback(void)
{
  BSP_Ser_Printf("SPI1_TxTimeoutCallback\r\n");
}

void SPI1_RxTimeoutCallback(void)
{
  BSP_Ser_Printf("SPI1_RxTimeoutCallback\r\n");
}

