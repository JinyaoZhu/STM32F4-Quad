#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#define SPI2_TIMEOUT_LIMIT     100 /* ms */

#define SPI2_BUFFERSIZE_TX 64
#define SPI2_BUFFERSIZE_RX 64

/* SPI2 Pin */
#define SPI2_SCK_PIN     GPIO_Pin_13
#define SPI2_MISO_PIN    GPIO_Pin_14
#define SPI2_MOSI_PIN    GPIO_Pin_15
#define SPI2_PORT        GPIOB

#define SPI2_DR_Addr   ((u32)0x4000380c)


void BSP_SPI2_Init(void);

void BSP_SPI2_DMA_Configuration(void);

CPU_INT08U BSP_SPI2_WR(CPU_INT08U* data,CPU_INT08U bytesNumber,CPU_INT08U return_data);

CPU_INT08U BSP_SPI2_WriteByte(CPU_INT08U data);

CPU_INT08U BSP_SPI2_ReceiveSendByte(CPU_INT16U  num );

void SPI2_TxTimeoutCallback(void);

void SPI2_RxTimeoutCallback(void);

void  BSP_SPI1_DMA_CH4_ISR_Handler (void);

void  BSP_SPI1_DMA_CH5_ISR_Handler (void);

#endif /* __BSP_SPI_H__ */
