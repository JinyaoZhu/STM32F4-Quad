#ifndef __BSP_SPI1_H__
#define __BSP_SPI1_H__

#define SPI1_TIMEOUT_LIMIT     1000 /* 1 = 40ns */


/* SPI2 Pin */
#define SPI1_SCK_PIN     GPIO_Pin_5
#define SPI1_MISO_PIN    GPIO_Pin_6
#define SPI1_MOSI_PIN    GPIO_Pin_7
#define SPI1_PORT        GPIOA



void BSP_SPI1_Init(void);

CPU_INT08U BSP_SPI1_SendReceiveByte(CPU_INT08U data);

void SPI1_TxTimeoutCallback(void);

void SPI1_RxTimeoutCallback(void);

#endif /* __BSP_SPI1_H__ */
