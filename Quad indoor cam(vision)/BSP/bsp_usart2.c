#include "bsp.h"


BSP_OS_SEM Sem_USART2_TxWait;
BSP_OS_SEM Sem_USART2_RxWait;
BSP_OS_SEM Sem_USART2_Lock;

/*
********************************************************************
                           BSP_USART2_Init()
********************************************************************
*/
void BSP_USART2_Init(void)
{
  GPIO_InitTypeDef        gpio_init;
  USART_InitTypeDef       usart_init;
  USART_ClockInitTypeDef  usart_clk_init;

  /* ------------------ INIT OS OBJECTS ----------------- */
  BSP_OS_SemCreate(&Sem_USART2_TxWait,   0, "Usart2 Tx Wait");
  BSP_OS_SemCreate(&Sem_USART2_RxWait,   0, "Usart2 Rx Wait");
  BSP_OS_SemCreate(&Sem_USART2_Lock,     1, "Usart2 Lock");

  /* ----------------- INIT USART STRUCT ---------------- */
  usart_init.USART_BaudRate            = 115200;
  usart_init.USART_WordLength          = USART_WordLength_8b;
  usart_init.USART_StopBits            = USART_StopBits_1;
  usart_init.USART_Parity              = USART_Parity_No ;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart_init.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

  usart_clk_init.USART_Clock           = USART_Clock_Disable;
  usart_clk_init.USART_CPOL            = USART_CPOL_Low;
  usart_clk_init.USART_CPHA            = USART_CPHA_2Edge;
  usart_clk_init.USART_LastBit         = USART_LastBit_Disable;


  BSP_PeriphEn(BSP_PERIPH_ID_USART2);
  BSP_PeriphEn(BSP_PERIPH_ID_GPIOA);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);                                                   /* ----------------- SETUP USART1 GPIO ---------------- */
  /* Configure GPIOA.2 as push-pull.                      */
  gpio_init.GPIO_Pin   = GPIO_Pin_2;
  gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &gpio_init);

  /* Configure GPIOA.3 as input floating.                 */
  gpio_init.GPIO_Pin   = GPIO_Pin_3;
  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &gpio_init);

  /* ------------------ SETUP USART1 -------------------- */
  BSP_IntVectSet(BSP_INT_ID_USART2, BSP_USART2_ISR);
  BSP_IntPrioSet(BSP_INT_ID_USART2, BSP_INT_ID_USART2_PRIO);
  BSP_IntDis(BSP_INT_ID_USART2);

  USART_Init(USART2, &usart_init);
  USART_ClockInit(USART2, &usart_clk_init);

  USART_ITConfig(USART2, USART_IT_TC, DISABLE);
  USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
  USART_ClearITPendingBit(USART2, USART_IT_TC);
  USART_ClearITPendingBit(USART2, USART_IT_RXNE);

  BSP_IntEn(BSP_INT_ID_USART2);
  USART_Cmd(USART2, ENABLE);
}



/*
********************************************************************
                           BSP_USART2_ISR()
********************************************************************
*/
volatile uint8_t g_packet_buf[COMM_PACKET_BUF_LEN];

void BSP_USART2_ISR(void)
{
  uint8_t c;
  static uint32_t buf_cnt = 0;
  static uint8_t capture_flag = 0;

  if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET) {

    c = USART_ReceiveData(USART2) & 0xFF;       /* Read one byte from the receive data register.      */

    if (c == '$') {
      capture_flag = 1;
      buf_cnt = 0;
      g_packet_buf[buf_cnt] = c;
      buf_cnt++;
    }
    else if (capture_flag == 1) {

      g_packet_buf[buf_cnt] = c;
      buf_cnt++;

      if (COMM_PACKET_BUF_LEN == buf_cnt) {

        if (c == '*')
          BSP_OS_SemPost(&Sem_USART2_RxWait);
        else
          capture_flag = 0;
      }

    }

    USART_ClearITPendingBit(USART2, USART_IT_RXNE);         /* Clear the USART2 receive interrupt.*/
  }

  if (USART_GetFlagStatus(USART2, USART_FLAG_TC) == SET) {
    USART_ITConfig(USART2, USART_IT_TC, DISABLE);
    USART_ClearITPendingBit(USART2, USART_IT_TC);           /* Clear the USART2 translate interrupt.                */
    BSP_OS_SemPost(&Sem_USART2_TxWait);                         /* Post to the semaphore                              */
  }
}



