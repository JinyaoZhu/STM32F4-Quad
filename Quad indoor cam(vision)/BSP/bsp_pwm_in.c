#include "bsp.h"
#include "GlobalVariable.h"

BSP_OS_SEM PWM_IN_Update_Sem;

// static volatile uint16_t ch1_width = 0;
// static volatile uint16_t ch2_width = 0;
// static volatile uint16_t ch3_width = 0;
// static volatile uint16_t ch4_width = 0;
// static volatile uint16_t ch5_width = 0;
// static volatile uint16_t ch6_width = 0;
// static volatile uint16_t ch7_width = 0;
// static volatile uint16_t ch8_width = 0;

// /*
// *******************************************************************
//                    BSP_PWM_IN_InitTimer()
// *******************************************************************
// */
// void BSP_PWM_IN_InitTimer(void)
// {
//   GPIO_InitTypeDef GPIO_InitStructure;
//   TIM_ICInitTypeDef  TIM_ICInitStructure;
//   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

//   BSP_OS_SemCreate(&PWM_IN_Update_Sem, 0, "PWM_IN_Update_Sem");

//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

//   /* set timebase */
//   TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//   TIM_TimeBaseStructure.TIM_Period = 0x0000FFFFu;
//   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//   TIM_TimeBaseStructure.TIM_ClockDivision = 0x00;
//   TIM_TimeBaseStructure.TIM_Prescaler = 100 - 1; /* source 168M / 10 = 1.68MHz */
//   TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

//   /* Set timers pins */
//   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

//   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
//   GPIO_Init(GPIOA, &GPIO_InitStructure);

//   GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);

//   TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
//   TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//   TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//   TIM_ICInitStructure.TIM_ICFilter = 0x05;

//   TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
//   TIM_ICInit(TIM1, &TIM_ICInitStructure);

//   BSP_IntVectSet(BSP_INT_ID_TIM1_CC, TIM1_ISR_Handler);
//   BSP_IntPrioSet(BSP_INT_ID_TIM1_CC, BSP_INT_ID_TIM1_CC_PRIO);
//   BSP_IntEn(BSP_INT_ID_TIM1_CC);

//   TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);

//   TIM_Cmd(TIM1, ENABLE);
// }


// /*
// *******************************************************************
//                  TIM1_ISR_Handler()
// *******************************************************************
// */
// void TIM1_ISR_Handler(void)
// {
//   uint16_t width;
//   static uint16_t last_cnt = 0;
//   static uint16_t ch = 1;
//   uint32_t cnt;

//   /* Interrupt handler for PWMIN */

//   if (TIM_GetITStatus(TIM1, TIM_IT_CC3)) {

//     /* Clear pending bit */
//     TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);

//     cnt = TIM_GetCapture3(TIM1);

//     if (cnt < last_cnt)
//       width = 0xffff - last_cnt + cnt + 1;
//     else
//       width = cnt - last_cnt;

//     last_cnt = cnt;

//     if (width > 8000) {
//       ch = 1;
//       return;
//     }

//     switch (ch) {
//       case 1:
//         ch1_width = width;
//         break;
//       case 2:
//         ch2_width = width;
//         break;
//       case 3:
//         ch3_width = width;
//         break;
//       case 4:
//         ch4_width = width;
//         break;
//       case 5:
//         ch5_width = width;
//         break;
//       case 6:
//         ch6_width = width;
//         break;
//       case 7:
//         ch8_width = width;
// 			  BSP_OS_SemPost(&PWM_IN_Update_Sem);
//         break;
//       default : break;
//     }
//     ch++;
//   }
// }

// /*
// *******************************************************************
//                    BSP_PWM_IN_GetDuty()
// *******************************************************************
// */
// uint8_t BSP_PWM_IN_GetWidth(PWM_IN_DataType *p)
// {
//   uint8_t status ;

//   status = BSP_OS_SemWait(&PWM_IN_Update_Sem, 50);
// 	
//   p->ch1_width = ch1_width / 1680000.0f;
//   p->ch2_width = ch2_width / 1680000.0f;
//   p->ch3_width = ch3_width / 1680000.0f;
//   p->ch4_width = ch4_width / 1680000.0f;
//   p->ch5_width = ch5_width / 1680000.0f;
//   p->ch6_width = ch6_width / 1680000.0f;
//   p->ch7_width = ch7_width / 1680000.0f;
//   p->ch8_width = ch8_width / 1680000.0f;
// 	
//   return status;
// }
#include "bsp.h"
#include "GlobalVariable.h"

static __IO uint32_t __ch1_high_cnt = 0;
static __IO uint32_t __ch1_rising_cnt = 0;
static __IO uint32_t __ch1_falling_cnt = 0;
static __IO uint32_t __ch1_period_cnt = 0;
static __IO uint32_t CH1_PeriodCnt = 0;
static __IO uint32_t CH1_DutyCnt = 0;

static __IO uint32_t __ch2_high_cnt = 0;
static __IO uint32_t __ch2_rising_cnt = 0;
static __IO uint32_t __ch2_falling_cnt = 0;
static __IO uint32_t __ch2_period_cnt = 0;
static __IO uint32_t CH2_PeriodCnt = 0;
static __IO uint32_t CH2_DutyCnt = 0;

static __IO uint32_t __ch3_high_cnt = 0;
static __IO uint32_t __ch3_rising_cnt = 0;
static __IO uint32_t __ch3_falling_cnt = 0;
static __IO uint32_t __ch3_period_cnt = 0;
static __IO uint32_t CH3_PeriodCnt = 0;
static __IO uint32_t CH3_DutyCnt = 0;

static __IO uint32_t __ch4_high_cnt = 0;
static __IO uint32_t __ch4_rising_cnt = 0;
static __IO uint32_t __ch4_falling_cnt = 0;
static __IO uint32_t __ch4_period_cnt = 0;
static __IO uint32_t CH4_PeriodCnt = 0;
static __IO uint32_t CH4_DutyCnt = 0;

static __IO uint32_t __ch5_high_cnt = 0;
static __IO uint32_t __ch5_rising_cnt = 0;
static __IO uint32_t __ch5_falling_cnt = 0;
static __IO uint32_t __ch5_period_cnt = 0;
static __IO uint32_t CH5_PeriodCnt = 0;
static __IO uint32_t CH5_DutyCnt = 0;

static __IO uint32_t __ch6_high_cnt = 0;
static __IO uint32_t __ch6_rising_cnt = 0;
static __IO uint32_t __ch6_falling_cnt = 0;
static __IO uint32_t __ch6_period_cnt = 0;
static __IO uint32_t CH6_PeriodCnt = 0;
static __IO uint32_t CH6_DutyCnt = 0;

static __IO uint8_t ch1_upadate_flag = 0;
static __IO uint8_t ch2_upadate_flag = 0;
static __IO uint8_t ch3_upadate_flag = 0;
static __IO uint8_t ch4_upadate_flag = 0;
static __IO uint8_t ch5_upadate_flag = 0;
static __IO uint8_t ch6_upadate_flag = 0;


BSP_OS_SEM PWM_IN_Update_Sem;

/*
*******************************************************************
*                  BSP_PWM_IN_InitTimer()
*******************************************************************
*/
void BSP_PWM_IN_InitTimer(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	BSP_OS_SemCreate(&PWM_IN_Update_Sem,0,"PWM_IN_Update_Sem");
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

  /* set timebase */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 0x0000FFFFu;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x00;
  TIM_TimeBaseStructure.TIM_Prescaler = 100 - 1; /* source 168M / 10 = 1.68MHz */
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = 0x0000FFFFu;
  TIM_TimeBaseStructure.TIM_Prescaler = 50 - 1;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);/* 1.68MHz */

  TIM_TimeBaseStructure.TIM_Period = 0x0000FFFFu;
  TIM_TimeBaseStructure.TIM_Prescaler = 100 - 1;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);/* 1.68MHz */

  /* Set timers pins */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_15;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);

  TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x05;

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInit(TIM1, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
  TIM_ICInit(TIM8, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  BSP_IntVectSet(BSP_INT_ID_TIM1_CC, TIM1_ISR_Handler);
  BSP_IntPrioSet(BSP_INT_ID_TIM1_CC, BSP_INT_ID_TIM1_CC_PRIO);
  BSP_IntEn(BSP_INT_ID_TIM1_CC);

  BSP_IntVectSet(BSP_INT_ID_TIM2, TIM2_ISR_Handler);
  BSP_IntPrioSet(BSP_INT_ID_TIM2, BSP_INT_ID_TIM2_PRIO);
  BSP_IntEn(BSP_INT_ID_TIM2);

  BSP_IntVectSet(BSP_INT_ID_TIM8_CC, TIM8_ISR_Handler);
  BSP_IntPrioSet(BSP_INT_ID_TIM8_CC, BSP_INT_ID_TIM8_CC_PRIO);
  BSP_IntEn(BSP_INT_ID_TIM8_CC);

  TIM_ITConfig(TIM1, TIM_IT_CC3, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC2, ENABLE);
  TIM_ITConfig(TIM8, TIM_IT_CC3, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

  TIM_Cmd(TIM1, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM8, ENABLE);
}


/*
*******************************************************************
*                TIM1_ISR_Handler()
*******************************************************************
*/
void TIM1_ISR_Handler(void)
{
  static uint8_t count_start_flag = 0;
  uint32_t ccr3;

  /* Interrupt handler for PWMIN */

  if (TIM_GetITStatus(TIM1, TIM_IT_CC3)) {

    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);

    ccr3 = TIM_GetCapture3(TIM1);

    /* Read data */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_10) == SET) {

      if (__ch1_rising_cnt > ccr3)
        __ch1_period_cnt = 0xffffu - __ch1_rising_cnt + ccr3;
      else
        __ch1_period_cnt = ccr3 - __ch1_rising_cnt;

      if (__ch1_period_cnt > __ch1_high_cnt) {
        CH1_PeriodCnt = __ch1_period_cnt;
        CH1_DutyCnt = __ch1_high_cnt;
        ch1_upadate_flag = 1;
      }
      __ch1_rising_cnt = ccr3;

      count_start_flag = 1;
    }
    else {

      if (count_start_flag == 1) {

        count_start_flag = 0;

        __ch1_falling_cnt = ccr3;

        if (__ch1_rising_cnt > __ch1_falling_cnt)
          __ch1_high_cnt = 0xffffu - __ch1_rising_cnt + __ch1_falling_cnt;
        else
          __ch1_high_cnt = __ch1_falling_cnt - __ch1_rising_cnt;

      }

    }
  }
}

/*
*******************************************************************
*                TIM8_ISR_Handler()
*******************************************************************
*/
void TIM8_ISR_Handler(void)
{
  static uint8_t count_start_flag1 = 0;
  static uint8_t count_start_flag2 = 0;
  static uint8_t count_start_flag3 = 0;
  uint32_t ccr1, ccr2, ccr3;

  /* Channel 1 */
  if (TIM_GetITStatus(TIM8, TIM_IT_CC1)) {

    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);

    ccr1 = TIM_GetCapture1(TIM8);

    /* Read data */
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6) == SET) {

      if (__ch2_rising_cnt > ccr1)
        __ch2_period_cnt = 0xffffu - __ch2_rising_cnt + ccr1;
      else
        __ch2_period_cnt = ccr1 - __ch2_rising_cnt;

      if (__ch2_period_cnt > __ch2_high_cnt) {
        CH2_PeriodCnt = __ch2_period_cnt;
        CH2_DutyCnt = __ch2_high_cnt;
        ch2_upadate_flag = 1;
      }

      __ch2_rising_cnt = ccr1;

      count_start_flag1 = 1;
    }
    else {

      if (count_start_flag1 == 1) {

        count_start_flag1 = 0;

        __ch2_falling_cnt = ccr1;

        if (__ch2_rising_cnt > __ch2_falling_cnt)
          __ch2_high_cnt = 0xffffu - __ch2_rising_cnt + __ch2_falling_cnt;
        else
          __ch2_high_cnt = __ch2_falling_cnt - __ch2_rising_cnt;

      }
    }
  }


  /* Channel 2 */
  if (TIM_GetITStatus(TIM8, TIM_IT_CC2)) {

    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);

    ccr2 = TIM_GetCapture2(TIM8);

    /* Read data */
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == SET) {

      if (__ch3_rising_cnt > ccr2)
        __ch3_period_cnt = 0xffffu - __ch3_rising_cnt + ccr2;
      else
        __ch3_period_cnt = ccr2 - __ch3_rising_cnt;

      if (__ch3_period_cnt > __ch3_high_cnt) {
        CH3_PeriodCnt = __ch3_period_cnt;
        CH3_DutyCnt = __ch3_high_cnt;
        ch3_upadate_flag = 1;
      }

      __ch3_rising_cnt = ccr2;

      count_start_flag2 = 1;
    }
    else {

      if (count_start_flag2 == 1) {

        count_start_flag2 = 0;

        __ch3_falling_cnt = ccr2;

        if (__ch3_rising_cnt > __ch3_falling_cnt)
          __ch3_high_cnt = 0xffffu - __ch3_rising_cnt + __ch3_falling_cnt;
        else
          __ch3_high_cnt = __ch3_falling_cnt - __ch3_rising_cnt;

      }
    }
  }

  /* Channel 3 */
  if (TIM_GetITStatus(TIM8, TIM_IT_CC3)) {
    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);

    ccr3 = TIM_GetCapture3(TIM8);

    /* Read data */
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8) == SET) {

      if (__ch4_rising_cnt > ccr3)
        __ch4_period_cnt = 0xffffu - __ch4_rising_cnt + ccr3;
      else
        __ch4_period_cnt = ccr3 - __ch4_rising_cnt;

      if (__ch4_period_cnt > __ch4_high_cnt) {
        CH4_PeriodCnt = __ch4_period_cnt;
        CH4_DutyCnt = __ch4_high_cnt;
        ch4_upadate_flag = 1;
      }

      __ch4_rising_cnt = ccr3;

      count_start_flag3 = 1;
    }
    else {

      if (count_start_flag3 == 1) {

        count_start_flag3 = 0;

        __ch4_falling_cnt = ccr3;

        if (__ch4_rising_cnt > __ch4_falling_cnt)
          __ch4_high_cnt = 0xffffu - __ch4_rising_cnt + __ch4_falling_cnt;
        else
          __ch4_high_cnt = __ch4_falling_cnt - __ch4_rising_cnt;

      }
    }
  }

}

/*
*******************************************************************
*                TIM2_ISR_Handler()
*******************************************************************
*/
void TIM2_ISR_Handler(void)
{
  static uint8_t count_start_flag1 = 0;
  static uint8_t count_start_flag2 = 0;
  uint32_t ccr1, ccr2;
  
  if (TIM_GetITStatus(TIM2, TIM_IT_CC1)) {
    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

    ccr1 = TIM_GetCapture1(TIM2);

    /* Read data */
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == SET) {

      if (__ch5_rising_cnt > ccr1)
        __ch5_period_cnt = 0xffffu - __ch5_rising_cnt + ccr1;
      else
        __ch5_period_cnt = ccr1 - __ch5_rising_cnt;

      if (__ch5_period_cnt > __ch5_high_cnt) {
        CH5_PeriodCnt = __ch5_period_cnt;
        CH5_DutyCnt = __ch5_high_cnt;
        ch5_upadate_flag = 1;
      }

      __ch5_rising_cnt = ccr1;

      count_start_flag1 = 1;
    }
    else {

      if (count_start_flag1 == 1) {

        count_start_flag1 = 0;

        __ch5_falling_cnt = ccr1;

        if (__ch5_rising_cnt > __ch5_falling_cnt)
          __ch5_high_cnt = 0xffffu - __ch5_rising_cnt + __ch5_falling_cnt;
        else
          __ch5_high_cnt = __ch5_falling_cnt - __ch5_rising_cnt;

      }
    }
  }

  if (TIM_GetITStatus(TIM2, TIM_IT_CC2)) {
    /* Clear pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);

    ccr2 = TIM_GetCapture2(TIM2);

    /* Read data */
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == SET) {

      if (__ch6_rising_cnt > ccr2)
        __ch6_period_cnt = 0xffffu - __ch6_rising_cnt + ccr2;
      else
        __ch6_period_cnt = ccr2 - __ch6_rising_cnt;

      if (__ch6_period_cnt > __ch6_high_cnt) {
        CH6_PeriodCnt = __ch6_period_cnt;
        CH6_DutyCnt = __ch6_high_cnt;
        ch6_upadate_flag = 1;
				BSP_OS_SemPost(&PWM_IN_Update_Sem);
      }

      __ch6_rising_cnt = ccr2;

      count_start_flag2 = 1;
    }
    else {

      if (count_start_flag2 == 1) {

        count_start_flag2 = 0;

        __ch6_falling_cnt = ccr2;

        if (__ch6_rising_cnt > __ch6_falling_cnt)
          __ch6_high_cnt = 0xffffu - __ch6_rising_cnt + __ch6_falling_cnt;
        else
          __ch6_high_cnt = __ch6_falling_cnt - __ch6_rising_cnt;

      }
    }
  }
}



/*
*******************************************************************
*                  BSP_PWM_IN_GetWidth()
*******************************************************************
*/
uint8_t BSP_PWM_IN_GetWidth(PWM_IN_DataType *p)
{
	uint8_t status ;
	
	status = BSP_OS_SemWait(&PWM_IN_Update_Sem,50);
	
	/* Channel 1 */
  if (ch1_upadate_flag == 1) {
    p->ch1_width = (float)CH1_DutyCnt*(1.0f/1680000.0f);
    ch1_upadate_flag = 0;
  }
  else
    p->ch1_width = 0;

	/* Channel 2 */
  if (ch2_upadate_flag == 1) {
    p->ch2_width = (float)CH2_DutyCnt*(1.0f/1680000.0f);
    ch2_upadate_flag = 0;
  }
  else
    p->ch2_width = 0;

	/* Channel 3 */
  if (ch3_upadate_flag == 1) {
    p->ch3_width = (float)CH3_DutyCnt*(1.0f/1680000.0f);
    ch3_upadate_flag = 0;
  }
  else
    p->ch3_width = 0;

	/* Channel 4 */
  if (ch4_upadate_flag == 1) {
    p->ch4_width = (float)CH4_DutyCnt*(1.0f/1680000.0f);
    ch4_upadate_flag = 0;
  }
  else
    p->ch4_width = 0;

	/* Channel 5 */
  if (ch5_upadate_flag == 1) {
    p->ch5_width = (float)CH5_DutyCnt*(1.0f/1680000.0f);
    ch5_upadate_flag = 0;
  }
  else
    p->ch5_width = 0;

	/* Channel 6 */
  if (ch6_upadate_flag == 1) {
    p->ch6_width = (float)CH6_DutyCnt*(1.0f/1680000.0f);
    ch6_upadate_flag = 0;
  }
  else
    p->ch6_width = 0;
	
	 return status;
}
