#include "bsp.h"


void BSP_PWM2_Init(uint32_t pwm_default)
{
  BSP_PWM2_GPIOConfig();
  BSP_PWM2_ModeConfig(pwm_default);
  BSP_Ser_Printf("PWM2 Initialize succeeded... \r\n");
}


void BSP_PWM2_GPIOConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  BSP_PeriphEn(BSP_PERIPH_ID_TIM4);
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOB);	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
  GPIO_Init(GPIOB,&GPIO_InitStructure);
}

/***********************************************************************
 *@brief  : 配置TIM3输出的PWM信号的模式，如周期，极性，占空比
 *@param  : none
 *@retval : none
 *@note   :
						*---------------------------------------------------------------------------+
						| TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:  |
						| TIM3CLK = 84MHz, Prescale = 0, TIM3 counter clock = 84MHz                 |
						| TIM3 ARR Register = 999 => TIM3 Frequency = TIM3 counter clock/(ARR+1)    |
						| TIM3(PWM) Frequency = 84kHZ																						    |
						| TIM3 Channel 1 duty cycle = (TIM3_CCR1/TIM3_ARR)*100                      |
						| TIM3 Channel 2 duty cycle = (TIM3_CCR2/TIM3_ARR)*100                      | 
						+---------------------------------------------------------------------------*
 ************************************************************************/
void BSP_PWM2_ModeConfig(uint32_t pwm_default)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
  /*ARR值*/
  TIM_TimeBaseStructure.TIM_Period    = TIM4_ARR;
  TIM_TimeBaseStructure.TIM_Prescaler = TIM4_PRESCALE;

  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM_OCInitStructure.TIM_Pulse = pwm_default;

  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4,ENABLE);

  TIM_Cmd(TIM4,ENABLE);
}


void BSP_PWM2_Set(uint32_t c)
{
	TIM_SetCompare4(TIM4 ,c);
}
