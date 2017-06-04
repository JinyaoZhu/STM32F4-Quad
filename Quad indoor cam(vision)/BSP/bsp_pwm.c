#include "bsp.h"

/****************************************************
 *@brief  : TIM3四个通道的pwm信号初始化
 *@param  : none
 *@retval : none
 *****************************************************/
void BSP_PWM_Init(uint32_t pwm_default)
{
  BSP_PWM_GPIOConfig();
  BSP_PWM_ModeConfig(pwm_default);
  BSP_Ser_Printf("PWM Initialize succeeded... \r\n");
}

/**************************************************
 *@brief  : 配置输出PWM的GPIO Pin(PA.06,PA.07  &  PB.00,PB.01)
 *@param  : none
 *@retval : none
 ***************************************************/
void BSP_PWM_GPIOConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  BSP_PeriphEn(BSP_PERIPH_ID_TIM3);
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOB);	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
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
void BSP_PWM_ModeConfig(uint32_t pwm_default)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
  /*ARR值*/
  TIM_TimeBaseStructure.TIM_Period    = TIM3_ARR;
  TIM_TimeBaseStructure.TIM_Prescaler = TIM3_PRESCALE;


  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM_OCInitStructure.TIM_Pulse = pwm_default;

  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
  TIM_OC1Init(TIM3,&TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
  TIM_OC2Init(TIM3,&TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM3,ENABLE);

  TIM_Cmd(TIM3,ENABLE);
}

/************************************************************************
 *@brief  : 设置通道1,2,3,4的占空比 = abs(输入)/((PWM_FREQUENCY/72)-1)
 *@param  : -ch1
 * 				  -ch2
            -ch3
            -ch4
 *@retval : none
 ************************************************************************/
void BSP_PWM_Set(uint32_t ch1 , uint32_t ch2 ,uint32_t ch3 , uint32_t ch4)
{
  if(ch1 > TIM3_PWM_HIGHEST)
    ch1 = TIM3_PWM_HIGHEST;
	else if(ch1 < TIM3_PWM_LOWEST)
		ch1 = TIM3_PWM_LOWEST;
	
  if(ch2 > TIM3_PWM_HIGHEST)
    ch2 = TIM3_PWM_HIGHEST;
	else if(ch2 < TIM3_PWM_LOWEST)
		ch2 = TIM3_PWM_LOWEST;
	
	if(ch3 > TIM3_PWM_HIGHEST)
    ch3 = TIM3_PWM_HIGHEST;
	else if(ch3 < TIM3_PWM_LOWEST)
		ch3 = TIM3_PWM_LOWEST;
	
	if(ch4 > TIM3_PWM_HIGHEST)
    ch4 = TIM3_PWM_HIGHEST;
	else if(ch4 < TIM3_PWM_LOWEST)
		ch4 = TIM3_PWM_LOWEST;
	
  TIM_SetCompare1(TIM3 ,ch1);
  TIM_SetCompare2(TIM3 ,ch2);
	TIM_SetCompare3(TIM3 ,ch3);
	TIM_SetCompare4(TIM3 ,ch4);
}
