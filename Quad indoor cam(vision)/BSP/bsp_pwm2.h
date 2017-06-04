#ifndef __BSP_PWM2_H__
#define __BSP_PWM2_H__
#include "bsp.h"

#define TIM4_PWM_LOWEST      TIM3_PWM_LOWEST
#define TIM4_PWM_HIGHEST     TIM3_PWM_HIGHEST
#define TIM4_PWM_WIDTH       TIM3_PWM_WIDTH
   
#define TIM4_PRESCALE   TIM3_PRESCALE
#define TIM4_FREQUENCY  TIM3_FREQUENCY
#define TIM4_ARR        TIM3_ARR



/*----------------Private-------------------------*/
void BSP_PWM2_GPIOConfig(void);
void BSP_PWM2_ModeConfig(uint32_t pwm_default);
/*----------------Public--------------------------*/
void BSP_PWM2_Init(uint32_t pwm_default);
void BSP_PWM2_Set(uint32_t c);

#endif /*__BSP_PWM_H__*/
