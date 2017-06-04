#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__
#include "bsp.h"

#define TIM3_PWM_LOWEST     (21000-1)
#define TIM3_PWM_HIGHEST    (42000-1)
#define TIM3_PWM_WIDTH      (TIM3_PWM_HIGHEST-TIM3_PWM_LOWEST)

#define TIM3_PWM_FREQUENCY  (400)            
#define TIM3_PRESCALE       (4-1)             

#define TIM3_FREQUENCY (84000000/(TIM3_PRESCALE+1))
#define TIM3_ARR       ((TIM3_FREQUENCY / TIM3_PWM_FREQUENCY) - 1) //(ARR = 52500 - 1)


/* PWM Frequency =  TIM3_FREQUENCY / (TIM3_ARR+1) */

/*----------------Private-------------------------*/
void BSP_PWM_GPIOConfig(void);
void BSP_PWM_ModeConfig(uint32_t pwm_default);
/*----------------Public--------------------------*/
void BSP_PWM_Init(uint32_t pwm_default);
void BSP_PWM_Set(uint32_t ch1 , uint32_t ch2 ,uint32_t ch3 , uint32_t ch4);

#endif /*__BSP_PWM_H__*/
