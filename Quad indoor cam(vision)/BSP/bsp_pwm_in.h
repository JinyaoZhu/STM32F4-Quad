#ifndef __BSP_PWM_IN__
#define __BSP_PWM_IN__

typedef struct
{
  float ch1_width;
	float ch2_width;
	float ch3_width;
	float ch4_width;
	float ch5_width;
	float ch6_width;
	float ch7_width;
	float ch8_width;
}PWM_IN_DataType;


void BSP_PWM_IN_InitTimer(void);

void TIM1_ISR_Handler(void);

void TIM2_ISR_Handler(void);

void TIM8_ISR_Handler(void);

uint8_t BSP_PWM_IN_GetWidth(PWM_IN_DataType *p);

#endif /* __BSP_PWM_IN__ */
