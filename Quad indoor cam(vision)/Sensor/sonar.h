#ifndef __SONAR_H__
#define __SONAR_H__


#define TRIG_PIN     GPIO_Pin_12  /* PC12 */
#define TRIG_PORT    GPIOC
#define ECHO_PIN     GPIO_Pin_2  /* PD02 */
#define ECHO_PORT    GPIOD

#define SONAR_IRQ_LINE      EXTI_Line2
#define SONAR_IRQ_SRC_PORT  EXTI_PortSourceGPIOD
#define SONAR_IRQ_SRC_PIN   GPIO_PinSource2


void Sonar_delay_10us(void);

void Sonar_Init(void);

uint8_t Sonar_GetDistance(float *distance);

void BSP_EXTI2_ISR_Handler(void);


#endif /* __SONAR_H__ */
