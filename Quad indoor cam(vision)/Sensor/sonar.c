#include "bsp.h"
#include "GlobalVariable.h"

uint32_t echo_cnt_start = 0;
uint32_t echo_cnt_end = 0;
uint32_t echo_duration = 0;

BSP_OS_SEM  Sem_Sonar_Wait;

/*
******************************************************
*                 Sonar_Init()
* Description : 
*
******************************************************
*/
void Sonar_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	EXTI_InitTypeDef EXTI_InitStructure;
  
	BSP_OS_SemCreate((BSP_OS_SEM*)&Sem_Sonar_Wait,(BSP_OS_SEM_VAL) 0,(CPU_CHAR*) "Sonar Wait"); 
	   
	/* GPIO Configuration */
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOC);
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOD);
	
  GPIO_InitStructure.GPIO_Pin = TRIG_PIN;					 
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
  GPIO_Init(TRIG_PORT, &GPIO_InitStructure);	              

  GPIO_InitStructure.GPIO_Pin = ECHO_PIN;				     
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ECHO_PORT,&GPIO_InitStructure);		
	
	GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
  	
	/* EXIT Configuration */
	BSP_IntVectSet(BSP_INT_ID_EXTI2,BSP_EXTI2_ISR_Handler);
	BSP_IntPrioSet(BSP_INT_ID_EXTI2,BSP_INT_SONAR_IQR_PRIO);
  BSP_IntEn(BSP_INT_ID_EXTI2);
	
	EXTI_ClearITPendingBit(SONAR_IRQ_LINE);
	BSP_PeriphEn(BSP_PERIPH_ID_SYSCFG);
	SYSCFG_EXTILineConfig(SONAR_IRQ_SRC_PORT,SONAR_IRQ_SRC_PIN);
	EXTI_InitStructure.EXTI_Line = SONAR_IRQ_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}

/*
******************************************************
*                Sonar_GetDistance(void)
* Description : 
*
******************************************************
*/
uint8_t Sonar_GetDistance(float *distance)
{
	float pulse_width;
	
  GPIO_SetBits(TRIG_PORT,TRIG_PIN); 		
  Sonar_delay_10us();
  GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
	
	if(BSP_OS_SemWait(&Sem_Sonar_Wait,20) == DEF_OK)
	{
		if(echo_cnt_end > echo_cnt_start)
			echo_duration = echo_cnt_end - echo_cnt_start;
		else
			echo_duration = 4294967296u - echo_cnt_start + echo_cnt_end;
		
		pulse_width = echo_duration * (1.0f/168000000); /* s */
		
		*distance = pulse_width * 340.0f/2; /* M */
	  return DEF_OK;
	}
	else
	{
		return DEF_FAIL;
  }
}


void Sonar_delay_10us(void)
{
  uint32_t i = 400;
	while(i--);
}

/*
*******************************************************************
*                 BSP_EXTI2_ISR_Handler()
*******************************************************************
*/
void BSP_EXTI2_ISR_Handler(void)
{
	CPU_TS32 cnt = CPU_TS_Get32();
	if (EXTI_GetITStatus(SONAR_IRQ_LINE) != RESET) {
		
		if(GPIO_ReadInputDataBit(ECHO_PORT,ECHO_PIN) == SET)
		{    	
			echo_cnt_start = cnt;
		}
		else 
		{
			echo_cnt_end =  cnt;
			BSP_OS_SemPost(&Sem_Sonar_Wait);
		}
		
		EXTI_ClearITPendingBit(SONAR_IRQ_LINE);
  }
}
