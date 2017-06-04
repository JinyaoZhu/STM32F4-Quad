#include "bsp.h"
#include "hmc5883l.h"
#include "math.h"

BSP_OS_SEM    Sem_HMC5883L_DataReady;

#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f) //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f) //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f) //!< Z axis level when bias current is applied.

#define HMC5883L_Delay()   BSP_OS_TimeDlyMs(10)

float MAG_GAIN_X;
float MAG_GAIN_Y;
float MAG_GAIN_Z;

/*
*********************************************************
* Description: Write a byte
* Argument   : none
* Return     : none
*********************************************************
*/
void HMC5883L_WriteByte(uint8_t reg, uint8_t value)
{
  BSP_I2C3_SingleWrite(reg,value,HMC5883L_Addr);
}

/*
*********************************************************
* Description: Read a byte
* Argument   : none
* Return     : none
*********************************************************
*/
uint8_t  HMC5883L_ReadByte(uint8_t reg)
{
  return BSP_I2C3_SingleRead(reg,HMC5883L_Addr);
}

/*
*********************************************************
* Description: Brust Read
* Argument   : none
* Return     : none
*********************************************************
*/
uint8_t HMC5883L_BrustRead(uint8_t reg, uint8_t* p_buf, uint16_t length)
{
  return BSP_I2C3_MultRead(reg,HMC5883L_Addr,p_buf,length);
}

/*
*********************************************************
* Description: Initialize HMC5883L Sensor
* Argument   : none
* Return     : none
*********************************************************
*/
uint8_t  HMC5883L_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	uint8_t status;
	HMC5883L_TYPE p_hmc5883l;
	int32_t xyz_total[3] = {0};
	uint32_t i = 0;

  BSP_OS_SemCreate(&Sem_HMC5883L_DataReady,1,"Sem_HMC5883L_DataReady");

  BSP_PeriphEn(BSP_PERIPH_ID_GPIOC);
	/* configure interrupt pin */
  GPIO_InitStructure.GPIO_Pin = HMC5883L_DRDY_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(HMC5883L_DRDY_PORT, &GPIO_InitStructure);	
	
	BSP_IntVectSet(BSP_INT_ID_EXTI1,BSP_EXTI1_ISR_Handler);
  BSP_IntPrioSet(BSP_INT_ID_EXTI1,BSP_INT_ID_EXTI1_PRIO);
	BSP_IntDis(BSP_INT_ID_EXTI1);

	BSP_PeriphEn(BSP_PERIPH_ID_SYSCFG);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);
	/* EXTI Config */
	EXTI_InitStruct.EXTI_Line = EXTI_Line1;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);
  
  status = HMC5883L_ConnectCheck();

	HMC5883L_WriteByte(HMC5883L_REGA, 0x79); /* 75Hz,pos bias */
	HMC5883L_WriteByte(HMC5883L_REGB, 0x60); /* set gain 660 counts/gauss */
  HMC5883L_WriteByte(HMC5883L_MODE, 0x01); /* single measurement mode */
  HMC5883L_Delay();
	HMC5883L_Read(&p_hmc5883l); /* get one sample and discard it */

	for( i =0; i< 100; i++)
	{
    HMC5883L_WriteByte(HMC5883L_MODE, 0x01);
		HMC5883L_Read(&p_hmc5883l);
		xyz_total[0] += p_hmc5883l.hx;
		xyz_total[1] += p_hmc5883l.hy;
		xyz_total[2] += p_hmc5883l.hz;
  }
	HMC5883L_WriteByte(HMC5883L_REGA, 0x7a); /* 75Hz,neg bias */
	for( i =0; i< 100; i++)
	{
    HMC5883L_WriteByte(HMC5883L_MODE, 0x01);
		HMC5883L_Read(&p_hmc5883l);
		xyz_total[0] -= p_hmc5883l.hx;
		xyz_total[1] -= p_hmc5883l.hy;
		xyz_total[2] -= p_hmc5883l.hz;
  }
	
	MAG_GAIN_X = fabs(660.0f*HMC58X3_X_SELF_TEST_GAUSS*2.0f*100.0f/(float)xyz_total[0]);
	MAG_GAIN_Y = fabs(660.0f*HMC58X3_Y_SELF_TEST_GAUSS*2.0f*100.0f/(float)xyz_total[1]);
	MAG_GAIN_Z = fabs(660.0f*HMC58X3_Z_SELF_TEST_GAUSS*2.0f*100.0f/(float)xyz_total[2]);

  HMC5883L_WriteByte(HMC5883L_REGA, 0x78); /* 75Hz */
  HMC5883L_WriteByte(HMC5883L_REGB, 0x20); /* set gain 1090 counts/gauss */
  HMC5883L_WriteByte(HMC5883L_MODE, 0x00); /* continuous measurement mode */
	
	if((MAG_GAIN_X < 1.5f) && (MAG_GAIN_Y < 1.5f) && (MAG_GAIN_Z < 1.5f)&& \
		 (MAG_GAIN_X > 0.5f) && (MAG_GAIN_Y > 0.5f) && (MAG_GAIN_Z > 0.5f))
	  BSP_Ser_Printf("MAG_GAIN_X:%4.3f,MAG_GAIN_Y:%4.3f,MAG_GAIN_Z:%4.3f\r\n",MAG_GAIN_X,MAG_GAIN_Y,MAG_GAIN_Z);
	else
	{
		BSP_Ser_Printf("HMC5883L error!\r\n");
		for(;;);
	}
	
	return status;
}

/*
**********************************************************
* Description : Read data from sensor
* Argument    : point to HMC5883L_TYPE
* Return      : Data read from sensor
* Note        : none
*********************************************************
*/
uint8_t HMC5883L_Read(HMC5883L_TYPE * pResult)
{
  uint8_t tmp[6];

	/* Enable DRDY interrupt */
	BSP_IntEn(BSP_INT_ID_EXTI1);
	
  if(BSP_OS_SemWait(&Sem_HMC5883L_DataReady,100) == DEF_OK)
	{
		/* Read raw data */
		if(HMC5883L_BrustRead(HMC5883L_HX_H, &tmp[0], 6) == DEF_FAIL)
			BSP_Ser_Printf("<HMC5883L_Read>Error! \r\n");
		
		pResult->hx   =  (int16_t)((tmp[0] << 8) | tmp[1]);
		pResult->hy   =  (int16_t)((tmp[2] << 8) | tmp[3]);
		pResult->hz   =  (int16_t)((tmp[4] << 8) | tmp[5]);
		
		return DEF_OK;
	}
	else
		return DEF_FAIL;
}

/*
**********************************************************
* Description : Check if data ready
* Argument    : 
* Return      : status
*********************************************************
*/
uint8_t HMC5883L_IsDataReady(void)
{
	uint8_t status = 0;
	
  status = HMC5883L_ReadByte(HMC5883L_STATE);
	
	if((status & 0x01) != 0)
	{
    return DEF_OK;
  }
	else
	{
    return DEF_FAIL;
  }
}

/*
**********************************************************
* Description : Check connection
* Argument    : 
* Return      : status
*********************************************************
*/
uint8_t HMC5883L_ConnectCheck(void)
{
  uint8_t status = DEF_FAIL;
	status = HMC5883L_ReadByte(HMC5883L_IRA);
	if(status == 0x48)
		return DEF_OK;
	else
		return DEF_FAIL;
}

/*
**************************************************************
* Description : HMC5883L interrupt handler
* Argument    : 
**************************************************************
*/
void BSP_EXTI1_ISR_Handler(void)
{
	/* Check status */
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		
		/* Disable DRDY interrupt */
    BSP_IntDis(BSP_INT_ID_EXTI1);
		
		/* Clear bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
		
		/* Post semaphore */
		BSP_OS_SemPost(&Sem_HMC5883L_DataReady);
	}
}
