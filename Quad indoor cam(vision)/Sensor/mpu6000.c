#include "bsp.h"

extern BSP_OS_MUTEX  Mutex_SPI1Lock;
BSP_OS_SEM Sem_MPU6000_DataReady;

/*
***********************************************
*          MPU6000_SingleWrite()
***********************************************
*/
void MPU6000_SingleWrite(uint8_t address,uint8_t value)
{
	BSP_OS_MutexWait(&Mutex_SPI1Lock,0);
	MPU6000_CS_LOW;
  BSP_SPI1_SendReceiveByte(address);
	BSP_SPI1_SendReceiveByte(value);
	MPU6000_CS_HIGH;
	BSP_OS_MutexPost(&Mutex_SPI1Lock);
}

/*
***********************************************
*          MPU6000_SingleRead()
***********************************************
*/
uint8_t MPU6000_SingleRead(uint8_t address)
{
	uint8_t value;
	BSP_OS_MutexWait(&Mutex_SPI1Lock,0);
	MPU6000_CS_LOW;
  BSP_SPI1_SendReceiveByte(0x80|address);
	value = BSP_SPI1_SendReceiveByte(0xff);
	MPU6000_CS_HIGH;
	BSP_OS_MutexPost(&Mutex_SPI1Lock);
	return value;
}

/*
***********************************************
*            MPU6000_BrustRead()
***********************************************
*/
void MPU6000_BrustRead(uint8_t address,uint8_t *p_buf,uint16_t length)
{
	uint16_t i;
	BSP_OS_MutexWait(&Mutex_SPI1Lock,0);
	MPU6000_CS_LOW;
  BSP_SPI1_SendReceiveByte(0x80|address);
	for(i = 0;i<length;i++)
	{
	  p_buf[i] = BSP_SPI1_SendReceiveByte(0xff);
	}
	MPU6000_CS_HIGH;
	BSP_OS_MutexPost(&Mutex_SPI1Lock);
}

/**************************************
* @brief : 初始化MPU6000
* @param : none
 ***************************************/
uint8_t MPU6000_Init(void)
{
	uint8_t status;
  GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStruct;
	
	BSP_OS_SemCreate(&Sem_MPU6000_DataReady,0,"Sem_MPU6000_DataReady");
	
	BSP_PeriphEn(BSP_PERIPH_ID_GPIOC);
	/* configure cs pin */
  GPIO_InitStructure.GPIO_Pin = MPU6000_CS_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MPU6000_CS_PORT, &GPIO_InitStructure);	
	MPU6000_CS_HIGH;
	
	/* configure interrupt pin */
  GPIO_InitStructure.GPIO_Pin = MPU6000_INT_PIN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MPU6000_INT_PORT, &GPIO_InitStructure);	
	
	BSP_IntVectSet(BSP_INT_ID_EXTI0, BSP_EXTI0_ISR_Handler);
  BSP_IntPrioSet(BSP_INT_ID_EXTI0,BSP_INT_ID_EXTI0_PRIO);
  BSP_IntDis(BSP_INT_ID_EXTI0);
	
	BSP_PeriphEn(BSP_PERIPH_ID_SYSCFG);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
	/* EXTI Config */
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);
	
  status = MPU6000_Check();
	
  MPU6000_SingleWrite(PWR_MGMT_1, 0x80);	   /* Reset must wait ~50ms */
	MPU6000_Delay(50);                         /* after reset it turns to sleep mode */
	MPU6000_SingleWrite(PWR_MGMT_1, 0x03);     /* wake up,PLL with Z axis gyroscope reference*/
	MPU6000_SingleWrite(USER_CTRL, 0x01);      /* Reset signal path */
	MPU6000_Delay(10);
	MPU6000_SingleWrite(USER_CTRL, 0x10);      /* Enable SPI mode */
	MPU6000_SingleWrite(SMPLRT_DIV, 0);     /* 1khz/(1+SMPLRT_DIV)*/
	MPU6000_SingleWrite(INT_PIN_CFG, 0x02);    /* Config INT Pin PP,active high, the INT pin emits a 50us long pulse */
	MPU6000_SingleWrite(INT_ENABLE,0x01);      /* Enable data ready interrupt */
  MPU6000_SingleWrite(CONFIG, 0x03);        /* low pass cut off freq :0x06(5Hz) (0x04 20Hz) (0x03 42Hz) (0x05 10Hz) (0x02 98Hz)*/
  MPU6000_SingleWrite(GYRO_CONFIG, 0x18);   /* Gyro range : 0x18(no selftest 2000deg/s) 0x08(500deg/s) 0x00(250deg/s) 0x10(1000deg/s)*/
  MPU6000_SingleWrite(ACCEL_CONFIG, 0x08);  /* no self test,acc range +-16g*/
// 	BSP_Ser_Printf("%x\r\n",MPU6000_SingleRead(PWR_MGMT_1));
// 	BSP_Ser_Printf("%x\r\n",MPU6000_SingleRead(SMPLRT_DIV));
// 	BSP_Ser_Printf("%x\r\n",MPU6000_SingleRead(USER_CTRL));
// 	BSP_Ser_Printf("%x\r\n",MPU6000_SingleRead(INT_PIN_CFG));
// 	BSP_Ser_Printf("%x\r\n",MPU6000_SingleRead(INT_ENABLE));
// 	BSP_Ser_Printf("%x\r\n",MPU6000_SingleRead(CONFIG));
// 	BSP_Ser_Printf("%x\r\n",MPU6000_SingleRead(GYRO_CONFIG));
// 	BSP_Ser_Printf("%x\r\n",MPU6000_SingleRead(ACCEL_CONFIG));
	return status;
}


/*******************************************
 *@name   : MPU6000_Check
 *@brief  : 检查MPU6000是否正常工作
 *@param  : none
 *@retval : none
 ********************************************
*/
uint8_t MPU6000_Check(void)
{
  uint8_t state;
  state = MPU6000_SingleRead(WHO_AM_I);	
  if( state == 0x68 )
    return DEF_OK;		   
  else	
    return DEF_FAIL;											  
}

/********************************************
 *@brief  : Get 6 Axis Data
 *@param	:  -MPU6000_TYPE*
 *@retval : 
 *********************************************/
uint8_t MPU6000_Read(MPU6000_TYPE* MPU6000_Value)
{
	uint8_t tmp[14];
	
	BSP_IntEn(BSP_INT_ID_EXTI0);

	if(BSP_OS_SemWait(&Sem_MPU6000_DataReady,50) == DEF_OK)
	{
		//LOGIC_DBG_PIN1_H;		
		MPU6000_BrustRead(ACCEL_XOUT_H,&tmp[0],14);
		//LOGIC_DBG_PIN1_L;		
		/* acc data */
		MPU6000_Value->acc_x   = ((tmp[0]<<8)+tmp[1]);
		
		MPU6000_Value->acc_y   = ((tmp[2]<<8)+tmp[3]);

		MPU6000_Value->acc_z   = -((tmp[4]<<8)+tmp[5]); 
		/* gyro data */
		MPU6000_Value->gyro_x  =  -((tmp[8]<<8)+tmp[9]);
		
		MPU6000_Value->gyro_y  =  -((tmp[10]<<8)+tmp[11]);

		MPU6000_Value->gyro_z  =  ((tmp[12]<<8)+tmp[13]); 
		
		MPU6000_Value->temperature = (tmp[6]<<8)+tmp[7];
		
		return DEF_OK;
	}
	else
	  return DEF_FAIL;
}

/********************************************
 *@brief  : Get 3 Axis Accelemeter data
 *@param	:  -MPU6000_TYPE*
 *@retval : 
 *********************************************/
uint8_t MPU6000_ReadAcc(MPU6000_TYPE* MPU6000_Value)
{
	uint8_t tmp[6];
	
	BSP_IntEn(BSP_INT_ID_EXTI0);
	
	BSP_OS_SemWait(&Sem_MPU6000_DataReady,0);
	/* acc data */
  MPU6000_BrustRead(ACCEL_XOUT_H,&tmp[0],6);

  MPU6000_Value->acc_y   = ((tmp[0]<<8)+tmp[1]);
	
  MPU6000_Value->acc_x   = -((tmp[2]<<8)+tmp[3]);

  MPU6000_Value->acc_z   = -((tmp[4]<<8)+tmp[5]);
	
  return DEF_OK; 	
}

/********************************************
 *@brief  : Get 3 Axis Gyroscope data
 *@param	:  -MPU6000_TYPE*
 *@retval : 
 *********************************************/
uint8_t MPU6000_ReadGyro(MPU6000_TYPE* MPU6000_Value)
{
	uint8_t tmp[6];
	
	BSP_IntEn(BSP_INT_ID_EXTI0);
	
	BSP_OS_SemWait(&Sem_MPU6000_DataReady,0);

	MPU6000_BrustRead(GYRO_XOUT_H,&tmp[0],6);

  MPU6000_Value->gyro_y   = -((tmp[0]<<8)+tmp[1]);
	
  MPU6000_Value->gyro_x   = ((tmp[2]<<8)+tmp[3]);

  MPU6000_Value->gyro_z   = ((tmp[4]<<8)+tmp[5]); 
  return DEF_OK;	
}

/*
**************************************************************
* Description : Print data read from Sensor
* Argument    : Point to MPU6000_TYPE Structure
**************************************************************
*/
void MPU6000_Printf(MPU6000_TYPE * ptResult)
{
	float ax,ay,az;
	float gx,gy,gz;
	ax = (ptResult->acc_x ) / ACC_SENSITIVITY;  
  ay = (ptResult->acc_y ) / ACC_SENSITIVITY; 	
  az = (ptResult->acc_z ) / ACC_SENSITIVITY; 
  gx = (ptResult->gyro_x) / GYRO_SENSITIVITY;	
  gy = (ptResult->gyro_y) / GYRO_SENSITIVITY;
	gz = (ptResult->gyro_z) / GYRO_SENSITIVITY;
   BSP_Ser_Printf("MPU6000:\tax: %5.2f,\tay: %5.2f,\taz: %5.2f,\tgx: %5.2f,\tgy: %5.2f,\tgz: %5.2f\r\n",
 	                ax,ay,az,gx,gy,gz);
//   BSP_Ser_Printf("%5.2f,%5.2f,%5.2f,\r\n", ax,ay,az);
//	  BSP_Ser_Printf("%5.2f,%5.2f,%5.2f,\r\n", gx,gy,gz);
	//BSP_Ser_Printf("%8d,%8d,%8d,\n\r",ptResult->acc_x,ptResult->acc_y,ptResult->acc_z);
}


void MPU6000_ClearInterrupt(void)
{
	MPU6000_SingleRead(INT_STATUS);
}


/*
**************************************************************
* Description : MPU6000 interrupt handler
* Argument    : 
**************************************************************
*/
void BSP_EXTI0_ISR_Handler(void)
{
	/* Check status */
	if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
		
		/* Disable the interrupt */
		BSP_IntDis(BSP_INT_ID_EXTI0);
		
		/* Clear bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		
		/* Post semaphore */
		BSP_OS_SemPost(&Sem_MPU6000_DataReady);
	}
}
