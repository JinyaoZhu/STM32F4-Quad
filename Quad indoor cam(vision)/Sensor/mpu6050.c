#include "bsp.h"
#include "math.h"

#define PI     3.14159f


/**************************************
* @brief : 初始化MPU6050
* @param : none
 ***************************************/
void MPU6050_Init(void)
{
  BSP_I2C_SingleWrite(PWR_MGMT_1, 0x80,MPU6050_SLAVE_ADDR);	   /* Reset must wait ~50ms */
	MPU6050_Delay(100);
	BSP_I2C_SingleWrite(USER_CTRL, 0x00,MPU6050_SLAVE_ADDR); /* Disable I2C Master mode */
	BSP_I2C_SingleWrite(INT_PIN_CFG, 0x02,MPU6050_SLAVE_ADDR);
	BSP_I2C_SingleWrite(PWR_MGMT_1, 0x00,MPU6050_SLAVE_ADDR);
  BSP_I2C_SingleWrite(SMPLRT_DIV, 0x00,MPU6050_SLAVE_ADDR);    /*采样率，典型值：0x07(125Hz),输出采样率 = 1khz/(1+SMPLRT_DIV)*/
  BSP_I2C_SingleWrite(MPU6050_CONFIG, 0x04,MPU6050_SLAVE_ADDR);/*低通滤波频率，典型值：0x06(5Hz) (0x04 20Hz) (0x03 42Hz) (0x05 10Hz) (0x02 98Hz)*/
  BSP_I2C_SingleWrite(GYRO_CONFIG, 0x10,MPU6050_SLAVE_ADDR);   /*陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s) 0x08(500deg/s) 0x00(250deg/s) 0x10(1000deg/s)*/
  BSP_I2C_SingleWrite(ACCEL_CONFIG, 0x01,MPU6050_SLAVE_ADDR);  /*加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)*/
}

/*******************************************
 *@name   : MPU6050_Check
 *@brief  : 检查MPU6050是否正常工作
 *@param  : none
 *@retval : none
 ********************************************/
u8 MPU6050_Check(void)
{
  u8 state;
  state = BSP_I2C_SingleRead(WHO_AM_I,MPU6050_SLAVE_ADDR);			   
  if( state == 0X68 )
    return 1;		   
  else	
    return 0;											  
}

/********************************************
 *@brief  : Get 6 Axis Data
 *@param	:  -MPU6050_TYPE*
 *@retval : 
 *********************************************/
uint8_t MPU6050_Read(MPU6050_TYPE* MPU6050_Value)
{
	uint8_t tmp[14];
	/* acc data */
	
	if(BSP_I2C_MultRead(ACCEL_XOUT_H,MPU6050_SLAVE_ADDR,&tmp[0],14) == DEF_FAIL)
	{
    BSP_Ser_Printf("\n\r MPU6050 IIC ERROR! \n\r");
		return DEF_FAIL;
  }

  MPU6050_Value->acc_x   = ((tmp[0]<<8)+tmp[1]);
	
  MPU6050_Value->acc_y   = ((tmp[2]<<8)+tmp[3]);

  MPU6050_Value->acc_z   = -((tmp[4]<<8)+tmp[5]); 
	/* gyro data */
  MPU6050_Value->gyro_x  =  -((tmp[8]<<8)+tmp[9]);
	
  MPU6050_Value->gyro_y  =  -((tmp[10]<<8)+tmp[11]);

  MPU6050_Value->gyro_z  =  ((tmp[12]<<8)+tmp[13]); 
	
	MPU6050_Value->temperature = (tmp[6]<<8)+tmp[7];
	
	return DEF_OK;
}

/********************************************
 *@brief  : Get 3 Axis Accelemeter data
 *@param	:  -MPU6050_TYPE*
 *@retval : 
 *********************************************/
uint8_t MPU6050_ReadAcc(MPU6050_TYPE* MPU6050_Value)
{
	uint8_t tmp[6];
	/* acc data */
	if(BSP_I2C_MultRead(ACCEL_XOUT_H,MPU6050_SLAVE_ADDR,&tmp[0],6)==DEF_FAIL)
	{
    BSP_Ser_Printf("\n\r MPU6050 IIC ERROR! \n\r");
		return DEF_FAIL;
  }

  MPU6050_Value->acc_y   = ((tmp[0]<<8)+tmp[1]);
	
  MPU6050_Value->acc_x   = -((tmp[2]<<8)+tmp[3]);

  MPU6050_Value->acc_z   = -((tmp[4]<<8)+tmp[5]);
	
  return DEF_OK; 	
}

/********************************************
 *@brief  : Get 3 Axis Gyroscope data
 *@param	:  -MPU6050_TYPE*
 *@retval : 
 *********************************************/
uint8_t MPU6050_ReadGyro(MPU6050_TYPE* MPU6050_Value)
{
	uint8_t tmp[6];
	/* acc data */
	if(BSP_I2C_MultRead(GYRO_XOUT_H,MPU6050_SLAVE_ADDR,&tmp[0],6) == DEF_FAIL)
	{
    BSP_Ser_Printf("\n\r MPU6050 IIC ERROR! \n\r");
		return DEF_FAIL;
  }

  MPU6050_Value->gyro_y   = -((tmp[0]<<8)+tmp[1]);
	
  MPU6050_Value->gyro_x   = ((tmp[2]<<8)+tmp[3]);

  MPU6050_Value->gyro_z   = ((tmp[4]<<8)+tmp[5]); 
  return DEF_OK;	
}

/*
**************************************************************
* Description : Print data read from Sensor
* Argument    : Point to MPU6050_TYPE Structure
**************************************************************
*/
void MPU6050_Printf(MPU6050_TYPE * ptResult)
{
	float ax,ay,az;
	float gx,gy,gz;
	ax = (ptResult->acc_x ) / ACC_SENSITIVITY;  
  ay = (ptResult->acc_y ) / ACC_SENSITIVITY; 	
  az = (ptResult->acc_z ) / ACC_SENSITIVITY; 
  gx = (ptResult->gyro_x) / GYRO_SENSITIVITY;	
  gy = (ptResult->gyro_y) / GYRO_SENSITIVITY;
	gz = (ptResult->gyro_z) / GYRO_SENSITIVITY;
  BSP_Ser_Printf("MPU6050:\tax: %5.2f,\tay: %5.2f,\taz: %5.2f,\tgx: %5.2f,\tgy: %5.2f,\tgz: %5.2f\n\r",
	                ax,ay,az,gx,gy,gz);
	//BSP_Ser_Printf("%8d,%8d,%8d,\n\r",ptResult->acc_x,ptResult->acc_y,ptResult->acc_z);
}
