#ifndef __MPU6050_H
#define __MPU6050_H

/* 存放MPU6050数据的结构体 */
typedef struct
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t gyro_x;  //rad/s
  int16_t gyro_y;
  int16_t gyro_z;
	int16_t temperature;
}
MPU6050_TYPE;


/*--------------外设地址-------------*/
#define MPU6050_SLAVE_ADDR 0xD0
/*-------------主机自身地址--------------*/
#define I2C_OWNADDRESS1    0x01
/****************************************
 * 定义MPU6050内部地址
 *****************************************/
#define	SMPLRT_DIV		  0x19	
#define	MPU6050_CONFIG  0x1A	
#define	GYRO_CONFIG		  0x1B	
#define	ACCEL_CONFIG	  0x1C	

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define USER_CTRL     0x6A
#define	PWR_MGMT_1		0x6B	
#define INT_PIN_CFG   0x37
#define	WHO_AM_I			0x75

/* 灵敏度 */
#define ACC_SENSITIVITY  16384.0 /* +- 2g */
#define GYRO_SENSITIVITY 32.8    /* +- 1000 degree/s */
 
#define MPU6050_Delay(ms)         BSP_OS_TimeDlyMs(ms)
/*-------------------------------------------------------------------------------------------------------------*/

/***********PRIVATE**********/
u8 	 MPU6050_Check(void);


/***********PUBLIC***********/
void MPU6050_Init(void);

void MPU6050_Printf(MPU6050_TYPE * ptResult);

uint8_t MPU6050_ReadAcc(MPU6050_TYPE* MPU6050_Value);
uint8_t MPU6050_ReadGyro(MPU6050_TYPE* MPU6050_Value);
uint8_t MPU6050_Read(MPU6050_TYPE* MPU6050_Value);

#endif /* __MPU6050_H */
