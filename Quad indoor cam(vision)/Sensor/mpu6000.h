#ifndef __MPU6000_H
#define __MPU6000_H

#define MPU6000_CS_PIN    GPIO_Pin_4
#define MPU6000_CS_PORT   GPIOC

#define MPU6000_CS_HIGH    GPIO_SetBits(MPU6000_CS_PORT, MPU6000_CS_PIN)
#define MPU6000_CS_LOW     GPIO_ResetBits(MPU6000_CS_PORT, MPU6000_CS_PIN)

#define MPU6000_INT_PIN    GPIO_Pin_0
#define MPU6000_INT_PORT   GPIOC

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
MPU6000_TYPE;


/****************************************
 * 定义MPU6000内部地址
 *****************************************/
#define	SMPLRT_DIV		  0x19	
#define	CONFIG          0x1A	
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
#define INT_ENABLE    0x38
#define INT_STATUS    0x3A
#define	WHO_AM_I			0x75


//#define ACC_SENSITIVITY  16384.0f /* +- 2g */
#define ACC_SENSITIVITY  8192.0f /* +- 4g */
//#define ACC_SENSITIVITY  4096.0f /* +- 8g */
//#define ACC_SENSITIVITY  2048.0f /* +- 16g */

// #define GYRO_SENSITIVITY 32.8f    /* +- 1000 degree/s */
//#define GYRO_SENSITIVITY 65.5f    /* +- 500 degree/s */
#define GYRO_SENSITIVITY 16.4f    /* +- 2000 degree/s */
 
#define MPU6000_Delay(ms)         BSP_OS_TimeDlyMs(ms)
/*-------------------------------------------------------------------------------------------------------------*/

/***********PRIVATE**********/
u8 	 MPU6000_Check(void);


/***********PUBLIC***********/
uint8_t MPU6000_Init(void);

void MPU6000_Printf(MPU6000_TYPE * ptResult);

uint8_t MPU6000_ReadAcc(MPU6000_TYPE* MPU6000_Value);
uint8_t MPU6000_ReadGyro(MPU6000_TYPE* MPU6000_Value);
uint8_t MPU6000_Read(MPU6000_TYPE* MPU6000_Value);

void MPU6000_SingleWrite(uint8_t address,uint8_t value);
uint8_t MPU6000_SingleRead(uint8_t address);
void MPU6000_BrustRead(uint8_t address,uint8_t *p_buf,uint16_t length);
void MPU6000_ClearInterrupt(void);
void BSP_EXTI0_ISR_Handler(void);

#endif /* __MPU6000_H */
