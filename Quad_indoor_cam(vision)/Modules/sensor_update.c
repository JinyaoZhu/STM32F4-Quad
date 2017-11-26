#include "bsp.h"
#include "sensor_update.h"
#include "GlobalVariable.h"

//static uint32_t Sensor_Init_Flag = 0;

#define CAL_TH_X 0.20
#define CAL_TH_Y 0.20
#define CAL_TH_Z 0.15

static float AlgRotMat[3][3];
EULER IMU_ErrorEuler = {0, 0.4f*DEG2RAD, -0.6f*DEG2RAD};

/* acceleartion without gain and offset */
const float AccMax_X =   9.699705;
const float AccMin_X =  -9.807779;

const float AccMax_Y = 9.888605;
const float AccMin_Y =  -9.807855;

const float AccMax_Z =  10.0;
const float AccMin_Z =  -9.6;

// const float AccMax_X =  G_VALUE;
// const float AccMin_X = -G_VALUE;

// const float AccMax_Y = G_VALUE;
// const float AccMin_Y = -G_VALUE;

// const float AccMax_Z =  G_VALUE;
// const float AccMin_Z =  -G_VALUE;

static float GyroOffsetX = 0;
static float GyroOffsetY = 0;
static float GyroOffsetZ = 0;

static float AccOffsetX = 0;
static float AccOffsetY = 0;
static float AccOffsetZ = 0;

static float AccGainX = 1;
static float AccGainY = 1;
static float AccGainZ = 1;

static float MagOffsetX = 0;
static float MagOffsetY = 0;
static float MagOffsetZ = 0;

/**************Sensor xyz Gain**************/
extern float MAG_GAIN_X;
extern float MAG_GAIN_Y;
extern float MAG_GAIN_Z;


#define ACC_SCALE_X  (G_VALUE / ACC_SENSITIVITY)
#define ACC_SCALE_Y  (G_VALUE / ACC_SENSITIVITY)
#define ACC_SCALE_Z  (G_VALUE / ACC_SENSITIVITY)

#define GYRO_SCALE_X (DEG2RAD / GYRO_SENSITIVITY)
#define GYRO_SCALE_Y (DEG2RAD / GYRO_SENSITIVITY)
#define GYRO_SCALE_Z (DEG2RAD / GYRO_SENSITIVITY)


/***********************************************/


/*
***********************************************
*            SensorUpdate_Init()
*Init all Sensors
***********************************************
*/
void SensorUpdate_Init(void)
{	
  uint32_t i;
  static uint32_t NUM_SAMPLES = 1000;
  VECTOR g_tmp = {0, 0, 0};
  VECTOR acc,gyro;
  VECTOR acc_gain;
  VECTOR acc_offset;
  EULER  imu_align_euler_tmp = {0,0,0};

  BSP_Ser_Printf("Sensor Init\n\r");

  acc_gain.x = (2.0f*G_VALUE)/(AccMax_X - AccMin_X);
  acc_gain.y = (2.0f*G_VALUE)/(AccMax_Y - AccMin_Y);
  acc_gain.z = (2.0f*G_VALUE)/(AccMax_Z - AccMin_Z);

  acc_offset.x = 0.5f*(AccMax_X + AccMin_X)*acc_gain.x;
  acc_offset.y = 0.5f*(AccMax_Y + AccMin_Y)*acc_gain.y;
  acc_offset.z = 0.5f*(AccMax_Z + AccMin_Z)*acc_gain.z;

  /* when calibrating do not apply any rotation */
  SensorUpdate_UpdateAlignRotMat(imu_align_euler_tmp);
  
  SensorUpdate_SetAccGain(acc_gain.x,acc_gain.y,acc_gain.z);

  SensorUpdate_SetAccOffset(acc_offset.x,acc_offset.y,acc_offset.z);

  SensorUpdate_SetGyroOffset(0,0,0);

  SensorUpdate_SetMagOffset(-0.2196,0.1284,0.4170);

	if(MPU6000_Init() == DEF_OK)
	{
		BSP_Ser_Printf("MPU6000 Init OK...\r\n");
	}
	else
		BSP_Ser_Printf("MPU6000 Init Fail...\r\n");
	
// 	if(HMC5883L_Init() == DEF_OK)
// 	{
// 		BSP_Ser_Printf("HMC5883L Init OK...\r\n");
// 	}
// 	else
// 		BSP_Ser_Printf("HMC5883L Init Fail...\r\n");	
	
// 	if(MS5611_Init() == DEF_OK)
// 		BSP_Ser_Printf("MS5611 Init OK...\r\n");
// 	else
// 		BSP_Ser_Printf("MS5611 Init Fail...\r\n");	 
	
// 	Sonar_Init();
	
	/* Gyro offset calibration */
  do
  {
    BSP_Ser_Printf("Calibrating ...\n\r");
		
    g_tmp.x = 0; g_tmp.y = 0; g_tmp.z = 0;
		
    for (i = 0; i < NUM_SAMPLES ; i++)
    {
			SensorUpdate_GetGyroAccRaw(&gyro,&acc);	   
			VectAdd(&g_tmp,&g_tmp,&gyro);
    }
		
		VectScale(&g_tmp,1.0f/(float)NUM_SAMPLES);
		SensorUpdate_SetGyroOffset(g_tmp.x,g_tmp.y,g_tmp.z);
				
		/* Calibrate check */
		g_tmp.x = 0;g_tmp.y = 0;g_tmp.z = 0;
		
		for (i = 0; i < NUM_SAMPLES ; i++)
		{
			SensorUpdate_GetGyroAccRaw(&gyro,&acc);
			g_tmp.x += gyro.x;
			g_tmp.y += gyro.y;
			g_tmp.z += gyro.z;
		}
		BSP_Ser_Printf("gyro error:x:%5.2f, y:%5.2f, z:%5.2f\n\r", g_tmp.x, g_tmp.y, g_tmp.z);
  } while ((fabs(g_tmp.x) >CAL_TH_X) || (fabs(g_tmp.y) >CAL_TH_Y) || (fabs(g_tmp.z) >CAL_TH_Z));
	
	SensorUpdate_UpdateAlignRotMat(IMU_ErrorEuler);

  BSP_Ser_Printf("Gyroscope Offset:x:%5.2f, y:%5.2f, z:%5.2f\n\r", GyroOffsetX,GyroOffsetY,GyroOffsetZ);
	BSP_Ser_Printf("MAG_OFFSET x:%5.2f, y:%5.2f, z:%5.2f\n\r", MagOffsetX,MagOffsetY,MagOffsetZ);
}

/*
*********************************************************************
*                  SensorUpdate_GetGyroAccRaw()
* Description : Get mpu6050 sensor data and convert to vector.
*********************************************************************
*/
uint8_t SensorUpdate_GetGyroAccRaw(VECTOR *g,VECTOR *a)
{
	uint8_t status;
	
	VECTOR acc_raw,gyro_raw;
	VECTOR acc_rot,gyro_rot;
	
	MPU6000_TYPE  mpu6000 = {0};

  status = MPU6000_Read(&mpu6000);

  if(status == DEF_OK)
	{
		/* m/s^2 */
		acc_raw.x = AccGainX * ACC_SCALE_X * (float)mpu6000.acc_x - AccOffsetX;
		acc_raw.y = AccGainY * ACC_SCALE_Y * (float)mpu6000.acc_y - AccOffsetY;
		acc_raw.z = AccGainZ * ACC_SCALE_Z * (float)mpu6000.acc_z - AccOffsetZ;

		/* Rad/s */
		gyro_raw.x = GYRO_SCALE_X * (float)mpu6000.gyro_x - GyroOffsetX;
		gyro_raw.y = GYRO_SCALE_Y * (float)mpu6000.gyro_y - GyroOffsetY;
		gyro_raw.z = GYRO_SCALE_Z * (float)mpu6000.gyro_z - GyroOffsetZ;
		
		acc_rot.x = AlgRotMat[0][0]*acc_raw.x + AlgRotMat[0][1]*acc_raw.y +AlgRotMat[0][2]*acc_raw.z;
    acc_rot.y = AlgRotMat[1][0]*acc_raw.x + AlgRotMat[1][1]*acc_raw.y +AlgRotMat[1][2]*acc_raw.z;
    acc_rot.z = AlgRotMat[2][0]*acc_raw.x + AlgRotMat[2][1]*acc_raw.y +AlgRotMat[2][2]*acc_raw.z;
	  
    gyro_rot.x = AlgRotMat[0][0]*gyro_raw.x + AlgRotMat[0][1]*gyro_raw.y +AlgRotMat[0][2]*gyro_raw.z;
    gyro_rot.y = AlgRotMat[1][0]*gyro_raw.x + AlgRotMat[1][1]*gyro_raw.y +AlgRotMat[1][2]*gyro_raw.z;
    gyro_rot.z = AlgRotMat[2][0]*gyro_raw.x + AlgRotMat[2][1]*gyro_raw.y +AlgRotMat[2][2]*gyro_raw.z;

		g->x  = gyro_rot.x;
		g->y  = gyro_rot.y;
		g->z  = gyro_rot.z;
		
		a->x = acc_rot.x;
		a->y = acc_rot.y;
		a->z = acc_rot.z;
  }

	return status;
}


/*
*************************************************************
*           SensorUpdate_SetGyroOffset()
*************************************************************
*/
void SensorUpdate_SetGyroOffset(float x,float y,float z)
{
  GyroOffsetX += x;
	GyroOffsetY += y;
	GyroOffsetZ += z;
}

/*
*************************************************************
*           SensorUpdate_SetAccOffset()
*************************************************************
*/
void SensorUpdate_SetAccOffset(float x,float y,float z)
{
  AccOffsetX = x;
	AccOffsetY = y;
	AccOffsetZ = z;
}

/*
*************************************************************
*           SensorUpdate_SetAccGain()
*************************************************************
*/
void SensorUpdate_SetAccGain(float x,float y,float z)
{
  AccGainX = x;
	AccGainY = y;
	AccGainZ = z;
}


/*
*************************************************************
*           SensorUpdate_SetMagOffset()
*************************************************************
*/
void SensorUpdate_SetMagOffset(float x,float y,float z)
{
  MagOffsetX = x;
	MagOffsetY = y;
	MagOffsetZ = z;
}


/*
*********************************************************************
*                      SensorUpdate_GetMagRaw()
* Description : Get magnetometer sensor data and convert to vector.
*********************************************************************
*/
uint8_t SensorUpdate_GetMagRaw(VECTOR *mag)
{
	uint8_t status;
	float mag_x_tmp,mag_y_tmp,mag_z_tmp;
	
	HMC5883L_TYPE hmc5883l = {0};

  status = HMC5883L_Read(&hmc5883l);
 
  if(status == DEF_OK)
	{
		mag_x_tmp  = -((float)hmc5883l.hx * MAG_GAIN_X/HMC5883L_SENSITIVITY) - MagOffsetX; /* Gauss */
		mag_y_tmp  = -((float)hmc5883l.hz * MAG_GAIN_Z/HMC5883L_SENSITIVITY) - MagOffsetY;
		mag_z_tmp  =  ((float)hmc5883l.hy * MAG_GAIN_Y/HMC5883L_SENSITIVITY) - MagOffsetZ;

		mag->x = mag_x_tmp;
		mag->y = mag_y_tmp;
		mag->z = mag_z_tmp;
	}
  return status; 
}

void SensorUpdate_UpdateAlignRotMat(const EULER euler)
{
  AlgRotMat[0][0] = cos(euler.yaw)*cos(euler.pitch);
  AlgRotMat[0][1] = cos(euler.yaw)*sin(euler.pitch)*sin(euler.roll)-sin(euler.yaw)*cos(euler.roll);
  AlgRotMat[0][2] = cos(euler.yaw)*sin(euler.pitch)*cos(euler.roll)+sin(euler.yaw)*sin(euler.roll);
  AlgRotMat[1][0] = sin(euler.yaw)*cos(euler.pitch);
  AlgRotMat[1][1] = sin(euler.yaw)*sin(euler.pitch)*sin(euler.roll)+cos(euler.yaw)*cos(euler.roll);
  AlgRotMat[1][2] = sin(euler.yaw)*sin(euler.pitch)*cos(euler.roll)-cos(euler.yaw)*sin(euler.roll);
  AlgRotMat[2][0] = -sin(euler.pitch);
  AlgRotMat[2][1] = cos(euler.pitch)*sin(euler.roll);
  AlgRotMat[2][2] = cos(euler.pitch)*cos(euler.roll);
}
