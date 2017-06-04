#include "bsp.h"
#include "math.h"
#include "GlobalVariable.h"

#define MS5611_DELAY        BSP_OS_TimeDlyMs(10);

typedef struct
{
  float C1,C2,C3,C4,C5,C6; /* Calibrate data read form ms5611 */
  float D1; /* Digital pressure value */
  float D2; /* Digital temperature value */
  float dT; /* Different between actual and referance temperature */
  float  T;  /* temperature */
	float  P;  /* mbar */
 	float pressure;
 	float temperature;
}MS5611_TYPE;



MS5611_TYPE MS5611_Data = {0};

static float MS5611_GroundOffset;

uint8_t MS5611_Init(void)
{	
	uint8_t status;
	
	MS5611_DELAY;
  MS5611_Reset();
	MS5611_DELAY;
	status = MS5611_ReadPROM();
	MS5611_DELAY;	
	
	MS5611_SetGroundOffset(0);
	
	BSP_Ser_Printf("Altitude:%f\r\n",MS5611_GetAltitude());
	
	return status;
}

/*
****************************************************************
*                   MS5611_Reset()
* Description : 
****************************************************************
*/
void MS5611_Reset(void)
{
  BSP_I2C3_SingleWriteNoMemAddr(MS561101BA_RST,MS561101BA_SlaveAddress);
}

/*
****************************************************************
*                 MS5611_ReadPROM()
* Description : Read calibrate data from ms5611 PROM
****************************************************************
*/
uint8_t MS5611_ReadPROM(void)
{
	uint8_t i = 0;
	uint16_t buf[6];
	for(i = 0; i<6 ;i++)
	{
		BSP_I2C3_SingleWriteNoMemAddr(MS561101BA_PROM_RD + i*2u,MS561101BA_SlaveAddress);
    buf[i] = BSP_I2C3_SingleRead16NoMemAddr(MS561101BA_SlaveAddress);
	}
	MS5611_Data.C1 = buf[0];
	MS5611_Data.C2 = buf[1];
	MS5611_Data.C3 = buf[2];
	MS5611_Data.C4 = buf[3];
	MS5611_Data.C5 = buf[4];
	MS5611_Data.C6 = buf[5];
	
	//BSP_Ser_Printf("%f,%f,%f,%f,%f,%f\r\n",MS5611_Data.C1,MS5611_Data.C2,MS5611_Data.C3,MS5611_Data.C4,MS5611_Data.C5,MS5611_Data.C6);
	
	if((MS5611_Data.C1 == 0)||(MS5611_Data.C2 == 0)||(MS5611_Data.C3 == 0)||
		 (MS5611_Data.C4 == 0)||(MS5611_Data.C5 == 0)||(MS5611_Data.C6 == 0))
	{
		return DEF_FAIL;
  }
	else
		return DEF_OK;
}

/*
***********************************************************************
*               MS5611_ReadPressureAndTemperatureData()
* Description : Get filtered pressure and temperature data.
*               
***********************************************************************
*/
void MS5611_ReadPressureAndTemperatureData(void)
{		
	BSP_I2C3_SingleWriteNoMemAddr(MS561101BA_D1_OSR_4096,MS561101BA_SlaveAddress);
	MS5611_DELAY; /* Convertion max time @OSR4096 = 9.04ms*/
	BSP_I2C3_SingleWriteNoMemAddr(MS561101BA_ADC_RD,MS561101BA_SlaveAddress);
	MS5611_Data.D1 = BSP_I2C3_SingleRead24NoMemAddr(MS561101BA_SlaveAddress);
	
	BSP_I2C3_SingleWriteNoMemAddr(MS561101BA_D2_OSR_4096,MS561101BA_SlaveAddress);
	MS5611_DELAY; /* Convertion max time @OSR4096 = 9.04ms*/
	BSP_I2C3_SingleWriteNoMemAddr(MS561101BA_ADC_RD,MS561101BA_SlaveAddress);
	MS5611_Data.D2 = BSP_I2C3_SingleRead24NoMemAddr(MS561101BA_SlaveAddress);
}


/*
***********************************************************************
*                   MS5611_CalculateTemperature()
* Description : Calculate actual temperature value
*
***********************************************************************
*/
void MS5611_CalculateTemperature(void)
{
  MS5611_Data.dT = MS5611_Data.D2 - MS5611_Data.C5*256.0f;
	MS5611_Data.T = 2000 + MS5611_Data.dT * MS5611_Data.C6 / 8388608.0f;/* Actual temperature */
}

/*
***********************************************************************
*        MS5611_CalculateCompensatedPressure()
* Description : Get compensated pressure value
*
***********************************************************************
*/
void MS5611_CalculateCompensatedPressure(void)
{
  double OFF2=0,SENS2=0,T2=0;
  double OFF;  /* Offset at actual temperature */
	double SENS; /* Sensitivity at actual temperature */
  double aux1,aux2;
	
	OFF  = (MS5611_Data.C2*65536.0f) + (MS5611_Data.C4 * MS5611_Data.dT/128.0f);
	SENS = (MS5611_Data.C1*32768.0f) + (MS5611_Data.C3 * MS5611_Data.dT/256.0f);
	
  if (MS5611_Data.T < 2000)     /* temperature lower than 20st.C */
	{ 
    T2 = (MS5611_Data.dT*MS5611_Data.dT)/(double)0x80000000;
		aux1 = MS5611_Data.T - 2000.0f;
		aux1 *= aux1;
    OFF2  = 2.5f * aux1; 
    SENS2 = 1.25f * aux1; 
    if (MS5611_Data.T < -1500.0f) 
	  { // temperature lower than -15st.C
	 	  aux2 = MS5611_Data.T + 1500.0f;
	 	  aux2 *= aux2;
      OFF2  += 7.0f * aux2; 
      SENS2 += 5.5f * aux2; 
    }
	  OFF  -= OFF2; 
    SENS -= SENS2;
	  MS5611_Data.T -= T2;
  }
	
	MS5611_Data.P = (MS5611_Data.D1 * SENS/2097152.0f - OFF)/32768.0f; /* Compensated pressure */
}

/*
***********************************************************************
*                   MS5611_GetAltitude()
* Description : Calculate Altitude.
*
***********************************************************************
*/
float MS5611_GetAltitude(void)
{
	MS5611_ReadPressureAndTemperatureData();
  MS5611_CalculateTemperature();
	MS5611_CalculateCompensatedPressure();
  MS5611_Data.temperature = MS5611_Data.T/100.0f; /* ¡æ */
  MS5611_Data.pressure = MS5611_Data.P/100.0f;    /* mbar */
	return ((1.0f - pow(MS5611_Data.pressure/1013.25f, 0.190295f)) * 44330.0f - MS5611_GroundOffset);/* meter */
}


void MS5611_SetGroundOffset(float offset)
{
  MS5611_GroundOffset = offset;
}
