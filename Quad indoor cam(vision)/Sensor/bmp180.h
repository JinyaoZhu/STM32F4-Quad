#ifndef __BMP180_H__
#define __BMP180_H__

#define BMP180_SLAVE_ADDR 0xEE

/* Register map */
#define BMP180_OUT_XLSB   0xF8
#define BMP180_LSB        0xF7
#define BMP180_MSB        0xF6
#define BMP180_CTRL_MEAS  0xF4
#define BMP180_SOFT_RESET 0xE0
#define BMP180_ID         0xD0
#define BMP180_CALIB21    0xBF
#define BMP180_CALIB20    0xBE
#define BMP180_CALIB19    0xBD
#define BMP180_CALIB18    0xBC
#define BMP180_CALIB17    0xBB
#define BMP180_CALIB16    0xBA
#define BMP180_CALIB15    0xB9
#define BMP180_CALIB14    0xB8
#define BMP180_CALIB13    0xB7
#define BMP180_CALIB12    0xB6
#define BMP180_CALIB11    0xB5
#define BMP180_CALIB10    0xB4
#define BMP180_CALIB09    0xB3
#define BMP180_CALIB08    0xB2
#define BMP180_CALIB07    0xB1
#define BMP180_CALIB06    0xB0
#define BMP180_CALIB05    0xAF
#define BMP180_CALIB04    0xAE
#define BMP180_CALIB03    0xAD
#define BMP180_CALIB02    0xAC
#define BMP180_CALIB01    0xAB
#define BMP180_CALIB00    0xAA

#define BMP180_OSS        0x03  /* oversampling_setting */

#define PRESSURE_SEA_LEVEL    125268.0 /* pressure at sea level */


typedef struct
{
  int16_t  AC1;
	int16_t  AC2;
	int16_t  AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t   B1;
	int16_t   B2;
	int16_t   MB;
	int16_t   MC;
	int16_t   MD;
}BMP180_CalParamStruct;


typedef struct
{
	int32_t ut;   /*uncompensate temperature*/
	int32_t up;   /*uncompensate pressure*/
  int32_t temperature;  /* 0.1 degree Celsius*/
	int32_t pressure;     /* Pa */
}BMP180_TYPE;




/*********************PRIVATE***************************/
uint8_t BMP180_ReadDevID(void);
uint8_t BMP180_GetCalParam(void);
void BMP180_GetUt(BMP180_TYPE *pBMP180_Struct);
void BMP180_GetUp(BMP180_TYPE *pBMP180_Struct);
void BMP180_GetTemperature(BMP180_TYPE *pBMP180_Struct);
void BMP180_CalPressure(BMP180_TYPE *pBMP180_Struct);

/*********************PUBLIC****************************/
uint8_t BMP180_Init(void);
void BMP180_Read(BMP180_TYPE *pBMP180_Struct);


#endif /*__BMP180_H__*/
