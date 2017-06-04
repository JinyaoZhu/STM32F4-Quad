#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#include "bsp.h"

#define USE_ARM_MATH

#ifndef USE_ARM_MATH
  #define PI     (3.14159265358979f)
	#include "math.h"
#else
  #include "arm_math.h"
#endif /* USE_ARM_MATH */

#include "cordic.h"

#define DEG2RAD        (PI/180.0f)
#define RAD2DEG        (180.0f/PI)

#define HZ2RAD         (2*PI)

typedef struct 
{
  float w;
  float x;
  float y;
  float z;
}QUATERNION;


typedef struct 
{
  float x;
  float y;
  float z;
}VECTOR;


typedef struct 
{
  float yaw;
  float pitch;
  float roll;
}EULER;


typedef struct
{
  float old_data;
	float output;
}DHPF_Struct;


typedef union 
{
	float f;
	uint16_t uint[2];
}f2uint16_t;

typedef union 
{
	float f;
	uint8_t b[4];
}f2b_t;

typedef union
{
  int16_t w;
	uint8_t b[2];
}sw2b_t;


typedef struct
{
  uint32_t cnt;
	int32_t sum;
	int32_t FIFO_Size;
	int32_t *FIFO;
}MovAvgFIFOStruct_int;


typedef struct
{
  uint32_t cnt;
	float sum;
	uint32_t FIFO_Size;
	float *FIFO;
}MovAvgFIFOStruct_f;


/*****************************QUATERNION AND VECTOR*******************************/
float InvSqrt(float x);

void QuaternionNormalize(QUATERNION *q);

void QuaternionConj(QUATERNION *result,const QUATERNION q);

void QuaternionMul(QUATERNION *result ,const QUATERNION q_left ,const QUATERNION q_right);

void QuaternionCopy(const QUATERNION a ,QUATERNION *b );

void RotateVet1(const QUATERNION *rotation ,VECTOR *a,VECTOR *b);

void RotateVet2(const QUATERNION *rotation ,const VECTOR *a, VECTOR *b);

float VectNormalize(VECTOR *v);

float VectNorm(VECTOR *v);

void VectCrossProduct(VECTOR *result ,const VECTOR a,const VECTOR b);

void VectAdd(VECTOR *result , VECTOR *a, VECTOR *b);

void VectSub(VECTOR *result , VECTOR *a, VECTOR *b);

void VectScale(VECTOR *v , float a);

void VectCopy(VECTOR *a, VECTOR *b);

void GetQuaternion(QUATERNION *q ,const VECTOR from,const VECTOR to);

void VectorToQuaternion(QUATERNION *q,VECTOR *v);

void QuaternionToVector(VECTOR *v,QUATERNION *q);

void ConvertToEuler(QUATERNION *q , EULER *e);

void ConvertToQuaternion(QUATERNION *q,EULER *e);

void EulerRoateVectSmall(VECTOR *vect,EULER *ypr);

void EulerRoateVect(VECTOR *to,VECTOR *from,EULER *e);

void EulerRoateVectXY(VECTOR *to,VECTOR *from,EULER *e);
/************************FILTER*******************************/
void     MoveAverage_int_FIFOStructInit(MovAvgFIFOStruct_int *p,int32_t *FIFO,int32_t size_of_FIFO);

int32_t  MoveAverage_int(int32_t new_data,MovAvgFIFOStruct_int *p);

void     MoveAverage_f_FIFOStructInit(MovAvgFIFOStruct_f *p,float *FIFO,uint32_t size_of_FIFO);

float    MoveAverage_f(float new_data,MovAvgFIFOStruct_f *p);

float    DHPF(float new_data,DHPF_Struct *p);

float    DLPF(float new_data,float old_data,float a,float dt);

float    WarpToPI(float a);

int16_t _atan2(float y, float x) ;

float applyDeadband(float x,float deadband);

float max(float a,float b);

float min(float a,float b);

void sort(uint16_t* arr,uint16_t len);

uint16_t median_5(uint16_t *a);

uint8_t checksum(uint8_t *data, uint16_t len);
/*********************CONSTRAINT**************************/
float    Constraint_f(float a , float lower ,float upper);

int32_t  Constraint_int(int32_t a , int32_t low ,int32_t upper);

#endif /*__ALGORITHM_H__*/
