#include "algorithm.h"





void EulerRoateVectSmall(VECTOR *vect, EULER *ypr)
{
  VECTOR tmp;
  VectCopy(vect, &tmp);
  vect->x =               tmp.x + ypr->yaw  * tmp.y - ypr->pitch * tmp.z;
  vect->y =   -ypr->yaw * tmp.x +             tmp.y + ypr->roll  * tmp.z;
  vect->z =  ypr->pitch * tmp.x - ypr->roll * tmp.y +              tmp.z;
}

/* from body to referent */
void EulerRoateVect(VECTOR *to, VECTOR *from, EULER *e)
{
  float sin_yaw, cos_yaw;
  float sin_pitch, cos_pitch;
  float sin_roll, cos_roll;
  VECTOR tmp;
  VectCopy(from, &tmp);
  sin_yaw = arm_sin_f32(e->yaw);
  cos_yaw = arm_cos_f32(e->yaw);
  sin_pitch = arm_sin_f32(e->pitch);
  cos_pitch = arm_cos_f32(e->pitch);
  sin_roll = arm_sin_f32(e->roll);
  cos_roll = arm_cos_f32(e->roll);
// 	  sin_yaw = sinf(e->yaw);
//   cos_yaw = cosf(e->yaw);
//   sin_pitch = sinf(e->pitch);
//   cos_pitch = cosf(e->pitch);
//   sin_roll = sinf(e->roll);
//   cos_roll = cosf(e->roll);
	
  to->x = tmp.x * (cos_yaw * cos_pitch) + tmp.y * (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) + tmp.z * (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll);
  to->y = tmp.x * (sin_yaw * cos_pitch) + tmp.y * (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) + tmp.z * (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll);
  to->z = tmp.x * (-sin_pitch)          + tmp.y * (cos_pitch * sin_roll)                                + tmp.z * (cos_pitch * cos_roll);
}

void EulerRoateVectXY(VECTOR *to, VECTOR *from, EULER *e)
{
  float sin_pitch, cos_pitch;
  float sin_roll, cos_roll;
  VECTOR tmp;
  VectCopy(from, &tmp);
  sin_pitch = sin(e->pitch);
  cos_pitch = cos(e->pitch);
  sin_roll = sin(e->roll);
  cos_roll = cos(e->roll);
  //fast_SinCos(e->pitch, &sin_pitch, &cos_pitch);
  //fast_SinCos(e->roll, &sin_roll, &cos_roll);
  to->x = tmp.x * cos_pitch + tmp.y * (sin_pitch * sin_roll) + tmp.z * (sin_pitch * cos_roll);
  to->y = tmp.y * (cos_roll) + tmp.z * (-sin_roll);
  to->z = tmp.x * (-sin_pitch) + tmp.y * (cos_pitch * sin_roll) + tmp.z * (cos_pitch * cos_roll);
}

/*
***************************************************
               QuaternionNormalize()
  Discription : Normalize
***************************************************
*/
void QuaternionNormalize(QUATERNION *q)
{
  float norm_inv = 1.0f / sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  (q->w) *= norm_inv;
  (q->x) *= norm_inv;
  (q->y) *= norm_inv;
  (q->z) *= norm_inv;
}


/*
***************************************************
                   QuaternionConj()
  Discription : Conjunction of a Quaternion
***************************************************
*/
void QuaternionConj(QUATERNION *result, const QUATERNION q)
{
  result->w =   q.w;
  result->x = -(q.x);
  result->y = -(q.y);
  result->z = -(q.z);
}


/*
***************************************************
                QuaternionCopy()
  Discription : Copy a Quadternion from a to b
***************************************************
*/
void QuaternionCopy(const QUATERNION a , QUATERNION *b )
{
  b->w = a.w;
  b->x = a.x;
  b->y = a.y;
  b->z = a.z;
}

/*
***************************************************
                VectNormalize()
  Discription : Normalize
  Return      : 1/|v|
***************************************************
*/
float VectNormalize(VECTOR *v)
{
  float norm_inv = 1.0f / sqrtf((v->x) * (v->x) + (v->y) * (v->y) + (v->z) * (v->z));
  (v->x) *= norm_inv;
  (v->y) *= norm_inv;
  (v->z) *= norm_inv;
  return norm_inv;
}

/*
***************************************************
              QuaternionMul()
  Discription : result = q_left x q_right
***************************************************
*/
void QuaternionMul(QUATERNION *result , const QUATERNION q_left , const QUATERNION q_right)
{

  result->w = q_left.w * q_right.w - q_left.x * q_right.x - q_left.y * q_right.y - q_left.z * q_right.z;
  result->x = q_left.x * q_right.w + q_left.w * q_right.x + q_left.y * q_right.z - q_left.z * q_right.y;
  result->y = q_left.y * q_right.w + q_left.w * q_right.y + q_left.z * q_right.x - q_left.x * q_right.z;
  result->z = q_left.z * q_right.w + q_left.w * q_right.z + q_left.x * q_right.y - q_left.y * q_right.x;
}

/*
*****************************************************
                    RotateVet1()
  Discription : Using quaternion rotate a VECTOR
                b(quadcopter coordinate)->R(Referent Coordinate) a->b
*****************************************************
*/
void RotateVet1(const QUATERNION *rotation , VECTOR *a, VECTOR *b)
{
  VECTOR from;
  float _2x  = (rotation->x) * 2.0f;
  float _2y  = (rotation->y) * 2.0f;
  float _2z  = (rotation->z) * 2.0f;
  float w_2x = (rotation->w) *  _2x;
  float w_2y = (rotation->w) *  _2y;
  float w_2z = (rotation->w) *  _2z;
  float x_2x = (rotation->x) *  _2x;
  float x_2y = (rotation->x) *  _2y;
  float x_2z = (rotation->x) *  _2z;
  float y_2y = (rotation->y) *  _2y;
  float y_2z = (rotation->y) *  _2z;
  float z_2z = (rotation->z) *  _2z;

  VectCopy(a, &from);

  b->x = (from.x) * (1.0f - y_2y - z_2z) + (from.y) * (x_2y - w_2z)        + (from.z) * (x_2z + w_2y);
  b->y = (from.x) * (x_2y + w_2z)        + (from.y) * (1.0f - x_2x - z_2z) + (from.z) * (y_2z - w_2x);
  b->z = (from.x) * (x_2z - w_2y)        + (from.y) * (y_2z + w_2x)        + (from.z) * (1.0f - x_2x - y_2y);
}

/*
*****************************************************
                   RotateVet2()
  Discription : Using a quaternion rotate a VECTOR
                R(Referent Coordinate) ->b(quadcopter coordinate)
*****************************************************
*/
void RotateVet2(const QUATERNION *rotation , const VECTOR *a, VECTOR *b)
{
  float _2x  = (rotation->x) * 2.0f;
  float _2y  = (rotation->y) * 2.0f;
  float _2z  = (rotation->z) * 2.0f;
  float w_2x = (rotation->w) *  _2x;
  float w_2y = (rotation->w) *  _2y;
  float w_2z = (rotation->w) *  _2z;
  float x_2x = (rotation->x) *  _2x;
  float x_2y = (rotation->x) *  _2y;
  float x_2z = (rotation->x) *  _2z;
  float y_2y = (rotation->y) *  _2y;
  float y_2z = (rotation->y) *  _2z;
  float z_2z = (rotation->z) *  _2z;


  b->x = (a->x) * (1.0f - y_2y - z_2z) + (a->y) * (x_2y + w_2z)        + (a->z) * (x_2z - w_2y);
  b->y = (a->x) * (x_2y - w_2z)        + (a->y) * (1.0f - x_2x - z_2z) + (a->z) * (y_2z + w_2x);
  b->z = (a->x) * (x_2z + w_2y)        + (a->y) * (y_2z - w_2x)        + (a->z) * (1.0f - x_2x - y_2y);
}

/*
***************************************************
                   VectNorm()
  Discription : Compute Nrom of a VECTOR

***************************************************
*/
float VectNorm(VECTOR *v)
{
  return sqrt((v->x) * (v->x) + (v->y) * (v->y) + (v->z) * (v->z));
}


/*
***************************************************
               AHRS_VectCrossProduct()
  Discription : Compute cross product of two VECTOR
                  a x b
***************************************************
*/
void VectCrossProduct(VECTOR *result , const VECTOR a, const VECTOR b)
{
  result->x = a.y * b.z - a.z * b.y;
  result->y = a.z * b.x - a.x * b.z;
  result->z = a.x * b.y - a.y * b.x;
}

/*
***************************************************
                   VectAdd()
  Discription : Compute cross sum of two VECTOR
                  a + b
***************************************************
*/
void VectAdd(VECTOR *result , VECTOR *a, VECTOR *b)
{
  VECTOR tmp_a, tmp_b;
  VectCopy(a, &tmp_a);
  VectCopy(b, &tmp_b);
  result->x = tmp_a.x + tmp_b.x;
  result->y = tmp_a.y + tmp_b.y;
  result->z = tmp_a.z + tmp_b.z;
}

/*
***************************************************
                   VectSub()
  Discription : Compute VECTOR a-b

***************************************************
*/
void VectSub(VECTOR *result , VECTOR *a, VECTOR *b)
{
  VECTOR tmp_a, tmp_b;
  VectCopy(a, &tmp_a);
  VectCopy(b, &tmp_b);
  result->x = tmp_a.x - tmp_b.x;
  result->y = tmp_a.y - tmp_b.y;
  result->z = tmp_a.z - tmp_b.z;
}

/*
***************************************************
                 VectCopy()
  Discription : Copy a Vector from a to b

***************************************************
*/
void VectCopy(VECTOR *a, VECTOR *b)
{
  b->x = a->x;
  b->y = a->y;
  b->z = a->z;
}

/*
****************************************************
                   VectScale()
  Discription : Gruop operation
                scalar
***************************************************
*/
void VectScale(VECTOR *v , const float a)
{
  v->x = (v->x) * a;
  v->y = (v->y) * a;
  v->z = (v->z) * a;
}
/*
***************************************************
                GetQuaternion()
  Discription : Get quaternion from two VECTORs whitch
                have been rotated
***************************************************
*/
void GetQuaternion(QUATERNION *q , const VECTOR from, const VECTOR to)
{
  VECTOR w;
  float norm_w_inv;
  float cos_theta;
  float sin_half_theta;

  float from_inv_norm = 1.0f / sqrt(from.x * from.x + from.y * from.y + from.z * from.z);
  float to_inv_norm   = 1.0f / sqrt(to.x * to.x + to.y * to.y + to.z * to.z);

  /* theta = angle between vect_from and vect_to */
  cos_theta = (from.x * to.x + from.y * to.y + from.z * to.z) * (from_inv_norm * to_inv_norm);

  sin_half_theta = sqrt((1.0f - cos_theta) * 0.5f);
  /* w = from x to */
  VectCrossProduct(&w, from, to);

  norm_w_inv = 1.0f / sqrt(w.x * w.x + w.y * w.y + w.z * w.z);

  q->w = sqrtf((1.0f + cos_theta) / 2.0f); //cos(theta/2)
  q->x = w.x * norm_w_inv * sin_half_theta;
  q->y = w.y * norm_w_inv * sin_half_theta;
  q->z = w.z * norm_w_inv * sin_half_theta;
}


/*
*******************************************************
                  VectorToQuaternion()
  Description : Convert a Vector to Quaternion
*******************************************************
*/
void VectorToQuaternion(QUATERNION *q, VECTOR *v)
{
  q->w = 0.0f;
  q->x = v->x;
  q->y = v->y;
  q->z = v->z;
}

/*
*******************************************************
                  QuaternionToVector()
  Description : Convert a Quaternion to Vector
*******************************************************
*/
void QuaternionToVector(VECTOR *v, QUATERNION *q)
{
  v->x = q->x;
  v->y = q->y;
  v->z = q->z;
}

/*
***************************************************
                  ConvertToEuler()
  Discription : Convert Quaternion to Euler angle
                (Tait-Bryan angle) yaw-pitch-roll
***************************************************
*/
void ConvertToEuler(QUATERNION *q , EULER *e)
{
  e->yaw   = atan2(2.0f * (q->w) * (q->z) + 2.0f * (q->x) * (q->y), 1.0f - 2.0f * (q->y) * (q->y) - 2.0f * (q->z) * (q->z));
  e->pitch = asin(2.0f * (q->w) * (q->y) - 2.0f * (q->z) * (q->x));
  e->roll  = atan2(2.0f * (q->w) * (q->x) + (q->y) * (q->z) , 1.0f - 2.0f * (q->x) * (q->x) - 2.0f * (q->y) * (q->y));
}


/*
*************************************************
           ConvertToQuaternion()
  Description : convert quaternion to Tait-Bryan angles
*************************************************
*/
void ConvertToQuaternion(QUATERNION *q, EULER *e)
{
  float yaw_half, pitch_half, roll_half;
  float cos_yaw_half, sin_yaw_half;
  float cos_pitch_half, sin_pitch_half;
  float cos_roll_half, sin_roll_half;
  //  float q_norm_inv;
  //  QUATERNION q_tmp;
  yaw_half   = e->yaw * 0.5f;
  pitch_half = e->pitch * 0.5f;
  roll_half  = e->roll * 0.5f;

  sin_yaw_half = sin(yaw_half);
  cos_yaw_half = cos(yaw_half);
  sin_pitch_half = sin(pitch_half);
  cos_pitch_half = cos(pitch_half);
  sin_roll_half = sin(roll_half);
  cos_roll_half = cos(roll_half);
  //fast_SinCos(yaw_half, &sin_yaw_half, &cos_yaw_half);
  //fast_SinCos(pitch_half, &sin_pitch_half, &cos_pitch_half);
  //fast_SinCos(roll_half, &sin_roll_half, &cos_roll_half);

  q->w = cos_roll_half * cos_pitch_half * cos_yaw_half + sin_roll_half * sin_pitch_half * sin_yaw_half;
  q->x = sin_roll_half * cos_pitch_half * cos_yaw_half - cos_roll_half * sin_pitch_half * sin_yaw_half;
  q->y = cos_roll_half * sin_pitch_half * cos_yaw_half + sin_roll_half * cos_pitch_half * sin_yaw_half;
  q->z = cos_roll_half * cos_pitch_half * sin_yaw_half - sin_roll_half * sin_pitch_half * cos_yaw_half;

  /* normalize */
  //  q_norm_inv = InvSqrt(q_tmp.w*q_tmp.w + q_tmp.x*q_tmp.x + q_tmp.y*q_tmp.y + q_tmp.z*q_tmp.z);
  //  q->w = q_tmp.w * q_norm_inv;
  //  q->x = q_tmp.x * q_norm_inv;
  //  q->y = q_tmp.y * q_norm_inv;
  //  q->z = q_tmp.z * q_norm_inv;
}

/*
****************************************************************
                 MoveAverage_int_FIFOStructInit()
  Description : Initialize FIFO
  Parameter   : *p    MovAvgFIFO_int
                FIFO  an int32 type array
                size_of_FIFO
****************************************************************
*/
void MoveAverage_int_FIFOStructInit(MovAvgFIFOStruct_int *p, int32_t *FIFO, int32_t size_of_FIFO)
{
  p->cnt = 0;
  p->sum = 0;
  p->FIFO_Size = size_of_FIFO;
  p->FIFO = FIFO;
}

/*
****************************************************************
                 MoveAverage_int()
  Description : Initialize FIFO
  Parameter   :
****************************************************************
*/
int32_t MoveAverage_int(int32_t new_data, MovAvgFIFOStruct_int *p)
{
  p->sum  -= (p->FIFO[p->cnt]);
  p->FIFO[p->cnt] = new_data;
  p->sum  += p->FIFO[p->cnt];
  p->cnt = (p->cnt + 1) % (p->FIFO_Size);
  return ((p->sum) / (p->FIFO_Size));
}

/*
****************************************************************
                 MoveAverage_f_FIFOStructInit()
  Description : Initialize FIFO
  Parameter   : *p    MovAvgFIFO_int
                FIFO  an float type array
                size_of_FIFO
****************************************************************
*/
void MoveAverage_f_FIFOStructInit(MovAvgFIFOStruct_f *p, float *FIFO, uint32_t size_of_FIFO)
{
  p->cnt = 0;
  p->sum = 0.0f;
  p->FIFO_Size = size_of_FIFO;
  p->FIFO = FIFO;
}

/*
****************************************************************
                    MoveAverage_f()
  Description : Initialize FIFO
  Parameter   :
****************************************************************
*/
float MoveAverage_f(float new_data, MovAvgFIFOStruct_f *p)
{
  p->sum  -= (p->FIFO[p->cnt]);
  p->FIFO[p->cnt] = new_data;
  p->sum  += p->FIFO[p->cnt];
  p->cnt = (p->cnt + 1) % (p->FIFO_Size);
  return ((p->sum) / (float)(p->FIFO_Size));
}



float Constraint_f(float a , float lower , float upper)
{
  if (a >= upper)
    a = upper;
  else if (a <= lower)
    a = lower;
  return (a);
}

int32_t Constraint_int(int32_t a , int32_t lower , int32_t upper)
{
  if (a >= upper)
    return upper;
  else if (a <= lower)
    return lower;
  else
    return a;
}

float DHPF(float new_data, DHPF_Struct *p)
{
  p->output *= 0.999f;
  (p->output) += (new_data - (p->old_data));
  (p->old_data) = new_data;
  return (p->output);
}

/*
****************************************************************
                       DLPF()
  Description : Low pass filter cutoff frequency 'a' (rad/s)
****************************************************************
*/
float DLPF(float new_data, float old_data, float a, float dt)
{
  float   aT = a * dt;
  aT = Constraint_f(aT, 0.0f, 0.9999999f);
  return ((1.0f - aT) * old_data + aT * new_data);
}


int32_t abs_int(int32_t x)
{
  if (x < 0)
    return -x;
  else
    return x;
}


int16_t _atan2(float y, float x) {
  float z = y / x;
  int16_t a;
  if ( fabs(y) < fabs(x) ) {
    a = (int16_t)(573.0f * z / (1.0f + 0.28f * z * z));
    if (x < 0) {
      if (y < 0)
        a -= 1800;
      else
        a += 1800;
    }
  }
  else {
    a = (int16_t)(900 - 573 * z / (z * z + 0.28f));
    if (y < 0)
      a -= 1800;
  }
  return a;
}



float InvSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

float WarpToPI(float a)
{
  while (a > PI)
    a -= 2.0f * PI;
  while (a < -PI)
    a += 2.0f * PI;
  return a;
}

float applyDeadband(float x, float deadband)
{
  if (fabs(x) < deadband) {
    return 0;
  }
  else if (x > 0) {
    return (x - deadband);
  }
  else {
    return (x + deadband);
  }
}

float max(float a, float b)
{
  if (a > b)
    return a;
  else
    return b;
}

float min(float a, float b)
{
  if (a < b)
    return a;
  else
    return b;
}



void sort(uint16_t* arr, uint16_t len)
{
  int16_t i, j, temp;
  for (j = 0; j < (len - 1); j++) {
    for (i = 0; i < (len - 1) - j; i++) {
      if (arr[i] > arr[i + 1]) {
        temp = arr[i];
        arr[i] = arr[i + 1];
        arr[i + 1] = temp;
      }
    }
  }
}

uint16_t median_5(uint16_t *a)
{
  uint16_t temp[5];
  uint16_t i;
  for (i = 0; i < 5; i++) {
    temp[i] = a[i];
  }
  sort(temp, 5);
  return temp[2];
}

uint8_t checksum(uint8_t *data, uint16_t len)
{
  uint32_t sum = 0;
  uint8_t i = 0;

  for (; len > 1; len -= 2)
  {
    sum += *data++;
    if (sum & 0x80000000)
      sum = (sum & 0xffff) + (sum >> 16);
  }

  if (len == 1)
  {
    *(uint8_t *)(&i) = *(uint8_t *)data;
    sum += i;
  }

  while (sum >> 16)
    sum = (sum & 0xffff) + (sum >> 16);

  return ((sum == 0xffff) ? sum : (~sum));
}

