#include "cordic.h"
#include "math.h"

#define INV_2PI    (0.159154943092f)
#define _2PI       (6.2831853f)

#define QUAD1 (1)
#define QUAD2 (2)
#define QUAD3 (3)
#define QUAD4 (4)

#define NBITS   (25) /* should below 29 */

static int32_t ArcTan[NBITS] = {0}; /* Rad * CordicBase */
static int32_t xInit;
static uint32_t CordicBase;
static float ConvertToReal;     /* converte sin and cos value to virtual unit */
static float ConvertToRad;
static float ConvertToCAU;      /* convert rad to Cordic angle unit */
static float MaxCAU;  /* Max Cordic angle unit */
static uint32_t Quad2Bondary;
static uint32_t Quad3Bondary;
static uint32_t Quad4Bondary;


/*
********************************************************
*                  CORDIC_Init
* Description :
********************************************************
*/

void CORDIC_Init(void)
{
  int16_t i;    /* to index ArcTan[] */
  double f;     /* to calc initial x projection */
  long powr; /* powers of 2 up to 2^(2*(NBITS - 1)) */

  CordicBase = (uint32_t)1u << NBITS;
  Quad2Bondary = CordicBase << 1u;
  Quad3Bondary = Quad2Bondary + CordicBase;
  Quad4Bondary = Quad3Bondary + CordicBase;

  ConvertToReal = 1.0f / CordicBase;
  MaxCAU      = CordicBase << 2u;
  ConvertToRad = PI / ((float)CordicBase * 2.0f);
  ConvertToCAU = MaxCAU / (2.0f * PI);
  powr = 1;
  for (i = 0; i < NBITS ; i++)
  {
    ArcTan[i] = (int32_t)(atan(1.0f / powr) / (PI / 2.0f) * CordicBase + 0.5f);
    powr <<= 1u;
  }

  f = 1.0f;
  powr = 1;
  for (i = 0; i < NBITS; i++)
  {
    f = (f * (powr + 1)) / powr;
    powr <<= 2u;
  }
  f = 1.0f / sqrt(f);
  xInit = (int32_t)(CordicBase * f + 0.5f);
}

/*
**************************************************************
*                     CORDIC_SinCos
* Description :
**************************************************************
*/
void CORDIC_SinCos(uint32_t theta, int32_t *sin, int32_t *cos)
{
  int32_t quadrant;    /* quadrant of incoming angle */
  int32_t z;          /* incoming angle move to 1st quadrant */
  uint32_t i;          /* to index rotation */
  int32_t x, y;       /* projections onto axes */
  int32_t x1, y1;     /* peojections of rotated vector */
     
  if (theta < CordicBase)
  {
    quadrant = QUAD1;
    z = (int32_t)theta;
  }
  else if (theta < Quad2Bondary)
  {
    quadrant = QUAD2;
    z = (int32_t)(Quad2Bondary - theta);
  }
  else if (theta < Quad3Bondary)
  {
    quadrant = QUAD3;
    z = (int32_t)(theta - Quad2Bondary);
  }
  else
  {
    quadrant = QUAD4;
    z = (int32_t)(Quad4Bondary - theta);
  }
  
  x = xInit;
  y = 0;

  /* Negate z, so same rotations taking angle z to  0 */
  z = -z;

  for (i = 0; i < NBITS; i++)
  {
    if (z < 0)
    { 
      z += ArcTan[i]; 
      y1 = y + (x >> i);
      x1 = x - (y >> i);
    }
    else
    {
      z -= ArcTan[i];
      y1 = y - (x >> i);
      x1 = x + (y >> i);
    }
    
    x = x1;
    y = y1;
  }
 
  *cos = (quadrant == QUAD1 || quadrant == QUAD4) ? x : -x;
  *sin = (quadrant == QUAD1 || quadrant == QUAD2) ? y : -y;
}

/*
*****************************************************
*                  CORDIC_Atan2()
* Description : +- 0.000001 rad
*****************************************************
*/
float CORDIC_Atan2(float y_val, float x_val)
{
  int32_t quadrant = QUAD1;
  int32_t x = 0;
  int32_t y = 0;
  int32_t i = 0;
  int32_t x1, y1;
  int32_t angleSum = 0;

  if (x_val == 0.0f)
    x_val += 0.000000000001f;
  if (y_val == 0.0f)
    y_val += 0.000000000001f;

  x_val *= 256.0f;
  y_val *= 256.0f;

  if ( (x_val > 0.0f) && (y_val >= 0.0f))
  {
    x = (int32_t)(x_val + 0.5f);
    y = (int32_t)(y_val + 0.5f);
    quadrant = QUAD1;
  }
  else if ((x_val <= 0.0f) && (y_val > 0.0f))
  {
    x = (int32_t)(-x_val + 0.5f);
    y = (int32_t)(y_val + 0.5f);
    quadrant = QUAD2;
  }
  else if ((x_val < 0.0f) && (y_val <= 0.0f))
  {
    x = (int32_t)(-x_val + 0.5f);
    y = (int32_t)(-y_val + 0.5f);
    quadrant = QUAD3;
  }
  else
  {
    x = (int32_t)(x_val + 0.5f);
    y = (int32_t)(-y_val + 0.5f);
    quadrant = QUAD4;
  }

  for (i = 0; i < NBITS; i++)
  {
    if (y > 0)
    {
      x1 = x + (y >> i);
      y1 = y - (x >> i);
      angleSum += ArcTan[i];
    }
    else
    {
      x1 = x - (y >> i);
      y1 = y + (x >> i);
      angleSum -= ArcTan[i];
    }

    x = x1;
    y = y1;
  }
  switch (quadrant)
  {
    case QUAD1:
      break;
    case QUAD2:
      angleSum = (int32_t)(0.5f * MaxCAU) - angleSum;
      break;
    case QUAD3:
      angleSum = -(int32_t)(0.5f * MaxCAU) + angleSum;
      break;
    case QUAD4:
      angleSum = -angleSum;
      break;
    default : break;
  }
  return (angleSum * ConvertToRad);
}

/*
************************************************
*               fast_sin()
* Description :  apprximation
************************************************
*/

float fast_sin(float x)
{
  int32_t cos, sin;
  uint32_t theta;

  while (x < 0.0f)
  {
    x += _2PI;
  }

  while (x > _2PI)
  {
    x -= _2PI;
  }

  theta = (uint32_t)(x * ConvertToCAU);
  
  CORDIC_SinCos(theta, &sin, &cos);

  return (float)(sin * ConvertToReal);
}


/*
************************************************
*               fast_cos()
* Description :  apprximation
************************************************
*/
float fast_cos(float x)
{
  int32_t cos, sin;
  uint32_t theta;

  while (x < 0.0f)
  {
    x += _2PI;
  }

  while (x > _2PI)
  {
    x -= _2PI;
  }

  theta = (uint32_t)(x * ConvertToCAU + 0.5f);

  CORDIC_SinCos(theta, &sin, &cos);

  return (float)(cos * ConvertToReal);
}


void fast_SinCos(float x, float *sin, float *cos)
{
  int32_t cos_tmp, sin_tmp;
  uint32_t theta;

  while (x < 0.0f)
  {
    x += _2PI;
  }

  while (x > _2PI)
  {
    x -= _2PI;
  }

  theta = (uint32_t)(x * ConvertToCAU + 0.5f);

  CORDIC_SinCos(theta, &sin_tmp, &cos_tmp);

  *sin = (float)(sin_tmp * ConvertToReal);
  *cos = (float)(cos_tmp * ConvertToReal);
}

float fast_atan2(float y,float x)
{
  return CORDIC_Atan2(y,x);
}

