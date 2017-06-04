#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#define HMC5883L_REGA   0x00
#define HMC5883L_REGB   0x01
#define HMC5883L_MODE   0x02
#define HMC5883L_HX_H   0x03
#define HMC5883L_HX_L   0x04 
#define HMC5883L_HZ_H   0x05
#define HMC5883L_HZ_L   0x06
#define HMC5883L_HY_H   0x07
#define HMC5883L_HY_L   0x08
#define HMC5883L_STATE  0x09
#define HMC5883L_IRA    0x0a    //读序列号使用的寄存器
#define HMC5883L_IRB    0x0b
#define HMC5883L_IRC    0x0c 

#define HMC5883L_Addr       0x3c       //device address

#define HMC5883L_DRDY_PIN      GPIO_Pin_1
#define HMC5883L_DRDY_PORT     GPIOC

#define HMC5883L_SENSITIVITY  (1090.0f)

/*---------------------* 
*   HMC5883 数据类型   * 
*----------------------*/
typedef struct
{
    int16_t  hx;
    int16_t  hy;
    int16_t  hz;
}HMC5883L_TYPE;


/*
*********************************************
*         FUNCTION PROTOTYPE
*********************************************
*/

/************Private*************/
void HMC5883L_WriteByte(uint8_t reg, uint8_t value);

uint8_t  HMC5883L_ReadByte(uint8_t reg);

uint8_t HMC5883L_BrustRead(uint8_t reg, uint8_t* p_buf, uint16_t length);

/************Public**************/
uint8_t HMC5883L_Init(void);

uint8_t HMC5883L_Read(HMC5883L_TYPE * pResult);

uint8_t HMC5883L_IsDataReady(void);

uint8_t HMC5883L_ConnectCheck(void);

void BSP_EXTI1_ISR_Handler(void);

#endif /*__HMC5883L_H__*/
