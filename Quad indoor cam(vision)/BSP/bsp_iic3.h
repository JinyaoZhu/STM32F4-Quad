#ifndef __BSP_IIC3_H__
#define	__BSP_IIC3_H__

/*----------Public-----------------*/
CPU_BOOLEAN 	 BSP_I2C3_Init(void);
CPU_BOOLEAN    BSP_I2C3_SingleWrite(CPU_INT08U RegAddr,CPU_INT08U DataToWrite,CPU_INT08U SlaveAddr);
CPU_BOOLEAN    BSP_I2C3_SingleWriteNoMemAddr(CPU_INT08U DataToWrite,CPU_INT08U SlaveAddr);
CPU_INT08U     BSP_I2C3_SingleRead(CPU_INT08U RegAddr,CPU_INT08U SlaveAddr);
CPU_INT16U     BSP_I2C3_SingleRead16NoMemAddr(CPU_INT08U SlaveAddr);
CPU_INT32U     BSP_I2C3_SingleRead24NoMemAddr(CPU_INT08U SlaveAddr);
CPU_BOOLEAN    BSP_I2C3_MultRead(CPU_INT08U RegAddr,CPU_INT08U SlaveAddr,CPU_INT08U *buf_ptr,CPU_INT16U nbr_of_byte);

/****************I2C ISR*******************/
void BSP_I2C3_EV_ISR(void);
void BSP_I2C3_ER_ISR(void);
#endif /* __BSP_IIC3_H__ */
