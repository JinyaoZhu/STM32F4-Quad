#include "motor.h"
#include "GlobalVariable.h"

/*
*****************************************************************************
*                           Motor_SetPWM()
* Description : 
*****************************************************************************
*/
void Motor_SetPWM(CTRL_DataType *control_data,MOTOR_DataType *motor,const sys_params_t sys_params)
{
	float u1,u2,u3,u4; /* force generate by each motor */
	
	float L1 = sys_params.length;
  float L2 = sqrt(2)*L1;
	
	/* PWM = F/3.9 */
	u1 = Constraint_f((control_data->throttle + control_data->mx/L1 - control_data->my/L1 - control_data->mz/L2)/4.0f/sys_params.factor_pwm2f,0,1);
	u2 = Constraint_f((control_data->throttle + control_data->mx/L1 + control_data->my/L1 + control_data->mz/L2)/4.0f/sys_params.factor_pwm2f,0,1);
	u3 = Constraint_f((control_data->throttle - control_data->mx/L1 + control_data->my/L1 - control_data->mz/L2)/4.0f/sys_params.factor_pwm2f,0,1);
	u4 = Constraint_f((control_data->throttle - control_data->mx/L1 - control_data->my/L1 + control_data->mz/L2)/4.0f/sys_params.factor_pwm2f,0,1);
	
	//g_TestTmpData1 = u1 + u2 +u3 +u4;
	
	motor->motor1 =  (uint16_t)(u1*TIM3_PWM_WIDTH + TIM3_PWM_LOWEST + 0.5f);
	motor->motor2 =  (uint16_t)(u2*TIM3_PWM_WIDTH + TIM3_PWM_LOWEST + 0.5f);
	motor->motor3 =  (uint16_t)(u3*TIM3_PWM_WIDTH + TIM3_PWM_LOWEST + 0.5f);
	motor->motor4 =  (uint16_t)(u4*TIM3_PWM_WIDTH + TIM3_PWM_LOWEST + 0.5f);
	
	//Motor_Smooth(motor);
  Motor_Constraint(motor);
	BSP_PWM_Set(motor->motor1,motor->motor2,motor->motor3,motor->motor4);
}

/*
*****************************************************************************
*                        Motor_MinThrust()
* Description : 
*****************************************************************************
*/
void Motor_MinThrust(MOTOR_DataType *motor)
{
	motor->motor1 =  TIM3_PWM_LOWEST;
	motor->motor2 =  TIM3_PWM_LOWEST;
	motor->motor3 =  TIM3_PWM_LOWEST;
	motor->motor4 =  TIM3_PWM_LOWEST;
	
	BSP_PWM_Set(motor->motor1,motor->motor2,motor->motor3,motor->motor4);
}


/*
*****************************************************************************
*                        Motor_MaxThrust()
* Description : 
*****************************************************************************
*/
void Motor_MaxThrust(MOTOR_DataType *motor)
{
	motor->motor1 =  TIM3_PWM_HIGHEST;
	motor->motor2 =  TIM3_PWM_HIGHEST;
	motor->motor3 =  TIM3_PWM_HIGHEST;
	motor->motor4 =  TIM3_PWM_HIGHEST;
	
	BSP_PWM_Set(motor->motor1,motor->motor2,motor->motor3,motor->motor4);
}

/*
*****************************************************************************
*                        MotorConstraint()
* Description : 
*****************************************************************************
*/
void Motor_Constraint(MOTOR_DataType *motor)
{
	motor->motor1 = Constraint_int(motor->motor1,TIM3_PWM_LOWEST,TIM3_PWM_HIGHEST);
	motor->motor2 = Constraint_int(motor->motor2,TIM3_PWM_LOWEST,TIM3_PWM_HIGHEST);
	motor->motor3 = Constraint_int(motor->motor3,TIM3_PWM_LOWEST,TIM3_PWM_HIGHEST);
	motor->motor4 = Constraint_int(motor->motor4,TIM3_PWM_LOWEST,TIM3_PWM_HIGHEST);
}

/*
****************************************************************************
*                            MotorSmooth()
* Description : compensate the motor mechanics effect
****************************************************************************
*/
void Motor_Smooth(MOTOR_DataType *motor)
{
  static uint16_t last_motor1 = TIM3_PWM_LOWEST;
	static uint16_t last_motor2 = TIM3_PWM_LOWEST;
	static uint16_t last_motor3 = TIM3_PWM_LOWEST;
	static uint16_t last_motor4 = TIM3_PWM_LOWEST;
	
	if((motor->motor1) > last_motor1)  motor->motor1 = (last_motor1 + 1*(motor->motor1))/2;
	else motor->motor1 = (motor->motor1) - (last_motor1 - (motor->motor1))*1;
	
	if((motor->motor2) > last_motor2)  motor->motor2 = (last_motor2 + 1*(motor->motor2))/2;
	else motor->motor2 = (motor->motor2) - (last_motor2 - (motor->motor2))*1;
	
	if((motor->motor3) > last_motor3)  motor->motor3 = (last_motor3 + 1*(motor->motor3))/2;
	else motor->motor3 = (motor->motor3) - (last_motor3 - (motor->motor3))*1;
	
	if((motor->motor4) > last_motor4)  motor->motor4 = (last_motor4 + 1*(motor->motor4))/2;
	else motor->motor4 = (motor->motor4) - (last_motor4 - (motor->motor4))*1;
	
	last_motor1 = (motor->motor1);
	last_motor2 = (motor->motor2);
	last_motor3 = (motor->motor3);
	last_motor4 = (motor->motor4);
}
