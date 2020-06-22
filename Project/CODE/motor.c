#include "motor.h"

float Angle_Control_P;
float Angle_Control_D;

void motor_init(void)
{
	pwm_init(PWM5_P00, 10000, 0);   //初始化10kHz
	pwm_init(PWM7_P22, 10000, 0);
	pwm_init(PWM6_P01, 10000, 0);
	pwm_init(PWM4P_P26, 10000, 0);
}

/*
使用前给Angle_Control_P, Angle_Control_D赋值
*/

float AngleControl(float Car_Angle, float Car_W, float Angle_Set)   //控直立
{
	float Motor_AngleControl, Angle_Control;
    Angle_Control = Car_Angle - Angle_Set;  
    Motor_AngleControl=Angle_Control*Angle_Control_P + Car_W*Angle_Control_D;
	return Motor_AngleControl;
}

void motor_output(float Motor_AngleControl)
{
	float motor_left, motor_right;
	motor_left = Motor_AngleControl;
	motor_right = Motor_AngleControl;
	if(motor_left > 10000)
		motor_left = 10000;
	if(motor_left < -10000)
		motor_left = -10000;
	if(motor_right > 10000)
		motor_right = 10000;
	if(motor_right < -10000)
		motor_right = -10000;
	if(motor_right >= 0)
	{
		pwm_duty(PWM5_P00, (int)motor_right);
		pwm_duty(PWM6_P01, 0);	
	}
	else
	{
		pwm_duty(PWM5_P00, 0);
		pwm_duty(PWM6_P01, (int)motor_right);	
	}
	if(motor_left >= 0)
	{
		pwm_duty(PWM7_P22, (int)motor_left);
		pwm_duty(PWM4P_P26, 0);	
	}
	else
	{
		pwm_duty(PWM7_P22, 0);
		pwm_duty(PWM4P_P26, (int)motor_left);	
	}
}
