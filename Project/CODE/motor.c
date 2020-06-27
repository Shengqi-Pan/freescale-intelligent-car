/*******************************************
 * @file            motor
 * @note            基于逐飞pwm的电机底层驱动
 * @author          btk
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/
#include "motor.h"

void motor_init(void)
{
    pwm_init(PWM5_P00, 30000, 0);   //初始化10kHz
    pwm_init(PWM7_P22, 30000, 0);
    pwm_init(PWM6_P01, 30000, 0);
    pwm_init(PWM4P_P26, 30000, 0);
}

void motor_output(float Motor_AngleControl)
{
    float motor_left, motor_right;
    if (Motor_AngleControl > AMPLITUDE_LIMIT)
        Motor_AngleControl = AMPLITUDE_LIMIT;
    if(Motor_AngleControl < -AMPLITUDE_LIMIT)
        Motor_AngleControl = -AMPLITUDE_LIMIT;
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
        pwm_duty(PWM6_P01, (int)(-motor_right));	
    }
    if(motor_left >= 0)
    {
        pwm_duty(PWM7_P22, (int)motor_left);
        pwm_duty(PWM4P_P26, 0);	
    }
    else
    {
        pwm_duty(PWM7_P22, 0);
        pwm_duty(PWM4P_P26, (int)(-motor_left));	
    }
}
