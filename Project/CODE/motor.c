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
    pwm_init(PWM5_P00,  17000, 0); //初始化PWM0  使用P00引脚  初始化频率为17Khz
	pwm_init(PWM6_P01,  17000, 0); //初始化PWM0  使用P01引脚  初始化频率为17Khz
	pwm_init(PWM2P_P22, 17000, 0); //初始化PWM2  使用P22引脚  初始化频率为17Khz
	pwm_init(PWM4P_P26, 17000, 0); //初始化PWM2  使用P26引脚  初始化频率为17Khz
}

void motor_output(float motor_angle_control, int16 motor_turn_control)
{
    float motor_left, motor_right;
    static float motor_left_old = 0, motor_right_old = 0;
    motor_left = motor_angle_control - motor_turn_control;
    motor_right = motor_angle_control + motor_turn_control;  
    // motor_left = motor_left>0 ? motor_left + 75 : motor_left - 75; //电机转差补偿，补偿1.5%
    // motor_right = motor_right>0 ? motor_right - 75 : motor_right + 75;
    if(motor_left - motor_left_old >= 2000)
        motor_left = motor_left_old + 2000;
    else if(motor_left - motor_left_old <= -2000)
        motor_left = motor_left_old - 2000;
    if(motor_right - motor_right_old >= 2000)
        motor_right = motor_right_old + 2000;
    else if(motor_right - motor_right_old <= -2000)
        motor_right = motor_right_old - 2000;
    motor_left_old = motor_left;
    motor_right_old = motor_right;
    if(motor_left > AMPLITUDE_LIMIT)
        motor_left = AMPLITUDE_LIMIT;
    else if(motor_left < -AMPLITUDE_LIMIT)
        motor_left = -AMPLITUDE_LIMIT;
    if(motor_right > AMPLITUDE_LIMIT)
        motor_right = AMPLITUDE_LIMIT;
    else if(motor_right < -AMPLITUDE_LIMIT)
        motor_right = -AMPLITUDE_LIMIT;
    /*if(motor_left < AMPLITUDE_LIMIT_MIN && motor_left >= 0)
        motor_left = AMPLITUDE_LIMIT_MIN;
    else if(motor_left > -AMPLITUDE_LIMIT_MIN && motor_left <= 0)
        motor_left = -AMPLITUDE_LIMIT_MIN;
    if(motor_right < AMPLITUDE_LIMIT_MIN && motor_right >= 0)
        motor_right = AMPLITUDE_LIMIT_MIN;
    else if(motor_right > -AMPLITUDE_LIMIT_MIN && motor_right <= 0)
        motor_right = -AMPLITUDE_LIMIT_MIN;*/
    test[2] = motor_left;
    test[3] = motor_right;
    if(motor_left > 0)
    {
        motor_left += DEAD_TIME;
    }
    else if(motor_left < 0)
    {
        motor_left -= DEAD_TIME;
    }
    if(motor_right > 0)
    {
        motor_right += DEAD_TIME;
    }
    else if(motor_right < 0)
    {
        motor_right -= DEAD_TIME;
    }
    if(motor_left >= 0)
    {
        pwm_duty(PWM5_P00, (int)motor_left + 1000); //右电机弱，补强2%的占空比
        pwm_duty(PWM6_P01, 1000);	
    }
    else
    {
        pwm_duty(PWM5_P00, 1000);
        pwm_duty(PWM6_P01, (int)(-motor_left) + 1000);	
    }
    if(motor_right >= 0)
    {
        pwm_duty(PWM2P_P22, (int)motor_right + 1000);
        pwm_duty(PWM4P_P26, 1000);	
    }
    else
    {
        pwm_duty(PWM2P_P22, 1000);
        pwm_duty(PWM4P_P26, (int)(-motor_right) + 1000);	
    }
  
}

void motor_stop()
{
    pwm_duty(PWM5_P00, 0);
    pwm_duty(PWM6_P01, 0);
    pwm_duty(PWM2P_P22, 0);
    pwm_duty(PWM4P_P26, 0);		
}
