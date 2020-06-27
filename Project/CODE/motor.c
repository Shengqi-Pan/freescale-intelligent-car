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
    pwm_init(PWM5_P00, 10000, 0);   //初始化10kHz
    pwm_init(PWM7_P22, 10000, 0);
    pwm_init(PWM6_P01, 10000, 0);
    pwm_init(PWM4P_P26, 10000, 0);
}

void motor_output(float motor_angle_control, int16 motor_turn_control)
{
    float motor_left, motor_right;
    motor_left = motor_angle_control - motor_turn_control;
    motor_right = motor_angle_control + motor_turn_control;  
     
    
    // if (motor_angle_control > AMPLITUDE_LIMIT)
    //     motor_angle_control = AMPLITUDE_LIMIT;
    // if(motor_angle_control < -AMPLITUDE_LIMIT)
    //     motor_angle_control = -AMPLITUDE_LIMIT;
    
    if(motor_left > AMPLITUDE_LIMIT)
        motor_left = AMPLITUDE_LIMIT;
    else if(motor_left < -AMPLITUDE_LIMIT)
        motor_left = -AMPLITUDE_LIMIT;
    if(motor_right > AMPLITUDE_LIMIT)
        motor_right = AMPLITUDE_LIMIT;
    else if(motor_right < -AMPLITUDE_LIMIT)
        motor_right = -AMPLITUDE_LIMIT;
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
