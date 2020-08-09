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
    motor_left = motor_left>0 ? motor_left + 70 : motor_left - 70; //电机转差补偿，补偿1.5%
    motor_right = motor_right>0 ? motor_right - 70 : motor_right + 70;
    /*if(motor_left - motor_left_old >= 1000)
        motor_left = motor_left_old + 1000;
    else if(motor_left - motor_left_old <= -1000)
        motor_left = motor_left_old - 1000;
    if(motor_right - motor_right_old >= 1000)
        motor_right = motor_right_old + 1000;
    else if(motor_right - motor_right_old <= -1000)
        motor_right = motor_right_old - 1000;*/
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
    /*if(motor_left >= 0)
    {
        pwm_duty(PWM5_P00, (uint16)(10000 - motor_left));
        pwm_duty(PWM6_P01,  0);
    }
    else
    {
        pwm_duty(PWM5_P00, (uint16)(10000 + motor_left));
        pwm_duty(PWM6_P01,  10000);
    }
    if(motor_right >= 0)
    {
        pwm_duty(PWM2P_P22, (uint16)(10000 - motor_right));
        pwm_duty(PWM4P_P26,  10000);
    }
    else
    {
        pwm_duty(PWM2P_P22, (uint16)(10000 + motor_right));
        pwm_duty(PWM4P_P26,  0);
    }*/
}

void motor_stop()
{
    /*pwm_duty(PWM5_P00,  1000); //初始化PWM0  使用P00引脚  初始化频率为17Khz
    pwm_duty(PWM6_P01,  2500); //初始化PWM0  使用P01引脚  初始化频率为17Khz
    pwm_duty(PWM2P_P22, 1000); //初始化PWM2  使用P22引脚  初始化频率为17Khz
    pwm_duty(PWM4P_P26, 2500); //初始化PWM2  使用P26引脚  初始化频率为17Khz
    delay_ms(800);*/
    while (1)
    {
        pwm_duty(PWM5_P00,  1000); //初始化PWM0  使用P00引脚  初始化频率为17Khz
        pwm_duty(PWM6_P01,  1000); //初始化PWM0  使用P01引脚  初始化频率为17Khz
        pwm_duty(PWM2P_P22, 1000); //初始化PWM2  使用P22引脚  初始化频率为17Khz
        pwm_duty(PWM4P_P26, 1000); //初始化PWM2  使用P26引脚  初始化频率为17Khz
    }
}

/***************************
 * @breif   闭环停车
 * @param   void
 * @return  void
 * @note    入库后闭环刹车
 * @author  psq
 ***************************/
void motor_stop_plus()
{
    int16 motor_left, motor_right;  
    int16 forward_cnt = 0;
    int16 forward_flag = 1;
    while (1)
    {
        if(forward_flag == 1 && ++forward_cnt == 200)
        {
            forward_flag = 0;
            pwm_duty(PWM5_P00,  4000); //初始化PWM0  使用P00引脚  初始化频率为17Khz
            pwm_duty(PWM6_P01,  1000); //初始化PWM0  使用P01引脚  初始化频率为17Khz
            pwm_duty(PWM2P_P22, 4000); //初始化PWM2  使用P22引脚  初始化频率为17Khz
            pwm_duty(PWM4P_P26, 1000); //初始化PWM2  使用P26引脚  初始化频率为17Khz
            delay_ms(300);
        }
        car_info.speed = get_speed(5);
        motor_left = -MOTOR_STOP_P * car_info.speed.left;
        motor_right = -MOTOR_STOP_P * car_info.speed.right;
        if(car_info.speed.left < 50 && car_info.speed.left > -50)
            motor_left = 0;
        if(car_info.speed.right < 50 && car_info.speed.right > -50)
            motor_right = 0;
       
        if(motor_left > AMPLITUDE_LIMIT)
            motor_left = AMPLITUDE_LIMIT;
        else if(motor_left < -AMPLITUDE_LIMIT)
            motor_left = -AMPLITUDE_LIMIT;
        if(motor_right > AMPLITUDE_LIMIT)
            motor_right = AMPLITUDE_LIMIT;
        else if(motor_right < -AMPLITUDE_LIMIT)
            motor_right = -AMPLITUDE_LIMIT;
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
        delay_ms(5);
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
    delay_ms(5);
}
