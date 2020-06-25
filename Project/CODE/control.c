/*******************************************
 * @file            control
 * @note            控制相关的函数，包括直立，速度，转向
 * @author          psq&btk
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#include "control.h"

/*
使用前给ANGLE_CONTROL_P, ANGLE_CONTROL_D赋值
*/
float angle_control(float car_angle, float car_w, float angle_set)   //控直立
{
    float motor_angle_control, angle_control;
    angle_control = angle_set - car_angle;  
    motor_angle_control = angle_control * ANGLE_CONTROL_P + car_w * ANGLE_CONTROL_D;
    return motor_angle_control;
}

/***************************
 * @breif   速度控制函数
 * @param   给定速度和当前速度
 * @return  控速附加角
 * @note    
 * @author  psq
 ***************************/
float speed_control(int speed_real, int speed_set)
{
    int speed_deviation = speed_real - speed_set;
    if(speed_deviation < -400)        return -14;
    else if(speed_deviation < -200)   return -10;  // 直道很慢 
    else if(speed_deviation < 0)      return -8;
    else if(speed_deviation < 100)    return -2;
    else if(speed_deviation < 200)    return 1;
    else if(speed_deviation >= 300)   return 2;  // 若超3m  附加角+2度
}