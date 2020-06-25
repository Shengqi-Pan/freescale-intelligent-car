/*******************************************
 * @file            control
 * @note            控制相关的函数，包括直立，速度，转向
 * @author          psq
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
