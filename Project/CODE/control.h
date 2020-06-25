/*******************************************
 * @file            control
 * @note            控制相关的函数，包括直立，速度，转向
 * @author          psq
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#ifndef _CONTROL_H
#define _CONTROL_H


#define ANGLE_CONTROL_P 2000
#define ANGLE_CONTROL_D 50

float angle_control(float car_angle, float car_w, float angle_set);

#endif