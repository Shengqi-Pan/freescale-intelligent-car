/*******************************************
 * @file            motor
 * @note            基于逐飞pwm的电机底层驱动
 * @author          btk
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/
#ifndef _MOTOR_H
#define _MOTOR_H

#include "zf_pwm.h"

#define AMPLITUDE_LIMIT 2000
extern float Angle_Control_P;
extern float Angle_Control_D;

void motor_init(void);
void motor_output(float Motor_AngleControl);

#endif