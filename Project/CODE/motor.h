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
#include "car_info.h"

#define AMPLITUDE_LIMIT 9000
#define AMPLITUDE_LIMIT_MIN 400
#define DEAD_TIME 40
#define TURN_LIMIT 2500

void motor_init(void);
void motor_output(float motor_angle_control, int16 motor_turn_control);
void motor_stop(void);
#endif