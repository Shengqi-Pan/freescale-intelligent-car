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
#include "zf_delay.h"
#include "encoder.h"
#include "headfile.h"

#define AMPLITUDE_LIMIT 8700
#define AMPLITUDE_LIMIT_MIN 1000
#define DEAD_TIME 40

#define MOTOR_STOP_P 4

void motor_init(void);
void motor_output(float motor_angle_control, int16 motor_turn_control);
void motor_stop(void);
void motor_stop_plus();
#endif