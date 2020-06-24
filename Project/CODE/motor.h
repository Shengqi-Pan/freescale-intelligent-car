#ifndef _MOTOR_H
#define _MOTOR_H

#include "zf_pwm.h"
#define AMPLITUDE_LIMIT 2000
extern float Angle_Control_P;
extern float Angle_Control_D;

void motor_init(void);
float AngleControl(float Car_Angle, float Car_W, float Angle_Set);
void motor_output(float Motor_AngleControl);

#endif