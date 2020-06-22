#include "zf_pwm.h"

extern float Angle_Control_P;
extern float Angle_Control_D;

void motor_init(void);
float AngleControl(float Car_Angle, float Car_W, float Angle_Set);
void motor_output(float Motor_AngleControl);