/*******************************************
 * @file            control
 * @note            控制相关的函数，包括直立，速度，转向
 * @author          psq, vc
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#include "control.h"
int16 ad_test[4] = {0, 0, 0, 0};
/*
使用前给ANGLE_CONTROL_P, ANGLE_CONTROL_D赋值
输入kalman滤波后的车身角 车身角速度 设定角
输出控制电机占空比的浮点数
*/
float angle_control(float car_angle, float car_w, float angle_set)   //控直立
{
    float motor_angle_control, angle_control;
    angle_control = angle_set - car_angle;  
    motor_angle_control = angle_control * ANGLE_CONTROL_P + car_w * ANGLE_CONTROL_D;
    return motor_angle_control;
}
/*
观察电磁读取的ad值，便于后面处理，不出现在主函数中
*/

void induc_test(void)
{
    int i,j; 
    for(i=0;i<4;i++)
        ad_test[i] = 0;
    for(j=0;j<3;j++)
    {
        getl_once();
        ad_test[0] += l_h_1;
        ad_test[1] += l_h_2;   
        ad_test[2] += l_s_1; 
        ad_test[3] += l_s_2;
    }
    for(i=0;i<4;i++)
    {
        ad_test[i]=ad_test[i]/3;  
        // if(ad_test[i]>Induc_Ref[2*i])
        // {
        //   Induc_Ref[2*i]=ad_test[i];
        // }
    }
}

/*
输出转向控制对应电机的浮点数
*/
float direction_control(void)
{
    int i;
    static uint16 deviation_h_reg = 0;
    uint16 deviation_h_dot;
    uint16 deviation_h;
    uint16 ad[4] = {0, 0, 0, 0};
    for(i=0;i<3;i++)
    {
        getl_once();
        ad[0] += l_h_1;
        ad[1] += l_h_2;
        ad[2] += l_s_1;
        ad[3] += l_s_2;
    }
    for(i=0;i<4;i++)
        ad[i] = ad[i]/3;
    if (DIRECTION_ON == 1)
    {
        //TODO: 归一化处理
        deviation_h = (l_h_1 - l_h_2)/(l_h_1 + l_h_2);
        //TODO: 限幅
        //TODO: 根据不同偏移量进行不同的偏移量求解
        deviation_h_dot = deviation_h - deviation_h_reg;
        deviation_h_reg = deviation_h;
        //TODO: 偏差变化率限幅
        //TODO: 模糊控制得到P和D
    }
    return -1;
}