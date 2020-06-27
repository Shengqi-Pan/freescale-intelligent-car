/*******************************************
 * @file            control
 * @note            控制相关的函数，包括直立，速度，转向
 * @author          psq&btk
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
    angle_control = car_angle - angle_set;  
    motor_angle_control = angle_control * ANGLE_CONTROL_P + car_w * ANGLE_CONTROL_D;
    return motor_angle_control;
}

/***************************
 * @breif   速度控制函数
 * @param   speed_real      左右轮转速真实值取平均
 * @param   speed_set       给定速度
 * @return  控速附加角
 * @note    
 * @author  psq
 ***************************/
float speed_control(int16 speed_real, int16 speed_set)
{
    static float angle_bias, angle_bias_last;
    int16 speed_deviation = speed_real - speed_set;
    /************直道控速************/
    // 速度慢了
    if(speed_deviation < -400)        angle_bias = 6;
    else if(speed_deviation < -200)   angle_bias = 4;  // 直道很慢 
    else if(speed_deviation < 0)      angle_bias = 2;
    // 速度快了
    else if(speed_deviation < 100)    angle_bias = 1; // TODO:这是因为给定的平衡角小于自然平衡角
    else if(speed_deviation < 200)    angle_bias = -1;
    else if(speed_deviation >= 300)   angle_bias = -2;  // 若超3m  附加角+2度

    /************限制bias变化防止突变************/
    if (angle_bias - angle_bias_last > 0.5)
        angle_bias = angle_bias_last + 0.5;
    else if (angle_bias - angle_bias_last < -0.5)
        angle_bias = angle_bias_last - 0.5;
    angle_bias_last = angle_bias;

    return angle_bias;
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

/***************************
 * @breif   起步函数
 * @param   void
 * @return  void
 * @note    起步时先控角度使车稳定，随后再往前冲
 * @author  psq
 ***************************/
void take_off(void)
{

}