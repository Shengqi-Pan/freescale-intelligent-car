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
    int16 speed_deviation = speed_real - speed_set;  // 实际速度和设定速度差值
    switch (car_info.state)
    {
        case STRAIGHT_AHEAD:
        /************直道控速************/
            // 速度慢了
            if(speed_deviation < -400)        angle_bias = 6;
            else if(speed_deviation < -200)   angle_bias = 4;  // 直道很慢 
            else if(speed_deviation < 0)      angle_bias = 2;
            // 速度快了
            else if(speed_deviation < 100)    angle_bias = 1; // TODO:这是因为给定的平衡角小于自然平衡角
            else if(speed_deviation < 200)    angle_bias = -1;
            else if(speed_deviation >= 300)   angle_bias = -2;  // 若超3m  附加角+2度
            break;
        case INTO_TURN:
        /************出/入弯控速************/
            // TODO:参数待调
            if(car_info.speed.average > 400)    // 连续小弯加速
                angle_bias = 2;
            else
                angle_bias = 8;
            break;
        case IN_TURN:
        /************弯中控速************/
            // TODO:参数待调
            if(car_info.speed.average > 400)    // 连续小弯加速
                angle_bias = 3;
            else
                angle_bias = 10;
            break;
        case RAMP_UP:
        /************上坡************/
            // TODO:不知道wwt这里这么调节的目的
            if(car_info.speed.average < 500)
                angle_bias = 1;
            else
                angle_bias = 3;
            break;
        case RAMP_DOWN:
        /************下坡************/
            // TODO:不知道wwt这里这么调节的目的
            angle_bias = 5;
            break;
        default:
            break;
    }

    /************限制bias变化防止突变************/
    if (angle_bias - angle_bias_last > 0.5)
        angle_bias = angle_bias_last + 0.5;
    else if (angle_bias - angle_bias_last < -0.5)
        angle_bias = angle_bias_last - 0.5;
    angle_bias_last = angle_bias;
    // data_conversion(angle_bias, 0,
    //                 0, 0,
    //                 virtual_scope_data);

    return angle_bias;
}

/*
观察电磁读取的ad值，便于后面处理，不出现在主函数中
*/
void induc_test(void)
{ 
    getl_once();
    ad_test[0] = (4*ad_test[0] + l_h_1)/5;
    ad_test[1] = (4*ad_test[1] + l_h_2)/5;
    ad_test[2] = (4*ad_test[2] + l_s_1)/5;
    ad_test[3] = (4*ad_test[3] + l_s_2)/5;
}

/*
输出转向控制对应电机的duty
*/
int16 direction_control(void)
{
    int16 induc_ref[4] = {200, 240, 700, 700};
    int16 motor_turn;
    static int16 deviation_h_reg = 0;
    int16 deviation_h_dot = 0;
    int16 deviation_h;
    int16 sensor[4];
    int16 turn_p, turn_d;
    static int16 ad[4] = {0, 0, 0, 0};
    getl_once();
    ad[0] = (4*ad[0] + l_h_1)/5;
    ad[1] = (4*ad[1] + l_h_2)/5;
    ad[2] = (4*ad[2] + l_s_1)/5;
    ad[3] = (4*ad[3] + l_s_2)/5;
    sensor[0] = (int)(ad[0]*HENG_FACTOR/induc_ref[0]);
    sensor[1] = (int)(ad[1]*HENG_FACTOR/induc_ref[1]);
    sensor[2] = (int)(ad[2]*SHU_FACTOR/induc_ref[2]);
    sensor[3] = (int)(ad[3]*SHU_FACTOR/induc_ref[3]);
    //data_conversion(ad[0], ad[1], sensor[0], sensor[1],virtual_scope_data);
    deviation_h = (sensor[0] - sensor[1]) * AMP_FACTOR / (sensor[0] + sensor[1]);
    //限幅
    if (deviation_h > 100)
    {
        deviation_h = 100;
    }
    else if (deviation_h < -100)
    {
        deviation_h = -100;
    }
    //根据不同偏移量进行不同的偏移量求解
    if(deviation_h < 25 && deviation_h > -25)
        deviation_h_dot = (4*deviation_h_dot + deviation_h - deviation_h_reg)/5;
    else
        deviation_h_dot = (9*deviation_h_dot + deviation_h - deviation_h_reg)/10;
    deviation_h_reg = deviation_h;
    //偏差变化率限幅
    if (deviation_h_dot > 10)
        deviation_h_dot = 10;
    else if (deviation_h_dot < -10)
        deviation_h_dot = -10;
    //TODO: 模糊控制得到P和D
    turn_p = 80;
    turn_d = 20;
    motor_turn = turn_p * deviation_h + turn_d * deviation_h_dot;
    return motor_turn;
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