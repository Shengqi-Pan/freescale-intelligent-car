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
    static float angle_bias = 0, angle_bias_last = 0;
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
    if (angle_bias - angle_bias_last > 0.1)
        angle_bias = angle_bias_last + 0.1;
    else if (angle_bias - angle_bias_last < -0.1)
        angle_bias = angle_bias_last - 0.1;
    angle_bias_last = angle_bias;

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

/***************************
 * @breif   转向控制函数
 * @param   void
 * @return  motor_turn 转向时的电机占空比
 * @note    返回转向时用的p、d参数
 * @author  btk
 ***************************/
int16 direction_control(void)
{
    int16 induc_ref[4] = {210, 250, 700, 700};
    int16 motor_turn;
    static int16 deviation_h_reg = 0;
    int16 deviation_h_dot = 0;
    int16 deviation_h;
    int16 sensor[4];
    float turn_p, turn_d;
    static int16 ad[4] = {0, 0, 0, 0};
    getl_once();
    ad[0] = (4*ad[0] + l_h_1)/5;
    ad[1] = (4*ad[1] + l_h_2)/5;
    ad[2] = (4*ad[2] + l_s_1)/5;
    ad[3] = (4*ad[3] + l_s_2)/5;
    sensor[0] = (int)(ad[0]*HENG_FACTOR/induc_ref[0]); // -angle_additional*低头ad变化/偏差度数
    sensor[1] = (int)(ad[1]*HENG_FACTOR/induc_ref[1]);
    sensor[2] = (int)(ad[2]*SHU_FACTOR/induc_ref[2]);
    sensor[3] = (int)(ad[3]*SHU_FACTOR/induc_ref[3]);
    data_conversion(ad[0], ad[1], sensor[0], sensor[1],virtual_scope_data);
    deviation_h = (sensor[0] - sensor[1]) * AMP_FACTOR / (sensor[0] + sensor[1]);
    //限幅
    if (deviation_h > 200)
    {
        deviation_h = 200;
    }
    else if (deviation_h < -200)
    {
        deviation_h = -200;
    }
    if(deviation_h - deviation_h_reg > 15)
        deviation_h = deviation_h_reg + 15;
    else if(deviation_h - deviation_h_reg < -15)
        deviation_h = deviation_h_reg - 15;
    //根据不同偏移量进行不同的偏移量求解
    if(deviation_h < 50 && deviation_h > -50)
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
    direction_pd_fuzzy(deviation_h, &turn_p, &turn_d);
    motor_turn = (int16)(turn_p * deviation_h + turn_d * deviation_h_dot* 20);
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

/***************************
 * @breif   模糊pd控制函数
 * @param   int16 deviation, float *turn_p, float *turn_d
 * @return  void
 * @note    返回转向时用的p、d参数
 * @author  btk
 ***************************/
void direction_pd_fuzzy(int16 deviation, float *p, float *d)
{
    int16 deviation_table[15] = {-200, -150, -100, -75, -50, 0, 50, 75, 100, 150, 200};
    float turn_p_table[15] = {10.5, 10.5 ,14.5, 14.5, 12, 9, 6, 4, 6, 9, 12, 14.5, 14.5, 10.5, 10.5};
    float turn_d_table[15] = {25, 25, 24, 20, 13, 9, 7, 3, 7, 9, 13, 20, 24, 25, 25};
    int8 i;
    int8 judge_flag = 1;
    if(deviation <= deviation_table[0])
    {
        *p = turn_p_table[0];
        *d = turn_d_table[0];
        judge_flag = 0;
    }
    if(deviation >= deviation_table[14])
    {
        *p = turn_p_table[14];
        *d = turn_p_table[14];
        judge_flag = 0;
    }
    if(judge_flag == 1)
    {
        for(i=0;i<14;i++)
        {
            if(deviation >= deviation_table[i] && deviation <= deviation_table[i+1])
            {
                *p = turn_p_table[i] + (deviation - deviation_table[i])*(turn_p_table[i+1] - turn_p_table[i])/(deviation_table[i+1] - deviation_table[i]);
                *d = turn_d_table[i] + (deviation - deviation_table[i])*(turn_d_table[i+1] - turn_d_table[i])/(deviation_table[i+1] - deviation_table[i]);
                break;
            }
        }
    }

    
}