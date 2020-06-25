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
    angle_control = angle_set - car_angle;  
    motor_angle_control = angle_control * ANGLE_CONTROL_P + car_w * ANGLE_CONTROL_D;
    return motor_angle_control;
}

/***************************
 * @breif   速度控制函数
 * @param   给定速度和当前速度
 * @return  控速附加角
 * @note    
 * @author  psq
 ***************************/
float speed_control(int speed_real, int speed_set)
{
    int speed_deviation = speed_real - speed_set;
    if(speed_deviation < -400)        return -14;
    else if(speed_deviation < -200)   return -10;  // 直道很慢 
    else if(speed_deviation < 0)      return -8;
    else if(speed_deviation < 100)    return -2;
    else if(speed_deviation < 200)    return 1;
    else if(speed_deviation >= 300)   return 2;  // 若超3m  附加角+2度
    return 0;
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
    /*for(i=0;i<4;i++)
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
    }*/
}

/*
输出转向控制对应电机的duty
*/
int16 direction_control(void)
{
    uint16 induc_ref[4] = {700, 700, 700, 700};
    int16 motor_turn;
    static uint16 deviation_h_reg = 0;
    int16 deviation_h_dot;
    int16 deviation_h;
    int16 sensor[4];
    int16 turn_p, turn_d;
    static uint16 ad[4] = {0, 0, 0, 0};
    getl_once();
    ad[0] = (4*ad[0] + l_h_1)/5;
    ad[1] = (4*ad[1] + l_h_2)/5;
    ad[2] = (4*ad[2] + l_s_1)/5;
    ad[3] = (4*ad[3] + l_s_2)/5;
    if (DIRECTION_ON == 1)
    {
        sensor[0] = (int)(ad[0]*HENG_FACTOR/induc_ref[0]);
        sensor[1] = (int)(ad[1]*HENG_FACTOR/induc_ref[1]);
        sensor[2] = (int)(ad[2]*SHU_FACTOR/induc_ref[2]);
        sensor[3] = (int)(ad[3]*SHU_FACTOR/induc_ref[3]);
        deviation_h = (int)(sensor[0]*1.3 - sensor[1]) * AMP_FACTOR / (sensor[0]*1.3 + sensor[1]);
        //限幅
        if (deviation_h > 600)
        {
            deviation_h = 600;
        }
        else if (deviation_h < -600)
        {
            deviation_h = -600;
        }
        //根据不同偏移量进行不同的偏移量求解
        if(deviation_h < 200 && deviation_h > -200)
         deviation_h_dot = (4*deviation_h_dot + deviation_h - deviation_h_reg)/5;
        else
         deviation_h_dot = (9*deviation_h_dot + deviation_h - deviation_h_reg)/10;
        deviation_h_reg = deviation_h;
        //偏差变化率限幅
        if (deviation_h_dot > 15)
            deviation_h_dot = 15;
        else if (deviation_h_dot < -15)
            deviation_h_dot = -15;
        //TODO: 模糊控制得到P和D
        turn_p = 3;
        turn_d = 8;
        motor_turn = turn_p * deviation_h + turn_d * deviation_h_dot;
        return motor_turn;
    }
    else
    {
        return 0;
    }
    
}