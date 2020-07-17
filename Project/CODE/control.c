/*******************************************
 * @file            control
 * @note            控制相关的函数，包括直立，速度，转向
 * @author          psq&btk
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#include "control.h"
int16 ad_test[4] = {0, 0, 0, 0};

/***************************
 * @breif   角度控制函数
 * @param   car_angle, car_w      卡尔曼滤波得到的角度和角速度
 * @param   angle_set             给角度
 * @return  控直立的电机占空比
 * @note    
 * @author  btk
 ***************************/
float angle_control(float car_angle, float car_w, float angle_set)   //控直立
{
    static float motor_angle_control, angle_control;
    angle_control = car_angle - angle_set;  
    if(car_info.state == TAKE_OFF && take_off_state == STAND_UP)         //起步时p，d应用独立参数
        motor_angle_control = angle_control * ANGLE_CONTROL_P_BEGIN + car_w * ANGLE_CONTROL_D_BEGIN;
    else if(car_info.state == RAMP_UP || car_info.state == RAMP_DOWN)       //过坡时减p加d
        motor_angle_control = angle_control * (ANGLE_CONTROL_P - 300) + car_w * (ANGLE_CONTROL_D + 8);
    else
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
    static int16 speed_deviation, speed_deviation_integrate;
    static int8 integrate_discard_cnt;          //积分项定时清0
    speed_deviation = speed_real - speed_set;  // 实际速度和设定速度差值
    if(++integrate_discard_cnt == 20)           //3ms清一次0
    {
        integrate_discard_cnt = 0;
        speed_deviation_integrate = 0;
    }
    speed_deviation_integrate += speed_deviation;   //积分项累加
    switch (car_info.state)
    {
        case TAKE_OFF: case STRAIGHT_AHEAD: case RING: case INTO_TURN: case IN_TURN:
        /************直道控速************/
            if(speed_real < 1200)
            {
                angle_bias = -speed_deviation * SPEED_CONTROL_P;
            }
            else
            {
                angle_bias = -(speed_deviation * SPEED_CONTROL_P + speed_deviation_integrate * SPEED_CONTROL_I);
            }
            break;
        // case INTO_TURN:
        // /************出/入弯控速************/
        //     // TODO:参数待调
        //     if(car_info.speed.average > 400)    // 连续小弯加速
        //         angle_bias = 2;
        //     else
        //         angle_bias = 8;
        //     break;
        // case IN_TURN:
        // /************弯中控速************/
        //     // TODO:参数待调
        //     if(car_info.speed.average > 400)    // 连续小弯加速
        //         angle_bias = 3;
        //     else
        //         angle_bias = 10;
        //     break;
        case RAMP_UP:
        /************上坡************/
            // if(speed_real < 1200)
            // {
            //     angle_bias = -speed_deviation * SPEED_CONTROL_P;
            // }
            // else
            // {
            //     angle_bias = -(speed_deviation * SPEED_CONTROL_P + speed_deviation_integrate * SPEED_CONTROL_I);
            // }
            return -8;
            break;
        case RAMP_DOWN:
        /************下坡************/
            // if(speed_real < 1200)
            // {
            //     angle_bias = -speed_deviation * SPEED_CONTROL_P;
            // }
            // else
            // {
            //     angle_bias = -(speed_deviation * SPEED_CONTROL_P + speed_deviation_integrate * SPEED_CONTROL_I);
            // }
            return -20;
            break;
        default:
            return 0;
            break;
    }

    /************限制bias变化防止突变************/
    if (angle_bias - angle_bias_last > 0.1)
        angle_bias = angle_bias_last + 0.1;
    else if (angle_bias - angle_bias_last < -0.1)
        angle_bias = angle_bias_last - 0.1;
    /*if(car_info.state == TAKE_OFF)              //起步限制偏差角，防止起步过猛
    {
        angle_bias = angle_bias>12 ? 12 : angle_bias;
        angle_bias = angle_bias<-12 ? -12 : angle_bias;
    }*/
    if(angle_bias > 8)
        angle_bias = 8;
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
    int16 motor_turn;
    static float deviation_h_reg = 0;   //存储上一次的电感偏差值，便于计算差值即deviation_h_dot
    static float deviation_l_reg = 0;
    static float deviation_h_dot = 0;   //横电感偏差变化率，应用在pd控制中消除振荡快速稳定
    static float deviation_l_dot = 0;
    static float deviation_h;           //横电感偏差值
    static float deviation_l;           //竖电感偏差值
    static float turn_p, turn_d;
    if(ring_state == RING_INTO && ring_dir == LEFT) //左环右环进行不同的归一化
    {
        induc_ref[2] = 65;
        induc_ref[3] = 15;
    }
    else if(ring_state == RING_INTO && ring_dir == RIGHT)
    {
        induc_ref[2] = 100;
        induc_ref[3] = 120;
    }
    getl_once();     //读一次电感值
    ad[0] = (4*ad[0] + l_h_1)/5;        //读取滤波
    ad[1] = (4*ad[1] + l_h_2)/5;
    ad[2] = (4*ad[2] + l_s_1)/5;
    ad[3] = (4*ad[3] + l_s_2)/5;
    if(car_info.state == TAKE_OFF && take_off_state == TURN_LEFT)
        return 1300;
    else if(car_info.state == TAKE_OFF && take_off_state == TURN_RIGHT)
        return -1300;
    else if(car_info.state == TAKE_OFF)
        return 0;
    sensor[0] = (int)(ad[0]*HENG_FACTOR/induc_ref[0]); // -angle_additional*低头ad变化/偏差度数 进行补偿 //归一化，小心溢出，factor取值不溢出越大越好
    sensor[1] = (int)(ad[1]*HENG_FACTOR/induc_ref[1]);
    sensor[2] = (int)(ad[2]*SHU_FACTOR/induc_ref[2]);
    sensor[3] = (int)(ad[3]*SHU_FACTOR/induc_ref[3]);
    if(ring_state != RING_INTO)         //横电感进行转弯
    {
        deviation_h = (sensor[0] - sensor[1]) * AMP_FACTOR / (sensor[0] + sensor[1]); //将其放大统一处理
        deviation_l_reg = 0;
        deviation_l_dot = 0;
        //限幅
        if(ad[0]<20 || ad[1]<20 && car_info.state != TAKE_OFF) //意外情况电机抱死
        {
            motor_stop();
            while(1);
        }
        deviation_h = deviation_h / (1.5 * car_info.angle / 32 - 0.27);
        if (deviation_h > 220)      //限幅
        {
            deviation_h = 220;
        }
        else if (deviation_h < -220)
        {
            deviation_h = -220;
        }
        if(deviation_h - deviation_h_reg > 20)        //变化率限幅
            deviation_h = deviation_h_reg + 20;
        else if(deviation_h - deviation_h_reg < -20)
            deviation_h = deviation_h_reg - 20;
        test[0] = deviation_h;          //test为全局数组，定义在car_info.h中，用于示波器调试
        //根据不同偏移量进行不同的偏移量求解
        if(deviation_h < 80 && deviation_h > -80)       //偏差大时滤波重一些
            deviation_h_dot = (4*deviation_h_dot + deviation_h - deviation_h_reg)/5.0;
        else
            deviation_h_dot = (9*deviation_h_dot + deviation_h - deviation_h_reg)/10.0;
        deviation_h_reg = deviation_h;
        //偏差变化率限幅
        if (deviation_h_dot > 20)
            deviation_h_dot = 20;
        else if (deviation_h_dot < -20)
            deviation_h_dot = -20;
        //模糊控制得到P和D
        // if(ring_state == RING_IN)   //使圆环更加圆滑
        //     deviation_h = 0.6 * deviation_h;
        // else if(ring_state == RING_OUT)
        //     deviation_h = 0.8 * deviation_h;
        if(car_info.state == RAMP_UP)
            deviation_h = 0.3 * deviation_h;
        direction_pd_fuzzy(deviation_h, &turn_p, &turn_d);  //模糊控制得到p，d
        motor_turn = (int16)(turn_p * deviation_h  + turn_d * deviation_h_dot * 1.6);
        return motor_turn;
    }
    else        //竖电感入环
    {
        deviation_h_reg = 0;
        deviation_h_dot = 0;
        deviation_l = (sensor[2] - sensor[3]) * AMP_FACTOR / (sensor[2] + sensor[3]);
        //限幅
        if(ad[0] < 20 && ad[1] < 20)
        {
            motor_stop();
            while(1);
        }
        if (deviation_l > 220)
        {
            deviation_l = 220;
        }
        else if (deviation_l < -220)
        {
            deviation_l = -220;
        }
        /*if(deviation_l - deviation_l_reg > 15)
            deviation_l = deviation_l_reg + 15;
        else if(deviation_l - deviation_l_reg < -15)
            deviation_l = deviation_l_reg - 15;*/
        //根据不同偏移量进行不同的偏移量求解
        if(deviation_l < 90 && deviation_l > -90)
            deviation_l_dot = (4*deviation_l_dot + deviation_l - deviation_l_reg)/5;
        else
            deviation_l_dot = (9*deviation_l_dot + deviation_l - deviation_l_reg)/10;
        deviation_l_reg = deviation_l;
        //偏差变化率限幅
        if (deviation_l_dot > 20)
            deviation_l_dot = 20;
        else if (deviation_l_dot < -20)
            deviation_l_dot = -20;
        direction_pd_fuzzy(deviation_l * 0.4, &turn_p, &turn_d);  //模糊控制得到p，d
        motor_turn = (int16)(turn_p * deviation_l  + turn_d * deviation_l_dot);
        if(ring_dir == LEFT)        //左环不向右转，右环同理
            motor_turn = motor_turn>0 ? motor_turn : 0;
        else if(ring_dir == RIGHT)
            motor_turn = motor_turn<0 ? motor_turn : 0;
        return motor_turn;
    }
    
    
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
    /*static int16 deviation_table[13] = {-110, -90, -70, -55, -35, -20, 0, 20, 35, 55, 70, 90, 110};    //注意分割，转弯时尽量控制在60以内
    static float turn_p_table[13] =     {9,     9 ,  11,  10,   9,   8,  6, 8,  9,  10, 11, 9,  9};
    static float turn_d_table[13] = {750, 700, 630, 550, 400, 280, 170, 280, 400, 550, 630, 700, 750};*/
    static int16 deviation_table[15] = {-220, -180, -130, -105, -70, -50, -25, 0, 25, 50, 70, 105, 130, 180, 220};    //注意分割，转弯时尽量控制在70以内
    static float turn_p_table[15] =     {  5,   6,     7,  7.5,    8,   7, 6.5,  5, 6.5, 7,  8, 7.5,  7,   6,  5};
    static float turn_d_table[15] = {450, 400, 350, 315, 275, 200, 150, 90, 150, 200, 275, 315, 350, 400, 450};
    int8 i;
    if(deviation <= deviation_table[0])
    {
        *p = turn_p_table[0];
        *d = turn_d_table[0];
    }
    else if(deviation >= deviation_table[14])
    {
        *p = turn_p_table[14];
        *d = turn_p_table[14];
    }
    else
    {
        for(i=0;i<14;i++) //模糊控制特殊情况，分段pd，不需要了解完全的模糊1理论
        {
            if(deviation >= deviation_table[i] && deviation <= deviation_table[i+1])
            {
                *p = turn_p_table[i] + (deviation - deviation_table[i])*(turn_p_table[i+1] - turn_p_table[i])/(deviation_table[i+1] - deviation_table[i]);
                *d = turn_d_table[i] + (deviation - deviation_table[i])*(turn_d_table[i+1] - turn_d_table[i])/(deviation_table[i+1] - deviation_table[i]);
                break;
            }
        }
    }
    if(car_info.speed.average > 2000) //速度大时过弯拉大d
    {
        if(car_info.speed.left_right_diff >= 800)  //高偏差
        {
            if(car_info.speed.average - 2000 <= 400)
            {
                *d *= 1.5 + (car_info.speed.average - 2000) / 400; 
            }
            else if(car_info.speed.average - 2000 > 400)
            {
                *d *= 2; 
            }
        }
        else if(car_info.speed.left_right_diff >= 500)  //中偏差
        {
            if(car_info.speed.average - 2000 <= 400)
            {
                *d *= 1.3 + (car_info.speed.average - 2000) / 400; 
            }
            else if(car_info.speed.average - 2000 > 400)
            {
                *d *= 1.8; 
            }
        }
        else if(car_info.speed.left_right_diff >= 300) //低偏差
        {
            if(car_info.speed.average - 2000 <= 400)
            {
                *d *= 1.1 + (car_info.speed.average - 2000) / 400; 
            }
            else if(car_info.speed.average - 2000 > 400)
            {
                *d *= 1.6; 
            }
        }
    }
    if(ring_state == RING_OUT)  //防止出环过调
    {
        *d = *d * 3;
    }
}