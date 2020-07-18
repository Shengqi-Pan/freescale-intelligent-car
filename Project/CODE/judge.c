/*******************************************
 * @file            judge.c
 * @note            判圆环，判坡等函数
 * @author          psq & btk
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#include "judge.h"

/***************************
 * @breif   判断圆环
 * @param   void
 * @return  判出圆环返回1，否则返回0
 * @note    
 ***************************/
uint8 is_ring()
{
    // static int16 ad3_ad4_diff;
    // static int16 ad3_ad4_diff_reg[4];
    // uint8 i;
    // ad3_ad4_diff = ad[2] - ad[3];
    // ad3_ad4_diff_reg[3] = ad3_ad4_diff;
    if(ad[0] > (500 + (car_info.angle - 27) * (car_info.angle<27 ? 4 : 3)) && ad[1] > (400 + (car_info.angle - 27) * (car_info.angle<27 ? 4 : 3))) //TODO: 由于电感原因，右环条件可能需要进一步调节
    {
        if(ad[2] > 2.5 * ad[3])
        {
            ring_dir = LEFT;
            return 1;
        }
    }
    if(ad[1] > (500 + (car_info.angle - 27) * (car_info.angle<27 ? 4 : 3)) && ad[0] > (400 + (car_info.angle - 27) * (car_info.angle<27 ? 4 : 3)))
    {
        if(ad[3] > 2.5 * ad[2])
        {
            ring_dir = RIGHT;
            return 1;
        }
    }
    return 0;
}

/***************************
 * @breif   判断前瞻是否到达圆环切点
 * @param   void
 * @return  判出切点返回1，否则返回0
 * @note    圆环1.0中使用了此函数，现在圆环2.0暂时用不到
 ***************************/
uint8 is_tangent()
{
    static int8 cnt = 0;
    static ad_last = 0;
    static down4_flag = 0;

    if(ring_dir == LEFT)
    {
        if(down4_flag == 0)
        {
            if(ad[2] < ad_last)
                --cnt;
            if(ad[2] > ad_last)
                cnt = 0;
            ad_last = ad[2];
            if(cnt <= -4)
                down4_flag = 1;
        }
        else if(down4_flag == 1)
        {
            if(ad[2] < ad_last)
                cnt = 0;
            if(ad[2] > ad_last)
                ++cnt;
            ad_last = ad[2];
            if(cnt >= 4)
            {
                down4_flag = 0;
                return 1;
            }
        }
    }
    else if(ring_dir == RIGHT)
    {
        if(down4_flag == 0)
        {
            if(ad[3] < ad_last)
                --cnt;
            if(ad[3] > ad_last)
                cnt = 0;
            ad_last = ad[3];
            if(cnt <= -4)
                down4_flag = 1;
        }
        else if(down4_flag == 1)
        {
            if(ad[3] < ad_last)
                cnt = 0;
            if(ad[3] > ad_last)
                ++cnt;
            ad_last = ad[3];
            if(cnt >= 4)
            {
                down4_flag = 0;
                return 1;
            }
        }
    }
    return 0;
}

/***************************
 * @breif   在判出圆环后通过编码器得到一段延迟距离
 * @param   void
 * @return  延迟结束返回1，否则返回0
 * @note    
 ***************************/
uint8 is_motor_tangent()
{
    if(ring_dir == RIGHT)
    {
        if(car_info.distance > 520 - (car_info.speed.average - 1700) / 5)  // 移动超过25cm
            return 1;
        else
            return 0;
    }
    else
    {
        if(car_info.distance > 520 - (car_info.speed.average - 1700) / 5)  // 移动超过10cm
            return 1;
        else
            return 0;
    }
        
}

/***************************
 * @breif   判断坡道
 * @param   void
 * @return  判出坡道返回1，否则返回0
 * @note    
 ***************************/
uint8 is_ramp()
{
    static int16 speed_reg[3] = {0,0,0};
    uint8 i;
    static uint8 ramp_flag = 0;
    if(car_info.speed.average > 1500)
        ramp_flag = 1;
    for(i=0;i<2;++i)
    {
        speed_reg[i] = speed_reg[i+1];
    }
    speed_reg[2] = car_info.speed.average;
    for(i=0;i<2;++i)
    {
        if(speed_reg[i] > 1000)
            return 0;
    }
    if(ramp_flag == 1)
    {
        ramp_flag = 0;
        return 1;
    }
    return 0; 
}
