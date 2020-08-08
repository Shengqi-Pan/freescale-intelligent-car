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
    static int16 ad3_ad4_diff;
    static int16 ad3_ad4_diff_reg[4] = {0,0,0,0};
    uint8 i;
    ad3_ad4_diff = ad[2] - ad[3];
    ad3_ad4_diff_reg[3] = ad3_ad4_diff;
    /*if(ad[0] > (550 + (car_info.angle - 9) * (car_info.angle<9 ? 10 : 3)) && ad[1] > (420 + (car_info.angle - 9) * (car_info.angle<9 ? 10 : 3))) //TODO: 由于电感原因，右环条件可能需要进一步调节
    {
        if(ad[2] > ad[3] && ad[2] > 160 + (car_info.angle - 9) * (car_info.angle<9 ? 10 : 3))
        {
            ring_dir = LEFT;
            return 1;
        }
    }
    if(ad[1] > (550 + (car_info.angle - 9) * (car_info.angle<9 ? 10 : 3)) && ad[0] > (420 + (car_info.angle - 9) * (car_info.angle<9 ? 10 : 3)))
    {
        if(ad[3] > ad[2] && ad[3] > 160 + (car_info.angle - 9) * (car_info.angle<9 ? 10 : 3))
        {
            ring_dir = RIGHT;
            return 1;
        }
    }*/
    if(ad[0] > 600 && ad[1] > 580) //TODO: 由于电感原因，右环条件可能需要进一步调节
    {
        if(ad3_ad4_diff_reg[0] <= 0 && ad3_ad4_diff_reg[1] <= 0 && ad3_ad4_diff_reg[2] >=0 && ad3_ad4_diff_reg[3] >= 0 && ad[2] > 190)
        {
            ring_dir = LEFT;
            return 1;
        }
    }
    if(ad[0] > 580 && ad[1] > 600) //TODO: 由于电感原因，右环条件可能需要进一步调节
    {
        if(ad3_ad4_diff_reg[0] >= 0 && ad3_ad4_diff_reg[1] >= 0 && ad3_ad4_diff_reg[2] <=0 && ad3_ad4_diff_reg[3] <= 0 && ad[3] > 190)
        {
            ring_dir = RIGHT;
            return 1;
        }
    }
    for(i=0;i<3;++i)
    {
        ad3_ad4_diff_reg[i] = ad3_ad4_diff_reg[i+1];
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
        if(car_info.distance > 400)  // 移动超过25cm
            return 1;
        else
            return 0;
    }
    else
    {
        if(car_info.distance > 400)  // 移动超过10cm
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
    return 0;
    if(car_info.angle > 40)
        return 0;
    if(ad[4] > (induc_ref[4] + (0.37 * car_info.angle - 14) * car_info.angle) * 3 ) //已经乘了1.5
    {
        return 1;
    }
    else
    {
        return 0;
    } 
}

/***************************
 * @breif   判断终点
 * @param   void
 * @return  判出终点返回1，否则返回0
 * @note    
 ***************************/
uint8 is_terminal()
{
    uint8 i = 0;
    uint16 sum = 0;
    if (!tsl1401_finish_flag)
    {
        return 0;
    }
    else
    {
        for (i = 0; i < 125; ++i)
        {
            if((int16)(ccd_data[i+2] - ccd_data[i]) >= 100)
                sum += 1;
        }
        tsl1401_finish_flag = 0;  // 清除标志位
        return sum >= TERMINAL_THRESH;  // 判终点条件
    }
}