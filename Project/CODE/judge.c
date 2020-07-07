#include "judge.h"

uint8 is_ring()
{
    static int16 ad3_ad4_diff;
    static int16 ad3_ad4_diff_reg[4];
    uint8 i;
    ad3_ad4_diff = ad[2] - ad[3];
    ad3_ad4_diff_reg[3] = ad3_ad4_diff;
    if(ad[1] > 520 && ad[0] > 550) //TODO: 由于电感原因，右环条件可能需要进一步调节
    {
        if(ad3_ad4_diff_reg[2] <= 50 && ad3_ad4_diff_reg[3] < 50 && ad3_ad4_diff_reg[0] > 50 && ad3_ad4_diff_reg[1] >= 50)
        {
            ring_dir = LEFT;
            return 1;
        }
    }
    if(ad[0] > 520 && ad[1] > 550)
    {
        if(ad3_ad4_diff_reg[2] >= 10 && ad3_ad4_diff_reg[3] > 10 && ad3_ad4_diff_reg[0] < 10 && ad3_ad4_diff_reg[1] <= 10)
        {
            ring_dir = RIGHT;
            return 1;
        }
    }
    for(i=0;i<3;i++)
        ad3_ad4_diff_reg[i] = ad3_ad4_diff_reg[i+1];
    return 0;
}


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

uint8 is_motor_tangent()
{   
    // int16 sensor0_sensor1_diff = sensor[0] - sensor[1];
    // int16 sensor1_sensor0_diff;
    // static int16 diff_last = 0;
    // sensor1_sensor0_diff = -sensor0_sensor1_diff;
    // if(ring_dir == LEFT)
    // {
    //     if(sensor0_sensor1_diff <= 2 && diff_last > 2)
    //     {
    //         return 1;
    //     }
    //     diff_last = sensor0_sensor1_diff;
    // }
    // else if(ring_dir == RIGHT)
    // {
    //     if(sensor1_sensor0_diff <= 2 && diff_last > 2)
    //     {
    //         return 1;
    //     }
    //     diff_last = sensor1_sensor0_diff;
    // }
    // return 0;    
    if(car_info.distance > 10)  // 移动超过1cm
        return 1;
    else
        return 0;
}
