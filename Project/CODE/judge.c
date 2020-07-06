#include "judge.h"

uint8 is_ring()
{
    float ad3_ad4_add = ad[2] + ad[3];
    if(((float)ad[0] / (ad[0] + ad[1]) > 0.55 && ad[0] > 550) || (((float)ad[1] / (ad[0] + ad[1]) > 0.55) && ad[1] > 550)) //TODO: 由于电感原因，右环条件可能需要进一步调节
    {
        if(ad[2] / ad3_ad4_add > 0.65 && ad[2] > 250) //两个电感差的绝对值大于90,此条件可能过分宽松
        {
            ring_dir = LEFT;
            return 1;
        }
        else if(ad[3] / ad3_ad4_add > 0.65 && ad[3] > 250)
        {
            ring_dir = RIGHT;
            return 1;
        }
    }
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
    if(car_info.distance > 370)  // 移动超过40cm
        return 1;
    else
        return 0;
}
