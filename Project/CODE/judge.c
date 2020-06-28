#include "judge.h"

uint8 is_ring()
{
    int16 ad3_ad4_diff = ad[2] - ad[3];
    if(sensor[0] > 1.1 * HENG_FACTOR && sensor[1] > 1.1 * HENG_FACTOR)
    {
        if(ad3_ad4_diff > 90) //两个电感差的绝对值大于90
        {
            ring_dir = LEFT;
            return 1;
        }
        else if(ad3_ad4_diff < -90)
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
    static flag = 0;

    if(flag == 0)
    {
        if(ring_dir == LEFT)
        {
            if(ad[2] < ad_last)
                --cnt;
            if(ad[2] > ad_last)
                cnt = 0;
            ad_last = ad[2];
            if(cnt <= -4)
                flag = 1;
        }
    }
    else if(flag == 1)
    {
        if(ring_dir == LEFT)
        {
            if(ad[2] < ad_last)
                cnt = 0;
            if(ad[2] > ad_last)
                ++cnt;
            ad_last = ad[2];
            if(cnt >= 4)
            {
                flag = 0;
                return 1;
            }
        }
    }
    return 0;
}