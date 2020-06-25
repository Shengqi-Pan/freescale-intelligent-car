#ifndef _CAR_INFO_H
#define _CAR_INFO_H

#include "common.h"
//--------------角速度结构体---------------//
typedef struct
{
    // y轴前后角速度
    float y;
    // z轴左右
    float z;
}Omega;

//--------------车速结构体---------------//
typedef struct
{
    int16 left;
    int16 right;
}Speed;

//--------------车身相关信息的结构体---------------//
typedef struct
{
    // 车身倾角
    float angle;
    // 角速度
    Omega omega;
    // 车速
    Speed speed;
}CarInfo;

extern CarInfo car_info;

#endif