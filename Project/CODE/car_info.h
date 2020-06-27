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
    int16 average;
    int16 left_right_diff;  // 左右轮速度差
}Speed;

//--------------车辆当前状态---------------//
typedef enum
{
    TAKE_OFF,  // 起步
    STRAIGHT_AHEAD,  // 走直道
    INTO_TURN,  // 开始转
    IN_TURN,  // 弯中
    RAMP_UP,  // 上坡
    RAMP_DOWN,  // 下坡
    RING,  //圆环
    STOP  //结束
}CarState;

//--------------车身相关信息的结构体---------------//
typedef struct
{
    // 车身倾角
    float angle;
    // 角速度
    Omega omega;
    // 车速
    Speed speed;
    // 车辆状态
    CarState state;
}CarInfo;

extern CarInfo car_info;

#endif