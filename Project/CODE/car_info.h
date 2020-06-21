#ifndef _CAR_INFO_H
#define _CAR_INFO_H

#include"icm20602.h"

//--------------车身相关信息的结构体---------------//
typedef struct
{
    // 车身倾角
    float angle;
    // 角速度
    Omega omega;
    // // 陀螺仪零偏
    // GyroBias gyro_bais;
}CarInfo;

#endif