/*******************************************
 * @file            icm20602
 * @note            基于逐飞ICM20602的进一步封装
 * @author          psq
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#ifndef _ICM20602_H
#define _ICM20602_H

#include"SEEKFREE_ICM20602.h"
#include<math.h>

//--------------加速度计---------------//
#define ACC_X_BIAS 0
#define ACC_Z_BIAS 0
#define ACC_X_FACTOR -4110
#define ACC_Z_FACTOR -4100
//--------------陀螺仪---------------//
#define GYRO_Y_FACTOR 1.99
#define GYRO_Z_FACTOR 2.0583

//--------------角速度结构体---------------//
typedef struct
{
    float y;
    float z;
}Omega;

//--------------陀螺仪零漂结构体---------------//
typedef struct
{
    int32 x;
    int32 y;
    int32 z;
}GyroBias;

float get_angle_from_icm();
Omega get_omega_from_icm();

#endif