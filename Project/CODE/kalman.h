/*******************************************
 * @file            kalman
 * @note            对icm20602读到的角度和角速度信息进行卡尔曼滤波
 * @author          psq
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#ifndef _KALMAN_H
#define _KALMAN_H

#include "icm20602.h"
#include "car_info.h"

extern CarInfo car_info;

void kalman(float gsensor_angle, Omega gyro_w);

#endif