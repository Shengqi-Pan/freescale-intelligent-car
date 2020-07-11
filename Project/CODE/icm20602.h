/*******************************************
 * @file            icm20602
 * @note            基于逐飞ICM20602的进一步封装
 * @author          psq
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#ifndef _ICM20602_H
#define _ICM20602_H

#include "SEEKFREE_ICM20602.h"
#include <math.h>
#include "car_info.h"

//--------------加速度计---------------//
#define ACC_X_BIAS -30
#define ACC_Z_BIAS -30
#define ACC_X_FACTOR 4072
#define ACC_Z_FACTOR 4180
//--------------陀螺仪---------------//
#define GYRO_Y_BIAS -14
#define GYRO_Z_BIAS 25
#define GYRO_Y_FACTOR 16.28
#define GYRO_Z_FACTOR 16.22

float get_angle_from_icm();
Omega get_omega_from_icm();
void start_turn_angle_calc();
void stop_turn_angle_calc();

#endif