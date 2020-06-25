/*******************************************
 * @file            control
 * @note            控制相关的函数，包括直立，速度，转向
 * @author          psq&btk
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#ifndef _CONTROL_H
#define _CONTROL_H

#include "l_ad.h"

#define ANGLE_CONTROL_P 2000
#define ANGLE_CONTROL_D 50
#define HENG_FACTOR 300
#define SHU_FACTOR 400
#define DIRECTION_ON 1
#define AMP_FACTOR 800
extern int16 ad_test[4];

float angle_control(float car_angle, float car_w, float angle_set);
void induc_test(void);

#endif