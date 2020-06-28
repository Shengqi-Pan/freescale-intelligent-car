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
#include "car_info.h"
#include "SEEKFREE_VIRSCO.h"
#include "SEEKFREE_WIRELESS.h"
#include "zf_uart.h"

#define ANGLE_CONTROL_P 1000
#define ANGLE_CONTROL_D 13
#define HENG_FACTOR 60
#define SHU_FACTOR 60
#define AMP_FACTOR 400

extern int16 ad_test[4];
float angle_control(float car_angle, float car_w, float angle_set);
void induc_test(void);
int16 direction_control(void);
float speed_control(int16 speed_real, int16 speed_set);
void take_off(void);
void direction_pd_fuzzy(int16 deviation, float *p, float *d);

#endif