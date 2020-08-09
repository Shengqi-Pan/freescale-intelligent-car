/*******************************************
 * @file            motor_encoder
 * @note            电机编码器驱动
 * @author          psq
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/
#ifndef _ENCODER_H
#define _ENCODER_H

#include "headfile.h"
#include "car_info.h"

// 左轮对应编码器1，右轮对应编码器2
// 定义脉冲引脚
#define SPEEDL_PLUSE   CTIM3_P04
#define SPEEDR_PLUSE   CTIM0_P34
// 定义方向引脚
#define SPEEDL_DIR     P51
#define SPEEDR_DIR     P50

void encoder_init(void);
Speed get_speed(uint16 time);
void start_distance_calc();
void stop_distance_calc();

#endif
