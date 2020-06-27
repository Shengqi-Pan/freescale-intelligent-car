/*******************************************
 * @file            encoder
 * @note            电机编码器驱动
 * @author          psq
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/
#include "encoder.h"

/***************************
 * @breif   两个编码器初始化
 * @param   void
 * @return  void
 * @note    
 ***************************/
void encoder_init(void)
{
    ctimer_count_init(SPEEDL_PLUSE);	//初始化定时器0作为外部计数
    ctimer_count_init(SPEEDR_PLUSE);	//初始化定时器3作为外部计数
}

/***************************
 * @breif   通过编码器获得车速
 * @param   time    读取速度的间隔(ms)
 * @return  控速附加角
 * @note    
 * @author  psq
 ***************************/
Speed get_speed(uint16 time)
{
    int16 temp_pulse_left = 0, temp_pulse_right = 0;
    Speed speed;
    //读取采集到的编码器脉冲数
    temp_pulse_left = ctimer_count_read(SPEEDL_PLUSE);
    temp_pulse_right = ctimer_count_read(SPEEDR_PLUSE);

    //计数器清零
    ctimer_count_clean(SPEEDL_PLUSE);
    ctimer_count_clean(SPEEDR_PLUSE);

    //采集方向信息
    if(1 == SPEEDL_DIR)    temp_pulse_left = temp_pulse_left;
    else                   temp_pulse_left = -temp_pulse_left;
    if(1 == SPEEDR_DIR)    temp_pulse_right = -temp_pulse_right;
    else                   temp_pulse_right = temp_pulse_right;

    // temp_pulse_left / 1024.0(线数) * 30(编码器的齿轮数) / 68(轮子的齿轮数) * 0.064 * 3.1415926 单位m
    // 单位:mm/s
    speed.left = (int16)(temp_pulse_left * 86.6248 / time);
    speed.right = (int16)(temp_pulse_right * 86.6248 / time);
    speed.average = (speed.left + speed.right) / 2;
    speed.left_right_diff = speed.left - speed.right;
    speed.left_right_diff = speed.left_right_diff > 0 ? speed.left_right_diff : -speed.left_right_diff;  // 取abs
    // data_conversion(speed.left, speed.right,
    //                 temp_pulse_left, temp_pulse_right,
    //                 virtual_scope_data);
    return speed;
}