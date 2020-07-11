/*******************************************
 * @file            icm20602
 * @note            基于逐飞ICM20602的进一步封装
 * @author          psq
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#include"icm20602.h"

static uint8 turn_angle_calc_flag = 0;

/***************************
 * @breif   从icm读取角度值
 * @param   void
 * @return  车子的倾角(°)
 * @note    
 ***************************/
float get_angle_from_icm()
{
    static float angle = 0;
    static float grav_x, grav_z;
    get_icm20602_accdata_simspi();  // 得到加速度计数据16位，如icm_acc_x
    grav_x = (icm_acc_x - ACC_X_BIAS) * 1.0 / ACC_X_FACTOR;  // 根据读到的值计算加速度大小
    grav_z = (icm_acc_z - ACC_Z_BIAS) * 1.0 / ACC_Z_FACTOR;
    angle = (3 * angle + atan2(grav_x, grav_z) * 57.3) / 4;;  // 实时更新角度
    // TODO: wwt是像下面这样逐步更新的，赋予了新读到的值比较小的权重
    // TODO: 目的是为了滤波，后期看看要不要加上
    // angle = (3 * angle + atan2(-grav_x, -grav_z) * 57.3) / 4;
    return angle;
}

/***************************
 * @breif   从icm读取角速度值
 * @param   void
 * @return  车子的角速度(dps)左负右正
 * @note    
 ***************************/
Omega get_omega_from_icm()
{
    static Omega omega = {0, 0};
    get_icm20602_gyro_simspi();  // 得到陀螺仪数据16位，如icm_gyro_x
    omega.y = (float)(-(icm_gyro_y - GYRO_Y_BIAS) / GYRO_Y_FACTOR);
    omega.z = (float)(-(icm_gyro_z - GYRO_Z_BIAS) / GYRO_Z_FACTOR);
    if(omega.z > 0)
        omega.y += 0.15 * omega.z;
    if(turn_angle_calc_flag)
    {
        // 对转向角速度进行积分，左转为负值，右转为正值
        car_info.turn_angle += omega.z / 1000 * 1.5 / cos(car_info.angle / 180 * 3.14);
    }
    return omega;
}

/***************************
 * @breif   开始对z轴角速度积分
 * @param   void
 * @return  void
 * @note    
 ***************************/
void start_turn_angle_calc()
{
    car_info.turn_angle = 0;
    turn_angle_calc_flag = 1;
}

/***************************
 * @breif   结束对z轴角速度积分
 * @param   void
 * @return  void
 * @note    
 ***************************/
void stop_turn_angle_calc()
{
    turn_angle_calc_flag = 0;
}