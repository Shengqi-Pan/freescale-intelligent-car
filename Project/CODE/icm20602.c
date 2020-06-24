/*******************************************
 * @file            icm20602
 * @note            基于逐飞ICM20602的进一步封装
 * @author          psq
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#include"icm20602.h"

// TODO: 陀螺仪零漂，先暂定为0，后续加入自动测定
GyroBias gyro_bais = {0, 0, 0};

/***************************
 * @breif   从icm读取角度值
 * @param   void
 * @return  车子的倾角(°)
 * @note    
 ***************************/
float get_angle_from_icm()
{
    static float angle = 0;
    float grav_x, grav_z;
    get_icm20602_accdata_simspi();  // 得到加速度计数据16位，如icm_acc_x
    grav_x = (icm_acc_x - ACC_X_BIAS) * 1.0 / ACC_X_FACTOR;  // 根据读到的值计算加速度大小
    grav_z = (icm_acc_z - ACC_Z_BIAS) * 1.0 / ACC_Z_FACTOR;
    angle = atan2(-grav_x, -grav_z) * 57.3;  // 实时更新角度
    // TODO: wwt是像下面这样逐步更新的，赋予了新读到的值比较小的权重
    // TODO: 目的是为了滤波，后期看看要不要加上
    // angle = (3 * angle + atan2(grav_x, grav_z) * 57.3) / 4;
    return angle;
}

/***************************
 * @breif   从icm读取角速度值
 * @param   void
 * @return  车子的角速度(dps)
 * @note    
 ***************************/
Omega get_omega_from_icm()
{
    static Omega omega = {0, 0};
    get_icm20602_gyro_simspi();  // 得到陀螺仪数据16位，如icm_gyro_x
    omega.y = (float)(-(icm_gyro_y - gyro_bais.y) / GYRO_Y_FACTOR);
    omega.z = (float)(-(icm_gyro_z - gyro_bais.z) / GYRO_Z_FACTOR);
    return omega;
}