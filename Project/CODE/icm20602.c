/*******************************************
 * @note        基于逐飞ICM20602的进一步封装
 * 
 * 
 * 
 * 
 * 
 *******************************************/

#include"icm20602.h"

/*
 * 从icm读取角度值
 */
float get_angle_from_icm()
{
    static float degree = 0;
    float grav_z, grav_x;
    get_icm20602_accdata_simspi();  // 得到加速度计数据16位，如icm_acc_x
    grav_x = (icm_acc_x - ACC_X_BIAS) * 1.0 / ACC_X_FACTOR;
    grav_z = (icm_acc_z - ACC_Z_BIAS) * 1.0 / ACC_Z_FACTOR;
    degree = atan2(grav_x, grav_z) * 57.3;
    return degree;
    // degree = (39 * degree + atan2(icm_acc_x, icm_acc_z) * 57.3) / 40;
}

/*
 * 从icm读取角速度值
 */
float get_omega_from_icm()
{
    return 0;
}