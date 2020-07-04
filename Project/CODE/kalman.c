/*******************************************
 * @file            kalman
 * @note            对icm20602读到的角度和角速度信息进行卡尔曼滤波
 * @author          psq
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#include"kalman.h"

/***************************
 * @breif   对读取到的值进行卡尔曼滤波
 * @param   读取到的角度值angle和y轴角速度值omega
 * @return  车子的倾角
 * @note    
 ***************************/
void kalman(float angle, float omega)
{
    static float Q_angle = 0.001;     //2.26   0.001;
    static float Q_gyro = 0.003;
    static float R_angle = 0.5;  
    static float dt = 0.00125;   //绿色用0.00135
    static float P[2][2] = {{1, 0}, {0, 1}};
    static float Pdot[4] = {0, 0, 0, 0};
    static const char C_0 = 1;
    static float q_bias = 0;
    static float angle_err = 0;
    static float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    static float R;
    
    // if(begin_cnt < 2000) // || (Page!=5 && (flag_heng_ref==0 || flag_shu_ref==0)))
    // {
    //     R_angle = 0.5;
    // }
    // else
    //   R_angle = 80;
    // R = R_angle;
    R = R_angle;
    
    // 需要返回的车身角
    car_info.angle += (omega - q_bias) * dt;	                                                 
    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;
  	
    P[0][0] += Pdot[0] * dt; 
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    angle_err = angle - car_info.angle;
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0]; 	
    E = R_angle + C_0 * PCt_0;
  	
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;	
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    car_info.angle += K_0 * angle_err; 
    q_bias += K_1 * angle_err;

    // 需要返回的车身角速度
    car_info.omega.y = omega - q_bias;  
}