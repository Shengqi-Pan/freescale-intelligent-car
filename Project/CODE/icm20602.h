/*******************************************
 * @note        基于逐飞ICM20602的进一步封装
 * 
 * 
 * 
 * 
 * 
 *******************************************/

#ifndef _ICM20602_H
#define _ICM20602_H

#include"SEEKFREE_ICM20602.h"
#include<math.h>

#define ACC_X_BIAS 30
#define ACC_Z_BIAS -80
#define ACC_X_FACTOR -4110
#define ACC_Z_FACTOR -4100

float get_angle_from_icm();
float get_omega_from_icm();

#endif