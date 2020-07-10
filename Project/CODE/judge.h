#ifndef _JUDGE_H
#define _JUDGE_H
#include "car_info.h"
#include "STC8Hxx.h"
#include "control.h"

uint8 is_ring(void);
uint8 is_tangent();
uint8 is_motor_tangent();
uint8 is_ramp();

#endif