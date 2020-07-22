/*******************************************
 * @file            judge.c
 * @note            判圆环，判坡等函数
 * @author          psq & btk
 * @software        MDK
 * @target core     STC8H8K64S4
 *******************************************/

#ifndef _JUDGE_H
#define _JUDGE_H
#include "car_info.h"
#include "STC8Hxx.h"
#include "control.h"
#include "SEEKFREE_TSL1401.h"

#define BIN_THRESH 100
#define TERMINAL_THRESH 40

uint8 is_ring();
uint8 is_tangent();
uint8 is_motor_tangent();
uint8 is_ramp();
uint8 is_terminal();

#endif