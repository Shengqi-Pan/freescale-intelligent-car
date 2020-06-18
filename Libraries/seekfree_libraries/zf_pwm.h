/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		pwm
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ790875685)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C51 V9.60
 * @Target core		STC8H8K64S4
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-4-14
 ********************************************************************************************************************/

#ifndef __ZF_PWM_H
#define __ZF_PWM_H
#include "common.h"


#define PWM_DUTY_MAX 10000



typedef enum
{
	PWM1P_P10 = 0x00,PWM1N_P11,
	PWM1P_P20,		 PWM1N_P21,
	PWM1P_P60,		 PWM1N_P61,
	
	PWM2P_P12 = 0x10,PWM2N_P13,
	PWM2P_P22,		 PWM2N_P23,
	PWM2P_P62,		 PWM2N_P63,
	
	PWM3P_P14 = 0x20,PWM3N_P15,
	PWM3P_P24,		 PWM3N_P25,
	PWM3P_P64,		 PWM3N_P65,
	
	PWM4P_P16 = 0x30,PWM4N_P17,
	PWM4P_P26,		 PWM4N_P27,
	PWM4P_P66,		 PWM4N_P67,
	PWM4P_P34,		 PWM4N_P33,
	
	PWM5_P20 = 0x40,
	PWM5_P17,
	PWM5_P00,
	PWM5_P74,
	
	PWM6_P21 = 0x50,
	PWM6_P54,
	PWM6_P01,
	PWM6_P75,
	
	PWM7_P22 = 0x60,
	PWM7_P33,
	PWM7_P02,
	PWM7_P76,
	
	PWM8_P23 = 0x70,
	PWM8_P34,
	PWM8_P03,
	PWM8_P77,

}PWMCH_enum;


void pwm_init(PWMCH_enum pwmch,uint32 freq, uint16 duty);
void pwm_duty(PWMCH_enum pwmch, uint16 duty);
void pwm_freq(PWMCH_enum pwmch, uint32 freq, uint16 duty);


#endif
