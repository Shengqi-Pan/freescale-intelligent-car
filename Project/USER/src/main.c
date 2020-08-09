/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            main
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ790875685)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        MDK FOR C51 V9.60
 * @Target core     STC8H8K64S4
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-06-01
 ********************************************************************************************************************/

#include "headfile.h"
#include "icm20602.h"
#include "car_info.h"
#include "kalman.h"
#include "motor.h"
#include "encoder.h"
#include "control.h"
//board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�ʶ��ϵͳƵ��

/*board.h�ļ���FOSC��ֵ���ò�Ϊ0����ϵͳƵ��ΪFOSC��ֵ��
��ʹ��stc-isp�������س����ʱ����Ҫ��IRCƵ������ΪFOSC��ֵ*/

/*��board_init��,�Ѿ���P54��������Ϊ��λ��
�����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����*/
void main()
{
    DisableGlobalIRQ();  //  �ر����ж�
    board_init();  // ��ʼ���Ĵ���
    pit_timer_us(TIM_1, 1500);  // ʹ��TIMER��Ϊ�����жϣ�ʱ��1.5msһ��
    icm20602_init_simspi();  // icm20602��ʼ��, ���Ų鿴�궨��
    seekfree_wireless_init();  // ���ߴ��ڳ�ʼ��
    motor_init();  // �����ʼ��
    l_init();  // ad��ʼ��
    // adc_init(ADC_P15, ADC_SYSclk_DIV_2);
    encoder_init();  // ��������ʼ��
    ccd_init();  // ����ccd��ʼ��
    delay_ms(10);
    EnableGlobalIRQ(); //  �������ж�
    while(1)
    {
        // ��λ��ʾ�����鿴
        // data_conversion(ad[0], car_info.speed.left, car_info.speed.right, car_info.speed.average, virtual_scope_data);
        // uart_putbuff(WIRELESS_UART, virtual_scope_data, sizeof(virtual_scope_data));
        // seekfree_wireless_send_buff(virtual_scope_data, sizeof(virtual_scope_data));
        // ccd_send_data(WIRELESS_UART, ccd_data);
    }
}