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

//board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�ʶ��ϵͳƵ��

/*board.h�ļ���FOSC��ֵ���ò�Ϊ0����ϵͳƵ��ΪFOSC��ֵ��
��ʹ��stc-isp�������س����ʱ����Ҫ��IRCƵ������ΪFOSC��ֵ*/

/*��board_init��,�Ѿ���P54��������Ϊ��λ��
�����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����*/

// ���泵����ǰ����Ϣ
extern CarInfo car_info;
float angle = 0;
Omega omega = {0, 0};
uint8 tim4_flag = 0;
float test[] = {0, 0, 0};

void main()
{
    // �����ȡ������Ϣ

    DisableGlobalIRQ(); //  �ر����ж�
    board_init(); //  ��ʼ���Ĵ���
    pit_timer_ms(TIM_0, 1); // ʹ��TIMER��Ϊ�����жϣ�ʱ��1msһ��
                            // ����1000���ж� ��תһ��LED��Ҳ����1000MS ��תһ��LED
    icm20602_init_simspi(); // icm20602��ʼ��, ���Ų鿴�궨��
    // uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, 115200, TIM_1);  // ����1��ʼ����������115200����������TX P31 ��������RX P30
    seekfree_wireless_init();
    delay_ms(10);
    EnableGlobalIRQ(); //  �������ж�

    while(1)
    {
        data_conversion((int16)angle, (int16)omega.y,
                        (int16)car_info.angle, (int16)car_info.omega.y,
                        virtual_scope_data);
        uart_putbuff(WIRELESS_UART, virtual_scope_data, sizeof(virtual_scope_data));
    }
}