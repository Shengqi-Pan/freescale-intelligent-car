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

void send_icm(void)
{
    uint8 high = icm_acc_x >> 8;
    uint8 low = (uint8)icm_acc_x;
    uart_putchar(UART_1, high);
    uart_putchar(UART_1, low);
}

void main()
{
    // �����ȡ������Ϣ
    float angle = 0;
    Omega omega = {0, 0};

    DisableGlobalIRQ(); //  �ر����ж�
    board_init(); //  ��ʼ���Ĵ���
    EnableGlobalIRQ(); //  �������ж�
    icm20602_init_simspi(); // icm20602��ʼ��, ���Ų鿴�궨��
    uart_init(UART_1, UART1_RX_P30, UART1_TX_P31, 115200, TIM_1);  // ����1��ʼ����������115200����������TX P31 ��������RX P30

    while(1)
    {
        // send_icm();                   // �����������ݣ�˳����Ӻ���
        angle = get_angle_from_icm();
        omega = get_omega_from_icm();
        kalman(angle, omega);
        delay_ms(100);
    }
}